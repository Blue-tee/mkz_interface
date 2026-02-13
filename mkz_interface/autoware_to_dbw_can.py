#!/usr/bin/env python3
"""
autoware_to_dbw_can.py
Bridge: Autoware vehicle_cmd_gate outputs -> Dataspeed DBW1 commands (Lincoln MKZ 2018, DBW1)

Key behaviors:
- /vehicle/enable and /vehicle/disable are std_msgs/Empty
- Throttle/Brake published as CMD_PERCENT
- Steering: Autoware steering_tire_angle (rad) -> DBW steering_wheel_angle_cmd (rad) via ratio
- Hazards accepted but NOT actuated (publish a shim latch only)
- Turn signals: burst-mode (avoid continuous spamming)

Gear change interlock (optional):
- Apply fixed brake %, wait for near-zero speed, send gear cmd, hold brake post-shift.
- Uses /vehicle/steering_report speed (m/s) and /vehicle/gear_report for confirmation.

NEW (Intelligent steering velocity, safe):
- Uses DBW SteeringReport.steering_wheel_angle (rad) + speed (m/s).
- Computes steering_wheel_angle_velocity as a function of:
    (a) wheel-angle error (target - measured)
    (b) vehicle speed (caps reduce at higher speeds)
- Adds smoothing + dv/dt limiting so it won’t chatter or spike.
- Can be toggled with steer_vel_enable.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import Bool, Empty, UInt8

from autoware_control_msgs.msg import Control as AwControl
from autoware_vehicle_msgs.msg import Engage as AwEngage
from autoware_vehicle_msgs.msg import GearCommand as AwGearCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand as AwTurnCmd
from autoware_vehicle_msgs.msg import HazardLightsCommand as AwHazCmd

from dbw_ford_msgs.msg import (
    ThrottleCmd,
    BrakeCmd,
    SteeringCmd,
    GearCmd,
    MiscCmd,
    SteeringReport as DbwSteeringReport,
    GearReport as DbwGearReport,
)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _const(msg_cls, name: str, default: int) -> int:
    return int(getattr(msg_cls, name, default))


@dataclass
class _LastControl:
    t_wall: float = 0.0
    accel_mps2: float = 0.0
    tire_angle_rad: float = 0.0


class AutowareToDbwCan(Node):
    # Shift state
    _SHIFT_IDLE = 0
    _SHIFT_PRE_BRAKE = 1
    _SHIFT_SENT_CMD = 2
    _SHIFT_POST_HOLD = 3

    def __init__(self) -> None:
        super().__init__("autoware_to_dbw_can")

        # QoS
        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        qos_gate = QoSProfile(depth=1)
        qos_gate.history = QoSHistoryPolicy.KEEP_LAST
        qos_gate.reliability = QoSReliabilityPolicy.RELIABLE
        qos_gate.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # ---- parameters ----
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("watchdog_ms", 100)

        # Autoware topics
        self.declare_parameter("aw_control_cmd", "/vehicle_cmd_gate/output/command/control_cmd")
        self.declare_parameter("aw_gear_cmd", "/vehicle_cmd_gate/output/command/gear_cmd")
        self.declare_parameter("aw_turn_cmd", "/vehicle_cmd_gate/output/command/turn_indicators_cmd")
        self.declare_parameter("aw_hazard_cmd", "/vehicle_cmd_gate/output/command/hazard_lights_cmd")
        self.declare_parameter("aw_engage", "/vehicle_cmd_gate/output/engage")

        # DBW command topics
        self.declare_parameter("dbw_throttle_cmd", "/vehicle/throttle_cmd")
        self.declare_parameter("dbw_brake_cmd", "/vehicle/brake_cmd")
        self.declare_parameter("dbw_steering_cmd", "/vehicle/steering_cmd")
        self.declare_parameter("dbw_gear_cmd", "/vehicle/gear_cmd")
        self.declare_parameter("dbw_misc_cmd", "/vehicle/misc_cmd")

        # DBW enable/disable topics (std_msgs/Empty)
        self.declare_parameter("dbw_enable_topic", "/vehicle/enable")
        self.declare_parameter("dbw_disable_topic", "/vehicle/disable")
        self.declare_parameter("dbw_dbw_enabled", "/vehicle/dbw_enabled")

        # DBW reports
        self.declare_parameter("dbw_steering_report", "/vehicle/steering_report")  # speed + steering_wheel_angle
        self.declare_parameter("dbw_gear_report", "/vehicle/gear_report")

        # Hazard shim
        self.declare_parameter("hazard_shim_topic", "/mkz_interface/hazard_cmd_latched")
        self.declare_parameter("hazard_shim_timeout_ms", 2000)

        # Safety / gating
        self.declare_parameter("require_engage", True)
        self.declare_parameter("require_dbw_enabled", True)
        self.declare_parameter("auto_enable_on_engage", False)
        self.declare_parameter("publish_safe_zero_when_not_ready", True)

        # Vehicle constants
        self.declare_parameter("steering_wheel_to_tire_ratio", 14.8)
        self.declare_parameter("max_steering_wheel_angle_deg", 470.0)

        # Steering sign (+1 keeps Autoware convention, -1 flips)
        self.declare_parameter("steering_sign", 1.0)

        # Legacy fixed steering velocity (used if steer_vel_enable=false)
        self.declare_parameter("steering_wheel_angle_velocity", 3.0)

        # Intelligent steering velocity (safe defaults)
        self.declare_parameter("steer_vel_enable", True)
        self.declare_parameter("steer_vel_min", 3.0)          # rad/s minimum
        self.declare_parameter("steer_vel_gain", 3.0)         # rad/s per rad of wheel-angle error
        self.declare_parameter("steer_vel_tau", 0.25)         # seconds, low-pass on velocity command
        self.declare_parameter("steer_vel_dv_max", 10.0)      # rad/s^2 max change rate of velocity
        self.declare_parameter("steer_vel_cap_speeds", [0.0, 2.0, 10.0, 25.0, 35.0])  # m/s
        self.declare_parameter("steer_vel_cap_vels",   [10.0, 9.0,  7.0,  5.0,  4.5]) # rad/s
        self.declare_parameter("report_fresh_ms", 300)  # shared freshness for reports (speed/steer/gear)

        # Longitudinal mapping (accel -> percent)
        self.declare_parameter("accel_to_throttle_gain", 0.25)
        self.declare_parameter("accel_to_brake_gain", 0.30)
        self.declare_parameter("max_throttle", 0.25)
        self.declare_parameter("max_brake", 0.40)
        self.declare_parameter("throttle_deadband", 0.01)
        self.declare_parameter("brake_deadband", 0.01)

        # Gear interlock (brake-to-shift)
        self.declare_parameter("gear_change_requires_brake", True)
        self.declare_parameter("gear_change_brake_percent", 0.70)
        self.declare_parameter("gear_post_shift_hold_ms", 1000)
        self.declare_parameter("gear_change_speed_thresh", 0.10)     # m/s
        self.declare_parameter("gear_change_timeout_ms", 1500)
        self.declare_parameter("gear_report_match_required", True)

        # ---- pull params ----
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.watchdog_ms = int(self.get_parameter("watchdog_ms").value)

        self.require_engage = bool(self.get_parameter("require_engage").value)
        self.require_dbw_enabled = bool(self.get_parameter("require_dbw_enabled").value)
        self.auto_enable_on_engage = bool(self.get_parameter("auto_enable_on_engage").value)
        self.safe_zero = bool(self.get_parameter("publish_safe_zero_when_not_ready").value)

        self.ratio = float(self.get_parameter("steering_wheel_to_tire_ratio").value)
        self.max_swa_rad = float(self.get_parameter("max_steering_wheel_angle_deg").value) * 3.1415926535 / 180.0
        self.steering_sign = float(self.get_parameter("steering_sign").value)

        self.swa_vel_fixed = float(self.get_parameter("steering_wheel_angle_velocity").value)

        self.steer_vel_enable = bool(self.get_parameter("steer_vel_enable").value)
        self.steer_vel_min = float(self.get_parameter("steer_vel_min").value)
        self.steer_vel_gain = float(self.get_parameter("steer_vel_gain").value)
        self.steer_vel_tau = float(self.get_parameter("steer_vel_tau").value)
        self.steer_vel_dv_max = float(self.get_parameter("steer_vel_dv_max").value)
        self.steer_vel_cap_speeds = list(self.get_parameter("steer_vel_cap_speeds").value)
        self.steer_vel_cap_vels = list(self.get_parameter("steer_vel_cap_vels").value)

        self.report_fresh_ms = int(self.get_parameter("report_fresh_ms").value)
        self.hazard_shim_timeout_ms = int(self.get_parameter("hazard_shim_timeout_ms").value)

        self.k_th = float(self.get_parameter("accel_to_throttle_gain").value)
        self.k_br = float(self.get_parameter("accel_to_brake_gain").value)
        self.max_th = float(self.get_parameter("max_throttle").value)
        self.max_br = float(self.get_parameter("max_brake").value)
        self.db_th = float(self.get_parameter("throttle_deadband").value)
        self.db_br = float(self.get_parameter("brake_deadband").value)

        self.gear_change_requires_brake = bool(self.get_parameter("gear_change_requires_brake").value)
        self.gear_change_brake_percent = float(self.get_parameter("gear_change_brake_percent").value)
        self.gear_post_shift_hold_ms = int(self.get_parameter("gear_post_shift_hold_ms").value)
        self.gear_change_speed_thresh = float(self.get_parameter("gear_change_speed_thresh").value)
        self.gear_change_timeout_ms = int(self.get_parameter("gear_change_timeout_ms").value)
        self.gear_report_match_required = bool(self.get_parameter("gear_report_match_required").value)

        # Sanity: cap arrays must match
        if len(self.steer_vel_cap_speeds) != len(self.steer_vel_cap_vels) or len(self.steer_vel_cap_speeds) < 2:
            self.get_logger().warn("steer_vel_cap_* arrays invalid; falling back to fixed cap (6 rad/s).")
            self.steer_vel_cap_speeds = [0.0, 35.0]
            self.steer_vel_cap_vels = [6.0, 6.0]

        # ---- pubs ----
        self.pub_thr = self.create_publisher(ThrottleCmd, self.get_parameter("dbw_throttle_cmd").value, qos)
        self.pub_brk = self.create_publisher(BrakeCmd, self.get_parameter("dbw_brake_cmd").value, qos)
        self.pub_str = self.create_publisher(SteeringCmd, self.get_parameter("dbw_steering_cmd").value, qos)
        self.pub_gear = self.create_publisher(GearCmd, self.get_parameter("dbw_gear_cmd").value, qos)
        self.pub_misc = self.create_publisher(MiscCmd, self.get_parameter("dbw_misc_cmd").value, qos)

        self.pub_enable = self.create_publisher(Empty, self.get_parameter("dbw_enable_topic").value, qos)
        self.pub_disable = self.create_publisher(Empty, self.get_parameter("dbw_disable_topic").value, qos)

        self.pub_hazard_shim = self.create_publisher(UInt8, self.get_parameter("hazard_shim_topic").value, qos)

        # ---- subs ----
        self.create_subscription(AwEngage, self.get_parameter("aw_engage").value, self._on_engage, qos)
        self.create_subscription(AwControl, self.get_parameter("aw_control_cmd").value, self._on_control, qos_gate)
        self.create_subscription(AwGearCommand, self.get_parameter("aw_gear_cmd").value, self._on_gear_cmd, qos)
        self.create_subscription(AwTurnCmd, self.get_parameter("aw_turn_cmd").value, self._on_turn, qos)
        self.create_subscription(AwHazCmd, self.get_parameter("aw_hazard_cmd").value, self._on_hazard, qos)
        self.create_subscription(Bool, self.get_parameter("dbw_dbw_enabled").value, self._on_dbw_enabled, qos_gate)

        self.create_subscription(DbwSteeringReport, self.get_parameter("dbw_steering_report").value, self._on_steering_report, qos)
        self.create_subscription(DbwGearReport, self.get_parameter("dbw_gear_report").value, self._on_gear_report, qos)

        # ---- state ----
        self.engaged = False
        self.dbw_enabled = False

        self.ctrl = _LastControl()
        self._count = 0

        # hazard shim state
        self.last_hazard = 0
        self.last_hazard_t = 0.0

        # Turn policy: publish MiscCmd only in short bursts
        self._aw_turn_cmd = 1
        self._hazards_continuous = False  # hazards not actuated
        self._turn_burst_until = None
        self._turn_value = 0  # 0 none, 1 left, 2 right

        # DBW report caches
        self.last_speed_mps = 0.0
        self.last_speed_t = 0.0
        self.last_wheel_angle_rad = 0.0
        self.last_wheel_angle_t = 0.0
        self.last_dbw_gear = 0
        self.last_dbw_gear_t = 0.0

        # Intelligent steer velocity internal state
        self._steer_vel_cmd = float(_clamp(self.swa_vel_fixed, 0.0, 17.5))
        self._steer_vel_t = time.time()

        # Shift FSM
        self.shift_state = self._SHIFT_IDLE
        self.shift_target_dbw_gear = 0
        self.shift_t0 = 0.0
        self.shift_t_phase = 0.0
        self.shift_cmd_sent = False

        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._on_timer)

        self.get_logger().info(
            "MKZ cmd bridge ready. Throttle/Brake CMD_PERCENT. Turn signals burst-mode. "
            f"Gear interlock: {'ON' if self.gear_change_requires_brake else 'OFF'}. "
            f"Intelligent steer vel: {'ON' if self.steer_vel_enable else 'OFF'}."
        )

    # -------- small utils --------
    def _bump(self) -> int:
        self._count = (self._count + 1) & 0xFF
        return self._count

    def _stamp(self, msg) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

    def _ready(self) -> bool:
        if self.require_engage and (not self.engaged):
            return False
        if self.require_dbw_enabled and (not self.dbw_enabled):
            return False
        return True

    def _control_fresh(self, now: float) -> bool:
        return self.ctrl.t_wall > 0.0 and (now - self.ctrl.t_wall) * 1000.0 <= float(self.watchdog_ms)

    def _report_fresh(self, t_wall: float) -> bool:
        if t_wall <= 0.0:
            return False
        return (time.time() - t_wall) * 1000.0 <= float(self.report_fresh_ms)

    # -------- callbacks --------
    def _on_engage(self, msg: AwEngage) -> None:
        new_engaged = bool(msg.engage)
        if new_engaged and (not self.engaged) and self.auto_enable_on_engage:
            self.pub_enable.publish(Empty())
        if (not new_engaged) and self.engaged and self.auto_enable_on_engage:
            self.pub_disable.publish(Empty())
        self.engaged = new_engaged

    def _on_dbw_enabled(self, msg: Bool) -> None:
        self.dbw_enabled = bool(msg.data)
        if not self.dbw_enabled and self.shift_state != self._SHIFT_IDLE:
            self.get_logger().warn("DBW disabled during shift; aborting shift FSM.")
            self._shift_abort()

    def _on_control(self, msg: AwControl) -> None:
        self.ctrl.accel_mps2 = float(msg.longitudinal.acceleration)
        self.ctrl.tire_angle_rad = float(msg.lateral.steering_tire_angle)
        self.ctrl.t_wall = time.time()

    def _on_turn(self, msg: AwTurnCmd) -> None:
        self._aw_turn_cmd = int(getattr(msg, "command", 0))
        self._recalc_turn_policy()

    def _on_hazard(self, msg: AwHazCmd) -> None:
        # Hazards not actuated; keep shim only
        self.last_hazard = int(msg.command)
        self.last_hazard_t = time.time()
        u = UInt8()
        u.data = int(self.last_hazard)
        self.pub_hazard_shim.publish(u)

        # Ensure hazard does NOT affect turn behavior
        self._hazards_continuous = False

    def _on_steering_report(self, msg: DbwSteeringReport) -> None:
        # DBW SteeringReport has:
        # - steering_wheel_angle (rad)
        # - speed (m/s)
        try:
            self.last_speed_mps = float(msg.speed)
        except Exception:
            self.last_speed_mps = 0.0
        self.last_speed_t = time.time()

        try:
            self.last_wheel_angle_rad = float(msg.steering_wheel_angle)
        except Exception:
            # keep previous
            pass
        self.last_wheel_angle_t = time.time()

    def _on_gear_report(self, msg: DbwGearReport) -> None:
        g = 0
        try:
            g = int(msg.state.gear)
        except Exception:
            try:
                g = int(msg.state)
            except Exception:
                g = 0
        self.last_dbw_gear = g
        self.last_dbw_gear_t = time.time()

    def _on_gear_cmd(self, msg: AwGearCommand) -> None:
        desired_dbw = int(self._map_aw_gear_to_dbw(int(msg.command)))
        if desired_dbw == 0:
            return

        if not self.gear_change_requires_brake:
            self._publish_gear_cmd(desired_dbw)
            return

        if not self._ready():
            self.get_logger().warn("Ignoring gear command (not ready: engage/dbw_enabled gating).")
            return

        if self.last_dbw_gear == desired_dbw and self._report_fresh(self.last_dbw_gear_t):
            return

        # If already shifting, retarget
        if self.shift_state != self._SHIFT_IDLE:
            if desired_dbw != self.shift_target_dbw_gear:
                self.get_logger().warn(
                    f"New gear request while shifting: retarget {self.shift_target_dbw_gear} -> {desired_dbw}"
                )
                self.shift_target_dbw_gear = desired_dbw
                self.shift_t0 = time.time()
                self.shift_t_phase = self.shift_t0
                self.shift_state = self._SHIFT_PRE_BRAKE
                self.shift_cmd_sent = False
            return

        self.shift_target_dbw_gear = desired_dbw
        self.shift_t0 = time.time()
        self.shift_t_phase = self.shift_t0
        self.shift_state = self._SHIFT_PRE_BRAKE
        self.shift_cmd_sent = False
        self.get_logger().info(f"Starting brake-to-shift: target_dbw_gear={desired_dbw}")

    # -------- turn burst helpers --------
    def _recalc_turn_policy(self) -> None:
        # Your working values: 1=DISABLE, 2=LEFT, 3=RIGHT (0 may be NO_COMMAND)
        now = time.time()
        self._hazards_continuous = False

        c = int(self._aw_turn_cmd)
        if c == 2:
            self._turn_value = 1  # LEFT
            self._turn_burst_until = now + 0.5
        elif c == 3:
            self._turn_value = 2  # RIGHT
            self._turn_burst_until = now + 0.5
        else:
            self._turn_value = 0  # OFF
            self._turn_burst_until = now + 0.2

    def _publish_misc_if_needed(self, now: float) -> None:
        send_value = None
        if self._hazards_continuous:
            send_value = 3  # HAZARD (not used)
        elif self._turn_burst_until is not None and now < float(self._turn_burst_until):
            send_value = int(self._turn_value)
        else:
            self._turn_burst_until = None

        if send_value is None:
            return

        m = MiscCmd()
        self._stamp(m)
        m.cmd.value = int(send_value)  # 0 NONE, 1 LEFT, 2 RIGHT
        m.pbrk.cmd = 0
        self.pub_misc.publish(m)

    # -------- intelligent steering velocity helpers --------
    def _interp_cap(self, speed_mps: float) -> float:
        # Piecewise-linear interpolation over (cap_speeds, cap_vels)
        s = float(speed_mps)
        xs = self.steer_vel_cap_speeds
        ys = self.steer_vel_cap_vels

        if s <= float(xs[0]):
            return float(ys[0])
        if s >= float(xs[-1]):
            return float(ys[-1])

        for i in range(len(xs) - 1):
            x0 = float(xs[i])
            x1 = float(xs[i + 1])
            if x0 <= s <= x1:
                y0 = float(ys[i])
                y1 = float(ys[i + 1])
                if abs(x1 - x0) < 1e-9:
                    return float(min(y0, y1))
                t = (s - x0) / (x1 - x0)
                return float(y0 + t * (y1 - y0))

        return float(ys[-1])

    def _compute_steer_velocity(self, now: float, target_wheel_angle: float) -> float:
        # Fallback to fixed velocity if disabled
        if not self.steer_vel_enable:
            return float(_clamp(self.swa_vel_fixed, 0.0, 17.5))

        # Use measured wheel angle if fresh, else fall back to last commanded target
        if self._report_fresh(self.last_wheel_angle_t):
            wheel_cur = float(self.last_wheel_angle_rad)
        else:
            # If we don't have fresh feedback, be conservative: assume we are at previous target.
            wheel_cur = float(target_wheel_angle)

        # Speed (fresh -> use; else conservative mid-speed cap)
        if self._report_fresh(self.last_speed_t):
            spd = float(abs(self.last_speed_mps))
        else:
            spd = 10.0  # conservative assumption

        cap = float(_clamp(self._interp_cap(spd), 0.0, 17.5))

        err = abs(float(target_wheel_angle) - float(wheel_cur))
        v_req = float(self.steer_vel_min + self.steer_vel_gain * err)
        v_req = float(_clamp(v_req, self.steer_vel_min, cap))

        # Smooth + dv/dt limit
        dt = max(1e-3, float(now - self._steer_vel_t))
        self._steer_vel_t = float(now)

        # First-order low-pass on velocity command
        tau = max(1e-3, float(self.steer_vel_tau))
        alpha = dt / (tau + dt)
        v_filt = float(self._steer_vel_cmd + alpha * (v_req - self._steer_vel_cmd))

        # dv/dt limiting
        dv_max = max(0.0, float(self.steer_vel_dv_max)) * dt
        v_limited = float(_clamp(v_filt, self._steer_vel_cmd - dv_max, self._steer_vel_cmd + dv_max))

        self._steer_vel_cmd = float(_clamp(v_limited, self.steer_vel_min, cap))
        return float(_clamp(self._steer_vel_cmd, 0.0, 17.5))

    # -------- DBW cmd builders --------
    def _build_throttle(self, percent: float) -> ThrottleCmd:
        m = ThrottleCmd()
        self._stamp(m)
        m.pedal_cmd_type = _const(ThrottleCmd, "CMD_PERCENT", 2)
        m.pedal_cmd = float(_clamp(percent, 0.0, 1.0))
        m.enable = True
        m.clear = False
        m.ignore = False
        m.count = self._bump()
        return m

    def _build_brake(self, percent: float) -> BrakeCmd:
        m = BrakeCmd()
        self._stamp(m)
        m.pedal_cmd_type = _const(BrakeCmd, "CMD_PERCENT", 2)
        m.pedal_cmd = float(_clamp(percent, 0.0, 1.0))
        m.enable = True
        m.clear = False
        m.ignore = False
        m.count = self._bump()
        return m

    def _build_steering(self, now: float, tire_angle_rad: float) -> SteeringCmd:
        # Autoware convention: positive tire_angle is LEFT (in FLU). steering_sign allows flipping if needed.
        wheel_target = float(self.steering_sign) * float(tire_angle_rad) * float(self.ratio)
        wheel_target = float(_clamp(wheel_target, -self.max_swa_rad, self.max_swa_rad))

        vel = self._compute_steer_velocity(now, wheel_target)

        m = SteeringCmd()
        self._stamp(m)
        m.enable = True
        m.clear = False
        m.ignore = False
        m.cmd_type = _const(SteeringCmd, "CMD_ANGLE", 0)
        m.steering_wheel_angle_cmd = float(wheel_target)
        m.steering_wheel_angle_velocity = float(vel)
        m.count = self._bump()
        return m

    def _publish_gear_cmd(self, desired_dbw: int) -> None:
        g = GearCmd()
        self._stamp(g)
        g.cmd.gear = int(desired_dbw)
        g.clear = False
        self.pub_gear.publish(g)

    # -------- shift FSM --------
    def _shift_abort(self) -> None:
        self.shift_state = self._SHIFT_IDLE
        self.shift_target_dbw_gear = 0
        self.shift_t0 = 0.0
        self.shift_t_phase = 0.0
        self.shift_cmd_sent = False

    def _run_shift(self, now: float) -> None:
        if not self._ready():
            self.get_logger().warn("Not ready during shift; aborting shift.")
            self._shift_abort()
            return

        if (now - self.shift_t0) * 1000.0 > float(self.gear_change_timeout_ms):
            self.get_logger().warn("Shift timed out; aborting shift.")
            self._shift_abort()
            return

        # Always override longitudinal during shift
        brk = float(_clamp(self.gear_change_brake_percent, 0.0, 1.0))
        self.pub_thr.publish(self._build_throttle(0.0))
        self.pub_brk.publish(self._build_brake(brk))

        # Keep steering based on last control if fresh
        tire = self.ctrl.tire_angle_rad if self._control_fresh(now) else 0.0
        self.pub_str.publish(self._build_steering(now, tire))

        # Turn signals: burst-only
        self._publish_misc_if_needed(now)

        speed_ok = False
        if self._report_fresh(self.last_speed_t):
            speed_ok = abs(self.last_speed_mps) <= float(self.gear_change_speed_thresh)

        if self.shift_state == self._SHIFT_PRE_BRAKE:
            if speed_ok:
                self.shift_state = self._SHIFT_SENT_CMD
                self.shift_t_phase = now
                self.shift_cmd_sent = False
            else:
                return

        if self.shift_state == self._SHIFT_SENT_CMD:
            if not self.shift_cmd_sent:
                self._publish_gear_cmd(self.shift_target_dbw_gear)
                self.shift_cmd_sent = True
                self.shift_t_phase = now

            if self.gear_report_match_required and self._report_fresh(self.last_dbw_gear_t):
                if self.last_dbw_gear == self.shift_target_dbw_gear:
                    self.shift_state = self._SHIFT_POST_HOLD
                    self.shift_t_phase = now
                else:
                    return
            else:
                self.shift_state = self._SHIFT_POST_HOLD
                self.shift_t_phase = now

        if self.shift_state == self._SHIFT_POST_HOLD:
            if (now - self.shift_t_phase) * 1000.0 >= float(self.gear_post_shift_hold_ms):
                self.get_logger().info("Shift complete (post-hold done).")
                self._shift_abort()

    # -------- main loop --------
    def _on_timer(self) -> None:
        now = time.time()

        # keep hazard shim alive for late subscribers
        if self.last_hazard_t > 0.0 and (now - self.last_hazard_t) * 1000.0 < self.hazard_shim_timeout_ms:
            u = UInt8()
            u.data = int(self.last_hazard)
            self.pub_hazard_shim.publish(u)

        # Run shift FSM if active
        if self.shift_state != self._SHIFT_IDLE:
            self._run_shift(now)
            return

        # watchdog on control_cmd
        if not self._control_fresh(now):
            if self.safe_zero:
                self.pub_thr.publish(self._build_throttle(0.0))
                self.pub_brk.publish(self._build_brake(0.0))
                self.pub_str.publish(self._build_steering(now, 0.0))
                self._publish_misc_if_needed(now)
            return

        if not self._ready():
            if self.safe_zero:
                self.pub_thr.publish(self._build_throttle(0.0))
                self.pub_brk.publish(self._build_brake(0.0))
                self.pub_str.publish(self._build_steering(now, 0.0))
                self._publish_misc_if_needed(now)
            return

        a = self.ctrl.accel_mps2

        thr = 0.0
        brk = 0.0
        if a >= 0.0:
            thr = _clamp(a * self.k_th, 0.0, self.max_th)
            if thr < self.db_th:
                thr = 0.0
        else:
            brk = _clamp((-a) * self.k_br, 0.0, self.max_br)
            if brk < self.db_br:
                brk = 0.0

        self.pub_thr.publish(self._build_throttle(thr))
        self.pub_brk.publish(self._build_brake(brk))
        self.pub_str.publish(self._build_steering(now, self.ctrl.tire_angle_rad))

        # Turn signals: only publish MiscCmd in short bursts
        self._publish_misc_if_needed(now)

    @staticmethod
    def _map_aw_gear_to_dbw(aw_gear: int) -> int:
        # Mapping kept consistent with your previous bridge:
        # 22=P, (20,21)=R/N treated as R, 1=D, 2..19=LOW, 23/24=extra -> 5
        if aw_gear == 22:
            return 1  # PARK
        if aw_gear in (20, 21):
            return 2  # REVERSE (neutral treated as reverse)
        if aw_gear == 1:
            return 3  # DRIVE
        if 2 <= aw_gear <= 19:
            return 4  # LOW
        if aw_gear in (23, 24):
            return 5
        return 0


def main() -> None:
    rclpy.init()
    node = AutowareToDbwCan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
