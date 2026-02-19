#!/usr/bin/env python3
"""
autoware_to_dbw_can.py
Bridge: Autoware vehicle_cmd_gate outputs -> Dataspeed DBW1 commands (Lincoln MKZ 2018, DBW1)

Behaviors kept from your existing bridge:
- /vehicle/enable and /vehicle/disable are std_msgs/Empty
- Throttle/Brake published as CMD_PERCENT
- Steering: Autoware steering_tire_angle (rad) -> DBW steering_wheel_angle_cmd (rad) via ratio
- Hazards accepted but NOT actuated (publish a shim latch only)
- Turn signals:
    * DBW expects periodic refresh (keepalive). We hold LEFT/RIGHT and publish at turn_keepalive_hz.
    * When OFF is requested, publish 0 for turn_off_burst_ms then stop.

Gear change interlock:
- Apply fixed brake %, wait for near-zero speed, send gear cmd, hold brake post-shift.
- Uses /vehicle/steering_report speed (m/s) and /vehicle/gear_report for confirmation.

Auto-park:
- When /planning/mission_planning/state == ARRIVED and vehicle speed ~0 for a dwell window, shift to PARK.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, Empty, UInt8

from autoware_control_msgs.msg import Control as AwControl
from autoware_vehicle_msgs.msg import Engage as AwEngage
from autoware_vehicle_msgs.msg import GearCommand as AwGearCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand as AwTurnCmd
from autoware_vehicle_msgs.msg import HazardLightsCommand as AwHazCmd

from autoware_planning_msgs.msg import RouteState

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
    # Shift FSM
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
        self.declare_parameter("report_fresh_ms", 300)

        # Longitudinal mapping (accel -> percent)
        self.declare_parameter("accel_to_throttle_gain", 0.25)
        self.declare_parameter("accel_to_brake_gain", 0.30)
        self.declare_parameter("max_throttle", 0.25)
        self.declare_parameter("max_brake", 0.40)
        self.declare_parameter("throttle_deadband", 0.01)
        self.declare_parameter("brake_deadband", 0.01)

        # ---------------------------------------------------------------------
        # Longitudinal "feel" shaping (Comfort/Normal/Sport) — reduces jerk while
        # keeping the MKZ responsive.
        #
        # Autoware provides desired longitudinal *acceleration* (m/s^2). This
        # bridge converts it into throttle/brake percentages. Direct mapping can
        # feel jerky on a real vehicle due to lag + accel sign changes near 0.
        #
        # Key knobs:
        #  - accel_filter_tau: low-pass filter on accel command (smooths chatter)
        #  - accel_coast_band: "coast" zone around 0 accel (prevents thr/brk flip)
        #  - thr/brk_slew_per_s: rate limits on actuator commands (jerk limiting)
        #  - stop_hold_*: gentle brake hold near stop to prevent creep/oscillation
        #
        # Switch profiles via drive_profile:
        #   comfort  -> smoothest
        #   normal   -> smooth + responsive (recommended)
        #   sport    -> more responsive, still not jerky
        # ---------------------------------------------------------------------
        self.declare_parameter("drive_profile", "normal")   # comfort|normal|sport
        self.declare_parameter("lon_shaping_enable", True)

        # Filtering / coast band
        self.declare_parameter("accel_filter_tau", 0.18)    # seconds (smaller=snappier)
        self.declare_parameter("accel_coast_band", 0.12)    # m/s^2 (bigger=less hunting)

        # Slew-rate limits (units: fraction-per-second of 0..1 command)
        self.declare_parameter("thr_slew_per_s", 0.85)      # throttle rate limit
        self.declare_parameter("brk_slew_per_s", 1.20)      # brake rate limit

        # Stop-hold (prevents creep + accel/brake oscillation near goal)
        self.declare_parameter("stop_hold_enable", True)
        self.declare_parameter("stop_hold_speed_mps", 0.18)     # below this, considered stopped
        self.declare_parameter("stop_hold_accel_band", 0.18)    # if |a| small, allow hold
        self.declare_parameter("stop_hold_brake", 0.04)         # 4% brake hold

        # Gear interlock (brake-to-shift)
        self.declare_parameter("gear_change_requires_brake", True)
        self.declare_parameter("gear_change_brake_percent", 0.70)
        self.declare_parameter("gear_post_shift_hold_ms", 1000)
        self.declare_parameter("gear_change_speed_thresh", 0.10)     # m/s
        self.declare_parameter("gear_change_timeout_ms", 1500)
        self.declare_parameter("gear_report_match_required", True)

        # Auto-park on ARRIVED + zero speed
        self.declare_parameter("route_state_topic", "/planning/mission_planning/state")  # autoware_planning_msgs/msg/RouteState
        self.declare_parameter("auto_park_enable", True)
        self.declare_parameter("auto_park_speed_thresh", 0.05)     # m/s
        self.declare_parameter("auto_park_dwell_ms", 500)          # ms

        # Turn keepalive (DBW expects periodic refresh)
        self.declare_parameter("turn_keepalive_enable", True)
        self.declare_parameter("turn_keepalive_hz", 10.0)          # Hz while turn is requested
        self.declare_parameter("turn_off_burst_ms", 300)           # ms

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

        # Longitudinal shaping params
        self.drive_profile = str(self.get_parameter("drive_profile").value).strip().lower()
        self.lon_shaping_enable = bool(self.get_parameter("lon_shaping_enable").value)

        self.a_tau = float(self.get_parameter("accel_filter_tau").value)
        self.a_coast = float(self.get_parameter("accel_coast_band").value)

        self.thr_slew = float(self.get_parameter("thr_slew_per_s").value)
        self.brk_slew = float(self.get_parameter("brk_slew_per_s").value)

        self.stop_hold_enable = bool(self.get_parameter("stop_hold_enable").value)
        self.stop_hold_speed = float(self.get_parameter("stop_hold_speed_mps").value)
        self.stop_hold_accel_band = float(self.get_parameter("stop_hold_accel_band").value)
        self.stop_hold_brake = float(self.get_parameter("stop_hold_brake").value)

        self.gear_change_requires_brake = bool(self.get_parameter("gear_change_requires_brake").value)
        self.gear_change_brake_percent = float(self.get_parameter("gear_change_brake_percent").value)
        self.gear_post_shift_hold_ms = int(self.get_parameter("gear_post_shift_hold_ms").value)
        self.gear_change_speed_thresh = float(self.get_parameter("gear_change_speed_thresh").value)
        self.gear_change_timeout_ms = int(self.get_parameter("gear_change_timeout_ms").value)
        self.gear_report_match_required = bool(self.get_parameter("gear_report_match_required").value)

        self.route_state_topic = str(self.get_parameter("route_state_topic").value)
        self.auto_park_enable = bool(self.get_parameter("auto_park_enable").value)
        self.auto_park_speed_thresh = float(self.get_parameter("auto_park_speed_thresh").value)
        self.auto_park_dwell_ms = int(self.get_parameter("auto_park_dwell_ms").value)

        self.turn_keepalive_enable = bool(self.get_parameter("turn_keepalive_enable").value)
        self.turn_keepalive_hz = float(self.get_parameter("turn_keepalive_hz").value)
        self.turn_off_burst_ms = int(self.get_parameter("turn_off_burst_ms").value)

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
        self.create_subscription(RouteState, self.route_state_topic, self._on_route_state, qos)

        # ---- state ----
        self.engaged = False
        self.dbw_enabled = False

        self.ctrl = _LastControl()
        self._count = 0

        # hazard shim state
        self.last_hazard = 0
        self.last_hazard_t = 0.0

        # Turn state
        self._aw_turn_cmd = 1  # default DISABLE
        self._turn_hold_value = 0           # 0 none, 1 left, 2 right
        self._turn_off_until_t = 0.0
        self._turn_last_pub_t = 0.0

        # Longitudinal shaping internal state (filter + slew limiter)
        self._a_filt = 0.0       # filtered accel command (m/s^2)
        self._thr_cmd = 0.0      # current throttle command (0..1)
        self._brk_cmd = 0.0      # current brake command (0..1)
        self._lon_t = time.time()

        # Apply comfort/normal/sport profile presets (starting point).
        # You can override any value in YAML (no code changes needed).
        self._apply_drive_profile()

        # DBW report caches
        self.last_speed_mps = 0.0
        self.last_speed_t = 0.0
        self.last_wheel_angle_rad = 0.0
        self.last_wheel_angle_t = 0.0
        self.last_dbw_gear = 0
        self.last_dbw_gear_t = 0.0

        # Route / auto-park state
        self.last_route_state = int(RouteState.UNKNOWN)
        self.last_route_state_t = 0.0
        self._auto_park_seen_zero_t = 0.0
        self._auto_park_latched = False

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
            "MKZ cmd bridge ready. Throttle/Brake CMD_PERCENT. "
            f"Gear interlock: {'ON' if self.gear_change_requires_brake else 'OFF'}. "
            f"Auto-park: {'ON' if self.auto_park_enable else 'OFF'}. "
            f"Turn keepalive: {'ON' if self.turn_keepalive_enable else 'OFF'}."
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




    def _param_was_overridden(self, name: str) -> bool:
        """True if parameter was explicitly set via YAML/CLI overrides.

        Precedence:
          explicit YAML/CLI value  >  drive_profile preset  >  .py default
        """
        try:
            return name in getattr(self, "_parameter_overrides", {})
        except Exception:
            return False

    def _apply_drive_profile(self) -> None:
        """Apply comfort/normal/sport presets for longitudinal shaping.

        IMPORTANT: explicit YAML/CLI parameters should override the profile.
        Precedence:
          explicit YAML/CLI value  >  drive_profile preset  >  .py default
        """
        p = str(getattr(self, "drive_profile", "normal")).strip().lower()

        if p == "comfort":
            preset = dict(
                accel_filter_tau=0.28,
                accel_coast_band=0.16,
                thr_slew_per_s=0.65,
                brk_slew_per_s=0.95,
                stop_hold_speed_mps=0.22,
                stop_hold_accel_band=0.22,
                stop_hold_brake=0.05,
            )
        elif p == "sport":
            preset = dict(
                accel_filter_tau=0.12,
                accel_coast_band=0.09,
                thr_slew_per_s=1.20,
                brk_slew_per_s=1.60,
                stop_hold_speed_mps=0.16,
                stop_hold_accel_band=0.16,
                stop_hold_brake=0.035,
            )
        else:
            preset = dict(
                accel_filter_tau=0.18,
                accel_coast_band=0.12,
                thr_slew_per_s=0.85,
                brk_slew_per_s=1.20,
                stop_hold_speed_mps=0.18,
                stop_hold_accel_band=0.18,
                stop_hold_brake=0.04,
            )

        # Apply preset only if user did NOT override that param in YAML/CLI
        if not self._param_was_overridden("accel_filter_tau"):
            self.a_tau = float(preset["accel_filter_tau"])
        if not self._param_was_overridden("accel_coast_band"):
            self.a_coast = float(preset["accel_coast_band"])
        if not self._param_was_overridden("thr_slew_per_s"):
            self.thr_slew = float(preset["thr_slew_per_s"])
        if not self._param_was_overridden("brk_slew_per_s"):
            self.brk_slew = float(preset["brk_slew_per_s"])
        if not self._param_was_overridden("stop_hold_speed_mps"):
            self.stop_hold_speed = float(preset["stop_hold_speed_mps"])
        if not self._param_was_overridden("stop_hold_accel_band"):
            self.stop_hold_accel_band = float(preset["stop_hold_accel_band"])
        if not self._param_was_overridden("stop_hold_brake"):
            self.stop_hold_brake = float(preset["stop_hold_brake"])

        self.get_logger().info(
            f"Drive profile={p}. lon_shaping_enable={self.lon_shaping_enable}. "
            f"tau={self.a_tau:.3f}s coast={self.a_coast:.3f} "
            f"thr_slew={self.thr_slew:.2f}/s brk_slew={self.brk_slew:.2f}/s "
            f"stop_hold={self.stop_hold_enable} (v<{self.stop_hold_speed:.2f} m/s, brk={self.stop_hold_brake:.3f})."
        )

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

    def _on_steering_report(self, msg: DbwSteeringReport) -> None:
        try:
            self.last_speed_mps = float(msg.speed)
        except Exception:
            self.last_speed_mps = 0.0
        self.last_speed_t = time.time()

        try:
            self.last_wheel_angle_rad = float(msg.steering_wheel_angle)
        except Exception:
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

    def _on_route_state(self, msg: RouteState) -> None:
        try:
            self.last_route_state = int(msg.state)
        except Exception:
            self.last_route_state = int(RouteState.UNKNOWN)
        self.last_route_state_t = time.time()

        if self.last_route_state != int(RouteState.ARRIVED):
            self._auto_park_latched = False
            self._auto_park_seen_zero_t = 0.0

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

    # -------- turn keepalive helpers --------
    def _recalc_turn_policy(self) -> None:
        now = time.time()
        cmd = int(getattr(self, "_aw_turn_cmd", 0))
        # Autoware TurnIndicatorsCommand: 0 NO_COMMAND, 1 DISABLE, 2 LEFT, 3 RIGHT
        if cmd == 2:
            self._turn_hold_value = 1
            self._turn_off_until_t = 0.0
        elif cmd == 3:
            self._turn_hold_value = 2
            self._turn_off_until_t = 0.0
        else:
            self._turn_hold_value = 0
            self._turn_off_until_t = now + (float(self.turn_off_burst_ms) / 1000.0)

        self._turn_last_pub_t = 0.0  # publish immediately

    def _publish_misc_if_needed(self, now: float) -> None:
        if not self.turn_keepalive_enable:
            return

        hz = float(self.turn_keepalive_hz)
        if hz <= 0.0:
            return
        period = 1.0 / hz
        if self._turn_last_pub_t > 0.0 and (now - self._turn_last_pub_t) < period:
            return

        if int(self._turn_hold_value) != 0:
            send_value = int(self._turn_hold_value)
        elif now <= float(self._turn_off_until_t):
            send_value = 0
        else:
            return

        m = MiscCmd()
        self._stamp(m)
        m.cmd.value = int(send_value)  # 0 NONE, 1 LEFT, 2 RIGHT
        m.pbrk.cmd = 0
        self.pub_misc.publish(m)
        self._turn_last_pub_t = now

    # -------- intelligent steering velocity helpers --------
    def _interp_cap(self, speed_mps: float) -> float:
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
        if not self.steer_vel_enable:
            return float(_clamp(self.swa_vel_fixed, 0.0, 17.5))

        wheel_cur = float(target_wheel_angle)
        if self._report_fresh(self.last_wheel_angle_t):
            wheel_cur = float(self.last_wheel_angle_rad)

        spd = 10.0
        if self._report_fresh(self.last_speed_t):
            spd = float(abs(self.last_speed_mps))

        cap = float(_clamp(self._interp_cap(spd), 0.0, 17.5))
        err = abs(float(target_wheel_angle) - float(wheel_cur))

        v_req = float(self.steer_vel_min + self.steer_vel_gain * err)
        v_req = float(_clamp(v_req, self.steer_vel_min, cap))

        dt = max(1e-3, float(now - self._steer_vel_t))
        self._steer_vel_t = float(now)

        tau = max(1e-3, float(self.steer_vel_tau))
        alpha = dt / (tau + dt)
        v_filt = float(self._steer_vel_cmd + alpha * (v_req - self._steer_vel_cmd))

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

        tire = self.ctrl.tire_angle_rad if self._control_fresh(now) else 0.0
        self.pub_str.publish(self._build_steering(now, tire))

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

        # Auto-park: ARRIVED + zero speed dwell -> shift to PARK (DBW gear=1)
        if self.auto_park_enable:
            arrived = (self.last_route_state == int(RouteState.ARRIVED))
            speed_fresh = self._report_fresh(self.last_speed_t)
            speed_zero = speed_fresh and (abs(self.last_speed_mps) <= float(self.auto_park_speed_thresh))

            if not arrived:
                self._auto_park_seen_zero_t = 0.0
            else:
                if speed_zero:
                    if self._auto_park_seen_zero_t <= 0.0:
                        self._auto_park_seen_zero_t = now
                else:
                    self._auto_park_seen_zero_t = 0.0

            already_in_park = self._report_fresh(self.last_dbw_gear_t) and (int(self.last_dbw_gear) == 1)

            if arrived and (not self._auto_park_latched) and (not already_in_park) and (self.shift_state == self._SHIFT_IDLE):
                dwell_ok = self._auto_park_seen_zero_t > 0.0 and (now - self._auto_park_seen_zero_t) * 1000.0 >= float(self.auto_park_dwell_ms)
                if dwell_ok and self._ready():
                    self.shift_target_dbw_gear = 1  # PARK
                    self.shift_t0 = now
                    self.shift_t_phase = now
                    self.shift_state = self._SHIFT_PRE_BRAKE
                    self.shift_cmd_sent = False
                    self._auto_park_latched = True
                    self.get_logger().info("Auto-park: ARRIVED + zero speed dwell satisfied -> shifting to PARK.")

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

        a_raw = float(self.ctrl.accel_mps2)

        # -----------------------------------------------------------------
        # Longitudinal command shaping (smooth but responsive)
        # -----------------------------------------------------------------
        if not self.lon_shaping_enable:
            # Legacy behavior: direct accel->percent mapping (most responsive, can be jerky)
            thr = 0.0
            brk = 0.0
            if a_raw >= 0.0:
                thr = _clamp(a_raw * self.k_th, 0.0, self.max_th)
                if thr < self.db_th:
                    thr = 0.0
            else:
                brk = _clamp((-a_raw) * self.k_br, 0.0, self.max_br)
                if brk < self.db_br:
                    brk = 0.0

            self.pub_thr.publish(self._build_throttle(thr))
            self.pub_brk.publish(self._build_brake(brk))
        else:
            # 1) Low-pass filter the acceleration command to remove chatter
            dt = max(1e-3, float(now - self._lon_t))
            self._lon_t = float(now)

            tau = max(1e-3, float(self.a_tau))
            alpha = dt / (tau + dt)
            self._a_filt = float(self._a_filt + alpha * (a_raw - self._a_filt))
            a = float(self._a_filt)

            # 2) Convert accel->percent with a "coast band" around 0 accel.
            #    Prevents throttle/brake flipping near zero (speed hunting).
            thr_tgt = 0.0
            brk_tgt = 0.0

            if abs(a) < float(self.a_coast):
                thr_tgt = 0.0
                brk_tgt = 0.0
            elif a > 0.0:
                thr_tgt = _clamp(a * self.k_th, 0.0, self.max_th)
                if thr_tgt < self.db_th:
                    thr_tgt = 0.0
            else:
                brk_tgt = _clamp((-a) * self.k_br, 0.0, self.max_br)
                if brk_tgt < self.db_br:
                    brk_tgt = 0.0

            # 3) Stop-hold: prevents creep and oscillation near a full stop.
            if self.stop_hold_enable and self._report_fresh(self.last_speed_t):
                speed_mps = float(abs(self.last_speed_mps))
                if speed_mps < float(self.stop_hold_speed) and abs(a) < float(self.stop_hold_accel_band):
                    thr_tgt = 0.0
                    brk_tgt = max(float(brk_tgt), float(self.stop_hold_brake))

            # 4) Slew-rate limit throttle and brake commands (jerk limiting).
            thr_step = max(0.0, float(self.thr_slew)) * dt
            brk_step = max(0.0, float(self.brk_slew)) * dt

            self._thr_cmd = float(_clamp(thr_tgt, self._thr_cmd - thr_step, self._thr_cmd + thr_step))
            self._brk_cmd = float(_clamp(brk_tgt, self._brk_cmd - brk_step, self._brk_cmd + brk_step))

            # Ensure mutual exclusion (never command both at once).
            if self._thr_cmd > 0.0 and self._brk_cmd > 0.0:
                if self._thr_cmd >= self._brk_cmd:
                    self._brk_cmd = 0.0
                else:
                    self._thr_cmd = 0.0

            self.pub_thr.publish(self._build_throttle(self._thr_cmd))
            self.pub_brk.publish(self._build_brake(self._brk_cmd))

        # Steering / misc unchanged
        self.pub_str.publish(self._build_steering(now, self.ctrl.tire_angle_rad))
        self._publish_misc_if_needed(now)

    @staticmethod
    def _map_aw_gear_to_dbw(aw_gear: int) -> int:
        # Mapping consistent with your previous bridge:
        # 22=P, (20,21)=R/N treated as R, 1=D, 2..19=LOW, 23/24=extra -> 5
        if aw_gear == 22:
            return 1  # PARK
        if aw_gear in (20, 21):
            return 2  # REVERSE
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
