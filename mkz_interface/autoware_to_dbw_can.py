#!/usr/bin/env python3
# Autoware → Dataspeed DBW1 bridge (percent/pedal-based throttle/brake)
#
# Minimal changes from your version:
#  - New params: throttle_cmd_type, brake_cmd_type (defaults 1 = CMD_PEDAL)
#  - Use those params in ThrottleCmd/BrakeCmd builders
#
# Everything else left as-is (topics, gating, timers, etc.)

import time
import signal
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool

# Autoware (Universe) commands
from autoware_control_msgs.msg import Control as AwControl
from autoware_vehicle_msgs.msg import (
    GearCommand as AwGearCommand,
    TurnIndicatorsCommand as AwTurnCmd,
    HazardLightsCommand as AwHazCmd,
    HazardLightsReport,                    # [+] SIM-ONLY status publisher
    Engage as AwEngage,
)

# Fallback (older Autoware.Auto)
try:
    from autoware_auto_control_msgs.msg import AckermannControlCommand as AutoAckermann
except Exception:
    AutoAckermann = None  # type: ignore

# Dataspeed Ford DBW1 messages
from dbw_ford_msgs.msg import (
    ThrottleCmd,
    BrakeCmd,
    SteeringCmd,
    GearCmd,
    MiscCmd,
    WheelSpeedReport,
    GearReport,
)

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def _gear_name(g: Optional[int]) -> str:
    names = {0:'NONE',1:'PARK',2:'REVERSE',3:'NEUTRAL',4:'DRIVE',5:'LOW'}
    return names.get(g, f'{g}')

def _extract_dbw_gear_value(state_obj) -> Optional[int]:
    """Support nested dbw_ford_msgs/Gear (state.gear) or flat uint8."""
    try:
        return int(getattr(state_obj, 'gear'))
    except Exception:
        try:
            return int(state_obj)
        except Exception:
            return None

def _set_if_has(obj, field, value):
    if hasattr(obj, field):
        setattr(obj, field, value)

def _set_turn_signal(msg_obj: MiscCmd, value_int: int) -> bool:
    """Your vehicle layout: field is 'cmd' and nested has 'value'."""
    if hasattr(msg_obj, 'cmd'):
        slot = getattr(msg_obj, 'cmd')
        if hasattr(slot, 'value'):
            slot.value = int(value_int)
            return True
    return False


class AutowareToDbw(Node):
    def __init__(self):
        super().__init__('autoware_to_dbw_can')

        # QoS
        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # -------- Parameters (overridden by config/topics.yaml) --------
        # Rates / safety
        self.declare_parameter('rate_hz', 50)
        self.declare_parameter('watchdog_ms', 200)
        self.declare_parameter('enable_on_engage', False)

        # Engage from vehicle_cmd_gate
        self.declare_parameter('engage_msg_type', 'engage')  # 'engage' | 'bool'
        self.declare_parameter('engage_topic', '/vehicle_cmd_gate/output/engage')

        # Control message selection
        self.declare_parameter('control_msg_type', 'control')  # 'control' | 'ackermann'

        # Conversion (percent-based longitudinal)
        self.declare_parameter('steering_wheel_to_tire_ratio', -14.8)   # tire(rad)→SW(rad)
        self.declare_parameter('accel_to_percent_gain', 0.5)          # m/s^2 → 0..1
        self.declare_parameter('max_throttle', 0.40)                   # 0..1 cap
        self.declare_parameter('max_brake', 0.60)                      # 0..1 cap

        # Interlock (brake-to-shift) parameters
        self.declare_parameter('gear_change_requires_brake', True)
        self.declare_parameter('gear_change_brake_percent', 0.65)      # 65% brake hold during shift
        self.declare_parameter('gear_change_speed_thresh', 0.2)        # m/s
        self.declare_parameter('gear_change_timeout_ms', 1500)         # ms
        self.declare_parameter('wheel_speed_units', 'mps')             # 'mps' or 'kph'

        # Post-shift brake hold
        self.declare_parameter('gear_post_shift_hold_ms', 1000)       # 1.0 s post-hold
        self.declare_parameter('gear_post_shift_brake_percent', -1.0) # -1 → reuse gear_change_brake_percent

        # Gear diagnostics
        self.declare_parameter('gear_diag_warn_after_ms', 700)
        self.declare_parameter('gear_diag_log_interval_ms', 500)

        # Autoware (gate) outputs
        self.declare_parameter('aw_control_cmd', '/vehicle_cmd_gate/output/command/control_cmd')
        self.declare_parameter('aw_gear_cmd',    '/vehicle_cmd_gate/output/command/gear_cmd')
        self.declare_parameter('aw_turn_cmd',    '/vehicle_cmd_gate/output/command/turn_indicators_cmd')
        self.declare_parameter('aw_hazard_cmd',  '/vehicle_cmd_gate/output/command/hazard_lights_cmd')

        # DBW commands
        self.declare_parameter('dbw_throttle_cmd', '/vehicle/throttle_cmd')
        self.declare_parameter('dbw_brake_cmd',    '/vehicle/brake_cmd')
        self.declare_parameter('dbw_steering_cmd', '/vehicle/steering_cmd')
        self.declare_parameter('dbw_gear_cmd',     '/vehicle/gear_cmd')
        self.declare_parameter('dbw_misc_cmd',     '/vehicle/misc_cmd')
        self.declare_parameter('dbw_enable_cmd',   '/vehicle/enable')
        self.declare_parameter('dbw_disable_cmd',  '/vehicle/disable')

        # DBW feedback (enabled, speed, gear state)
        self.declare_parameter('dbw_dbw_enabled',     '/vehicle/dbw_enabled')
        self.declare_parameter('dbw_wheel_speed_report', '/vehicle/wheel_speed_report')
        self.declare_parameter('dbw_gear_report',        '/vehicle/gear_report')

        # NEW: pedal command type knobs (defaults: 1 = CMD_PEDAL)
        self.declare_parameter('throttle_cmd_type', 1)  # 1 = CMD_PEDAL, 2 = CMD_PERCENT
        self.declare_parameter('brake_cmd_type', 1)     # 1 = CMD_PEDAL, 2 = CMD_PERCENT

        # ---- Hazards (unsupported on Dataspeed DBW; sim-only status is optional)
        self.declare_parameter('sim_publish_hazard_status', False)
        self.declare_parameter('hazard_lights_status_topic', '/vehicle/status/hazard_lights_status')

        # Pull params
        p = {pp.name: pp.value for pp in self.get_parameters([
            'rate_hz','watchdog_ms','enable_on_engage',
            'engage_msg_type','engage_topic','control_msg_type',
            'steering_wheel_to_tire_ratio','accel_to_percent_gain','max_throttle','max_brake',
            'gear_change_requires_brake','gear_change_brake_percent','gear_change_speed_thresh','gear_change_timeout_ms',
            'wheel_speed_units','gear_diag_warn_after_ms','gear_diag_log_interval_ms',
            'aw_control_cmd','aw_gear_cmd','aw_turn_cmd','aw_hazard_cmd',
            'dbw_throttle_cmd','dbw_brake_cmd','dbw_steering_cmd','dbw_gear_cmd','dbw_misc_cmd',
            'dbw_enable_cmd','dbw_disable_cmd','dbw_dbw_enabled',
            'dbw_wheel_speed_report','dbw_gear_report',
            'throttle_cmd_type','brake_cmd_type',
            'sim_publish_hazard_status','hazard_lights_status_topic',
            'gear_post_shift_hold_ms','gear_post_shift_brake_percent',
        ])}

        self.rate_hz               = float(p['rate_hz'])
        self.watchdog_ms           = int(p['watchdog_ms'])
        self.enable_on_engage      = bool(p['enable_on_engage'])
        self.engage_msg_type       = str(p['engage_msg_type']).lower()
        self.engage_topic          = str(p['engage_topic'])
        self.control_msg_type      = str(p['control_msg_type']).lower()

        self.st_ratio              = float(p['steering_wheel_to_tire_ratio'])
        self.accel_to_percent_gain = float(p['accel_to_percent_gain'])
        self.max_throttle          = float(p['max_throttle'])
        self.max_brake             = float(p['max_brake'])

        self.interlock_enabled     = bool(p['gear_change_requires_brake'])
        self.interlock_brake_pct   = float(p['gear_change_brake_percent'])
        self.interlock_speed_thr   = float(p['gear_change_speed_thresh'])
        self.interlock_timeout_ms  = int(p['gear_change_timeout_ms'])
        self.ws_units              = str(p['wheel_speed_units']).lower()

        self.gear_diag_warn_after_ms   = int(p['gear_diag_warn_after_ms'])
        self.gear_diag_log_interval_ms = int(p['gear_diag_log_interval_ms'])

        self.aw_control_cmd = str(p['aw_control_cmd'])
        self.aw_gear_cmd    = str(p['aw_gear_cmd'])
        self.aw_turn_cmd    = str(p['aw_turn_cmd'])
        self.aw_hazard_cmd  = str(p['aw_hazard_cmd'])

        self.dbw_throttle_cmd = str(p['dbw_throttle_cmd'])
        self.dbw_brake_cmd    = str(p['dbw_brake_cmd'])
        self.dbw_steering_cmd = str(p['dbw_steering_cmd'])
        self.dbw_gear_cmd     = str(p['dbw_gear_cmd'])
        self.dbw_misc_cmd     = str(p['dbw_misc_cmd'])
        self.dbw_enable_cmd   = str(p['dbw_enable_cmd'])
        self.dbw_disable_cmd  = str(p['dbw_disable_cmd'])

        self.dbw_dbw_enabled     = str(p['dbw_dbw_enabled'])
        self.dbw_wheel_speed_report = str(p['dbw_wheel_speed_report'])
        self.dbw_gear_report        = str(p['dbw_gear_report'])

        # Brake hold
        self.post_hold_ms = int(p['gear_post_shift_hold_ms'])
        _gpsbp = float(p['gear_post_shift_brake_percent'])
        self.post_hold_brake_pct = _gpsbp if _gpsbp >= 0.0 else self.interlock_brake_pct


        # NEW: assign pedal types
        self.throttle_cmd_type = int(p['throttle_cmd_type'])
        self.brake_cmd_type    = int(p['brake_cmd_type'])

        # Hazards (sim-only status option)
        self._sim_publish_hazard_status = bool(p['sim_publish_hazard_status'])
        self._hazard_status_topic = str(p['hazard_lights_status_topic'])
        self._hazard_warned_once = False

        # -------- Publishers (DBW) --------
        self.pub_throttle = self.create_publisher(ThrottleCmd, self.dbw_throttle_cmd, qos)
        self.pub_brake    = self.create_publisher(BrakeCmd,    self.dbw_brake_cmd,    qos)
        self.pub_steer    = self.create_publisher(SteeringCmd, self.dbw_steering_cmd, qos)
        self.pub_gear     = self.create_publisher(GearCmd,     self.dbw_gear_cmd,     qos)
        self.pub_misc     = self.create_publisher(MiscCmd,     self.dbw_misc_cmd,     qos)
        self.pub_enable   = self.create_publisher(Bool,        self.dbw_enable_cmd,   qos)
        self.pub_disable  = self.create_publisher(Bool,        self.dbw_disable_cmd,  qos)
        # Publisher for SIM-ONLY hazard status (no DBW actuation)
        self._pub_hazard_status = self.create_publisher(HazardLightsReport, self._hazard_status_topic, 10)

        # -------- Subscriptions --------
        # Engage
        if self.engage_msg_type == 'engage':
            self.sub_engage = self.create_subscription(AwEngage, self.engage_topic, self.on_engage_msg, qos)
        else:
            self.sub_engage = self.create_subscription(Bool, self.engage_topic, self.on_engage_bool, qos)

        # DBW enable feedback
        self.sub_dbw_en = self.create_subscription(Bool, self.dbw_dbw_enabled, self.on_dbw_enabled, qos)

        # Control (AW)
        if self.control_msg_type == 'control':
            self.sub_control = self.create_subscription(AwControl, self.aw_control_cmd, self.on_control_control, qos)
            self.using_control = True
        else:
            if AutoAckermann is None:
                self.get_logger().warn("control_msg_type=ackermann requested but autoware_auto_control_msgs not available; using 'control'")
                self.sub_control = self.create_subscription(AwControl, self.aw_control_cmd, self.on_control_control, qos)
                self.using_control = True
            else:
                self.sub_control = self.create_subscription(AutoAckermann, self.aw_control_cmd, self.on_control_ackermann, qos)
                self.using_control = False

        # Ancillary (AW)
        self.sub_gear   = self.create_subscription(AwGearCommand, self.aw_gear_cmd,   self.on_gear_request, qos)
        self.sub_turn   = self.create_subscription(AwTurnCmd,     self.aw_turn_cmd,   self.on_turn,   qos)
        self.sub_hazard = self.create_subscription(AwHazCmd,      self.aw_hazard_cmd, self.on_hazard, qos)

        # Feedback (speed, gear state)
        self.sub_ws   = self.create_subscription(WheelSpeedReport, self.dbw_wheel_speed_report, self.on_wheel_speeds, qos)
        self.sub_grep = self.create_subscription(GearReport,        self.dbw_gear_report,        self.on_dbw_gear_report, qos)

        # -------- State --------
        self.engaged: bool = False
        self.dbw_enabled: bool = False
        self.last_accel: float = 0.0
        self.last_tire_angle: float = 0.0
        self.last_cmd_time: Optional[float] = None

        # Latest Autoware turn/hazard
        self._aw_turn_cmd: int = 0
        self._aw_hazard_cmd: int = 0

        # Turn/hazard policy
        self._hazards_continuous: bool = False
        self._turn_burst_until: Optional[float] = None
        self._turn_value: int = 0  # 0/1/2 (NONE/LEFT/RIGHT)

        # Speed + gear feedback
        self.current_speed_mps: float = 0.0
        self.current_dbw_gear: Optional[int] = None

        # Interlock FSM
        self.pending_gear_dbw: Optional[int] = None
        self.interlock_deadline: Optional[float] = None
        self.interlock_active: bool = False
        self.post_hold_deadline: Optional[float] = None

        # Shutdown coordination
        self._shutdown_requested = False

        self.count = 0
        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'Autoware→DBW bridge @ {self.rate_hz:.1f} Hz '
            f'(longitudinal={"pedal" if self.throttle_cmd_type==1 else "percent"}, interlock={"on" if self.interlock_enabled else "off"})'
        )

    # ================= Engage & enable =================
    def on_engage_msg(self, msg: AwEngage):
        prev = self.engaged
        self.engaged = bool(msg.engage)
        self._handle_engage_edge(prev)

    def on_engage_bool(self, msg: Bool):
        prev = self.engaged
        self.engaged = bool(msg.data)
        self._handle_engage_edge(prev)

    def _handle_engage_edge(self, prev: bool):
        if self.enable_on_engage and (not prev) and self.engaged:
            self.get_logger().info('Engage↑ → DBW enable request')
            self.pub_enable.publish(Bool(data=True))
        if prev and (not self.engaged):
            self.get_logger().info('Engage↓ → safe_zero (+ optional disable)')
            self.safe_zero()
            if self.enable_on_engage:
                self.pub_disable.publish(Bool(data=True))

    def on_dbw_enabled(self, msg: Bool):
        self.dbw_enabled = bool(msg.data)

    # ================= Feedback used by interlock =================
    def on_wheel_speeds(self, msg: WheelSpeedReport):
        v = float(msg.front_left + msg.front_right + msg.rear_left + msg.rear_right) / 4.0
        if self.ws_units == 'kph':
            v = v / 3.6
        self.current_speed_mps = v

    def on_dbw_gear_report(self, msg: GearReport):
        g = _extract_dbw_gear_value(msg.state)
        if g is None:
            self.get_logger().warn('GearReport.state has unexpected type/layout')
            return
        self.current_dbw_gear = g
        if self.pending_gear_dbw is not None and self.current_dbw_gear == self.pending_gear_dbw:
            self.get_logger().info(
                f'Gear confirmed by DBW: {self.current_dbw_gear} ({_gear_name(self.current_dbw_gear)})'
            )
            self.pending_gear_dbw = None
            self.interlock_active = False
            self.interlock_deadline = None
            self.post_hold_deadline = time.time() + self.post_hold_ms / 1000.0
            return

    # ================= Control callbacks =================
    def on_control_control(self, cmd: AwControl):
        try:
            self.last_accel = float(cmd.longitudinal.acceleration)
        except Exception:
            self.last_accel = 0.0
        try:
            self.last_tire_angle = float(cmd.lateral.steering_tire_angle)
        except Exception:
            self.last_tire_angle = 0.0
        self.last_cmd_time = time.time()

    def on_control_ackermann(self, cmd: 'AutoAckermann'):
        self.last_accel = float(cmd.longitudinal.acceleration)
        self.last_tire_angle = float(cmd.lateral.steering_tire_angle)
        self.last_cmd_time = time.time()

    # ================= Ancillary: gear / turns / hazard =================
    def on_gear_request(self, cmd: AwGearCommand):
        # Keep your existing mapping as-is (no behavior changes requested here).
        aw = int(getattr(cmd, "command", 0))
        if aw == 0:                dbw = 0  # NONE
        elif aw == 3:              dbw = 3  # NEUTRAL (your build)
        elif 2 <= aw <= 19:        dbw = 4  # DRIVE and DRIVE_*
        elif aw in (20, 21):       dbw = 2  # REVERSE / REVERSE_2
        elif aw == 1:             dbw = 1  # PARK
        elif aw in (4, 23, 24):       dbw = 5  # LOW / LOW_2
        else:                      dbw = 0  # default NONE

        if not self.interlock_enabled:
            self._publish_gear_cmd(dbw)
            return

        if self.current_speed_mps > self.interlock_speed_thr:
            self.get_logger().warn(
                f'Gear change requested at {self.current_speed_mps:.2f} m/s (> {self.interlock_speed_thr}); ignoring'
            )
            return

        self.pending_gear_dbw = dbw
        self.interlock_active = True
        self.interlock_deadline = time.time() + self.interlock_timeout_ms / 1000.0

        self.get_logger().info(
            f'Gear change request → DBW:{dbw} ({_gear_name(dbw)})'
            f'; applying {self.interlock_brake_pct*100:.0f}% brake hold and issuing shift'
        )

        self._publish_gear_cmd(dbw)

    def _publish_gear_cmd(self, dbw_gear: int):
        if not self._ok_to_send():
            return
        m = GearCmd()
        _set_if_has(m, 'enable', True)
        _set_if_has(m, 'clear', False)
        _set_if_has(m, 'ignore', False)
        if hasattr(m, 'count'):
            m.count = self._bump()
        try:
            m.cmd.gear = dbw_gear
        except AttributeError:
            m.cmd = dbw_gear
        self.pub_gear.publish(m)

    def on_hazard(self, cmd: AwHazCmd):
        """
        Dataspeed DBW (MKZ) cannot actuate hazards. Ignore the command with a one-time WARN.
        Optionally, for SIM ONLY, publish HazardLightsReport so RViz/Planning Sim can visualize it.
        """
        if not self._hazard_warned_once:
            self.get_logger().warn(
                "Hazard lights are not controllable via Dataspeed DBW on the MKZ. "
                "Ignoring HazardLightsCommand (this warning prints once)."
            )
            self._hazard_warned_once = True

        # Ensure hazards do NOT affect MiscCmd/turn policy on the real vehicle
        self._aw_hazard_cmd = 1  # force DISABLE in our internal policy
        # (keep existing _aw_turn_cmd as-is; user did not request turn mapping changes)
        self._recalc_misc_policy()

        # SIM-ONLY: echo as status for visualization
        if not self._sim_publish_hazard_status:
            return
        rep = HazardLightsReport()
        rep.stamp = self.get_clock().now().to_msg()
        cmd_val = int(getattr(cmd, "command", 0))
        if cmd_val == AwHazCmd.ENABLE:
            rep.report = HazardLightsReport.ENABLE
        elif cmd_val == AwHazCmd.DISABLE:
            rep.report = HazardLightsReport.DISABLE
        else:
            return  # NO_COMMAND
        self._pub_hazard_status.publish(rep)

    def on_turn(self, cmd: AwTurnCmd):
        self._aw_hazard_cmd = 1
        self._aw_turn_cmd = int(getattr(cmd, "command", 0))
        self._recalc_misc_policy()

    def _recalc_misc_policy(self):
        # TurnIndicatorsCommand {DISABLE=1, LEFT=2, RIGHT=3}
        # HazardLightsCommand   {DISABLE=1, ENABLE=2}
        if self._aw_hazard_cmd == 2:
            self._hazards_continuous = True
            self._turn_burst_until = None
            self._turn_value = 0
        else:
            self._hazards_continuous = False
            if self._aw_turn_cmd == 2:
                self._turn_value = 1  # LEFT
                self._turn_burst_until = time.time() + 0.5
            elif self._aw_turn_cmd == 3:
                self._turn_value = 2  # RIGHT
                self._turn_burst_until = time.time() + 0.5
            else:
                self._turn_value = 0
                self._turn_burst_until = time.time() + 0.2

    # ================= Timer loop =================
    def on_timer(self):
        now = time.time()

        # ---- Longitudinal heartbeat and commands (gated by engage+dbw_enabled) ----
        if (self.last_cmd_time is None) or ((now - self.last_cmd_time) * 1000.0 > self.watchdog_ms):
            self.safe_zero()
        else:
            if not self._ok_to_send():
                self.safe_zero()
            else:
                accel = float(self.last_accel)
                throttle_pct = 0.0
                brake_pct = 0.0
                if accel >= 0.0:
                    throttle_pct = clamp(accel * self.accel_to_percent_gain, 0.0, self.max_throttle)
                    brake_pct = 0.0
                else:
                    brake_pct = clamp(-accel * self.accel_to_percent_gain, 0.0, self.max_brake)
                    throttle_pct = 0.0

                # Interlock override: keep brake applied and zero throttle while shifting
                if self.interlock_active:
                    throttle_pct = 0.0
                    brake_pct = max(brake_pct, self.interlock_brake_pct)
                    if self.pending_gear_dbw is not None:
                        self._publish_gear_cmd(self.pending_gear_dbw)
                    if self.interlock_deadline and now > self.interlock_deadline:
                        self.pending_gear_dbw = None
                        self.interlock_active = False
                        self.interlock_deadline = None
                # Post-shift hold window (zero throttle + hold brake for extra time)
                if self.post_hold_deadline:
                    if now <= self.post_hold_deadline:
                        throttle_pct = 0.0
                        brake_pct = max(brake_pct, self.post_hold_brake_pct)
                    else:
                        self.post_hold_deadline = None

                # Lateral: tire angle(rad) → steering wheel angle(rad)
                swa_cmd = clamp(self.last_tire_angle * self.st_ratio, -8.0, 8.0)

                self.pub_throttle.publish(self._mk_throttle_percent(throttle_pct))
                self.pub_brake.publish(self._mk_brake_percent(brake_pct))
                self.pub_steer.publish(self._mk_steer_angle(swa_cmd, swa_vel=3.0))

        # ---- MiscCmd policy ----
        send_value: Optional[int] = None
        if self._hazards_continuous:
            send_value = 3  # HAZARD continuously
        elif self._turn_burst_until and now < self._turn_burst_until:
            send_value = int(self._turn_value)
        else:
            self._turn_burst_until = None

        if send_value is not None:
            m = MiscCmd()
            if hasattr(m, 'count'):
                m.count = self._bump()
            _set_turn_signal(m, send_value)
            self.pub_misc.publish(m)

    # ================= Helpers =================
    def _ok_to_send(self) -> bool:
        return self.engaged and self.dbw_enabled

    def _bump(self) -> int:
        self.count = (self.count + 1) & 0xFF
        return self.count

    def _mk_throttle_percent(self, percent: float) -> ThrottleCmd:
        msg = ThrottleCmd()
        _set_if_has(msg, 'enable', True)
        _set_if_has(msg, 'clear', False)
        _set_if_has(msg, 'ignore', False)

        # Autoware expresses intent as 0..1 "percent"
        p = clamp(float(percent), 0.0, 1.0)

        if int(self.throttle_cmd_type) == 1:  # CMD_PEDAL (unitless, valid 0.15..0.80)
            # Floor-only mapping: preserve magnitude (no amplification), just ensure minimum is met.
            if p <= 0.0:
                cmd = 0.0
            else:
                cmd = max(((p*0.65)+0.15), 0.15)   # keep p as-is if >= 0.15; bump tiny p to 0.15 so PARK reacts
                cmd = min(cmd, 0.80)     # respect pedal upper bound
        else:  # CMD_PERCENT (0..1)
            cmd = p

        msg.pedal_cmd = float(cmd)
        _set_if_has(msg, 'pedal_cmd_type', int(self.throttle_cmd_type))
        if hasattr(msg, 'count'):
            msg.count = self._bump()
        return msg


    def _mk_brake_percent(self, percent: float) -> BrakeCmd:
        msg = BrakeCmd()
        _set_if_has(msg, 'enable', True)
        _set_if_has(msg, 'clear', False)
        _set_if_has(msg, 'ignore', False)

        p = clamp(float(percent), 0.0, 1.0)

        if int(self.brake_cmd_type) == 1:  # CMD_PEDAL (unitless, valid 0.15..0.50)
            if p <= 0.0:
                cmd = 0.0
            else:
                cmd = max(((p*0.65)+0.15), 0.15)   # only floor tiny values; otherwise keep p unchanged
                cmd = min(cmd, 0.80)     # respect pedal upper bound
        else:  # CMD_PERCENT (0..1)
            cmd = p

        msg.pedal_cmd = float(cmd)
        _set_if_has(msg, 'pedal_cmd_type', int(self.brake_cmd_type))
        if hasattr(msg, 'count'):
            msg.count = self._bump()
        return msg
 


    def _mk_steer_angle(self, swa_rad: float, swa_vel: float = 3.0) -> SteeringCmd:
        msg = SteeringCmd()
        _set_if_has(msg, 'enable', True)
        _set_if_has(msg, 'clear', False)
        _set_if_has(msg, 'ignore', False)
        msg.steering_wheel_angle_cmd = float(swa_rad)
        _set_if_has(msg, 'steering_wheel_angle_velocity', float(max(0.0, swa_vel)))
        _set_if_has(msg, 'cmd_type', 0)  # CMD_ANGLE
        _set_if_has(msg, 'quiet', False)
        _set_if_has(msg, 'alert', False)
        if hasattr(msg, 'count'):
            msg.count = self._bump()
        return msg

    def safe_zero(self):
        # Keep heartbeat alive but send zero actuation
        self.pub_throttle.publish(self._mk_throttle_percent(0.0))
        self.pub_brake.publish(self._mk_brake_percent(0.0))
        self.pub_steer.publish(self._mk_steer_angle(0.0, swa_vel=3.0))


def main():
    rclpy.init()
    node = AutowareToDbw()

    def _sigint_handler(signum, frame):
        node._shutdown_requested = True
    signal.signal(signal.SIGINT, _sigint_handler)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node._shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

