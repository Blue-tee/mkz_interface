# DBW → Autoware status bridge (robust wheel speed & misc extraction)
# Publishes /vehicle/status/* using autoware_vehicle_msgs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool

from autoware_vehicle_msgs.msg import (
    VelocityReport,
    SteeringReport as AwSteeringReport,
    GearReport as AwGearReport,
    TurnIndicatorsReport,
    HazardLightsReport,
    ControlModeReport,
)

from dbw_ford_msgs.msg import (
    WheelSpeedReport,
    SteeringReport,
    GearReport,
    Misc1Report,
    BrakeReport,
    ThrottleReport,
)

def _extract_dbw_gear_value(state_obj):
    """Return DBW enum 0..5 from nested Gear or plain uint8."""
    for attr in ('gear',):
        if hasattr(state_obj, attr):
            try:
                return int(getattr(state_obj, attr))
            except Exception:
                pass
    return int(state_obj)

def _dbw_to_aw_gear(dbw_val: int) -> int:
    """
    Map DBW gear (NONE=0,PARK=1,REVERSE=2,NEUTRAL=3,DRIVE=4,LOW=5)
    → Autoware gear enum:
      NONE=0, NEUTRAL=1, DRIVE=2..19, REVERSE=20..21, PARK=22, LOW=23..24
    """
    return {
        1: 22,  # PARK
        2: 20,  # REVERSE
        3: 1,   # NEUTRAL
        4: 2,   # DRIVE
        5: 23,  # LOW
    }.get(int(dbw_val), 0)

def _coerce_uint8(obj, candidates=('value','data','cmd')):
    """Return an int from an object that might be nested or already numeric."""
    for a in candidates:
        if hasattr(obj, a):
            try:
                return int(getattr(obj, a))
            except Exception:
                pass
    try:
        return int(obj)
    except Exception:
        return 0

class DbwToAutoware(Node):
    def __init__(self):
        super().__init__('dbw_can_to_autoware')

        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # Parameters (topics + conversions)
        self.declare_parameter('dbw_wheel_speed_report', '/vehicle/wheel_speed_report')
        self.declare_parameter('dbw_steering_report',    '/vehicle/steering_report')
        self.declare_parameter('dbw_gear_report',        '/vehicle/gear_report')
        self.declare_parameter('dbw_misc_report',        '/vehicle/misc_1_report')
        self.declare_parameter('dbw_dbw_enabled',        '/vehicle/dbw_enabled')
        self.declare_parameter('dbw_brake_report',       '/vehicle/brake_report')
        self.declare_parameter('dbw_throttle_report',    '/vehicle/throttle_report')

        self.declare_parameter('aw_velocity_status',     '/vehicle/status/velocity_status')
        self.declare_parameter('aw_steering_status',     '/vehicle/status/steering_status')
        self.declare_parameter('aw_gear_status',         '/vehicle/status/gear_status')
        self.declare_parameter('aw_turn_status',         '/vehicle/status/turn_indicators_status')
        self.declare_parameter('aw_hazard_status',       '/vehicle/status/hazard_lights_status')
        self.declare_parameter('aw_control_mode_status', '/vehicle/status/control_mode')
        self.declare_parameter('aw_steering_status_for_gate', '/vehicle_cmd_gate/input/steering')

        self.declare_parameter('steering_wheel_to_tire_ratio', -14.8)
        self.declare_parameter('wheel_speed_units', 'mps')  # 'mps' or 'kph'

        p = {pp.name: pp.value for pp in self.get_parameters([
            'dbw_wheel_speed_report','dbw_steering_report','dbw_gear_report','dbw_misc_report',
            'dbw_dbw_enabled','dbw_brake_report','dbw_throttle_report',
            'aw_velocity_status','aw_steering_status','aw_gear_status','aw_turn_status',
            'aw_hazard_status','aw_control_mode_status','aw_steering_status_for_gate',
            'steering_wheel_to_tire_ratio','wheel_speed_units'
        ])}

        # Publishers
        self.pub_vel   = self.create_publisher(VelocityReport,       p['aw_velocity_status'], qos)
        self.pub_steer = self.create_publisher(AwSteeringReport,     p['aw_steering_status'], qos)
        self.pub_gear  = self.create_publisher(AwGearReport,         p['aw_gear_status'], qos)
        self.pub_turn  = self.create_publisher(TurnIndicatorsReport, p['aw_turn_status'], qos)
        self.pub_haz   = self.create_publisher(HazardLightsReport,   p['aw_hazard_status'], qos)
        self.pub_mode  = self.create_publisher(ControlModeReport,    p['aw_control_mode_status'], qos)

        self.pub_steer_gate = None
        if isinstance(p['aw_steering_status_for_gate'], str) and p['aw_steering_status_for_gate']:
            self.pub_steer_gate = self.create_publisher(AwSteeringReport, p['aw_steering_status_for_gate'], qos)

        # Subscriptions
        self.sub_ws  = self.create_subscription(WheelSpeedReport, p['dbw_wheel_speed_report'], self.on_wheel_speeds, qos)
        self.sub_sr  = self.create_subscription(SteeringReport,   p['dbw_steering_report'],    self.on_steer_report, qos)
        self.sub_gr  = self.create_subscription(GearReport,       p['dbw_gear_report'],        self.on_gear_report, qos)
        self.sub_m1  = self.create_subscription(Misc1Report,      p['dbw_misc_report'],        self.on_misc_report, qos)
        self.sub_en  = self.create_subscription(Bool,             p['dbw_dbw_enabled'],        self.on_dbw_enabled, qos)
        self.sub_br  = self.create_subscription(BrakeReport,      p['dbw_brake_report'],       self.on_brake_report, qos)
        self.sub_th  = self.create_subscription(ThrottleReport,   p['dbw_throttle_report'],    self.on_throttle_report, qos)

        self.dbw_enabled = False
        self.ratio = float(p['steering_wheel_to_tire_ratio'])
        self.ws_units = str(p['wheel_speed_units']).lower()

        # Resolve wheel-speed field names at runtime (supports multiple layouts)
        self._ws_fields = None  # set to a 4-tuple of attribute names when first msg arrives

        self.get_logger().info('DBW→Autoware status bridge started')

    # ---- helpers ----
    def _extract_wheel_speeds(self, msg: WheelSpeedReport):
        # Try common layouts once and cache the result
        candidates = [
            ('fl','fr','rl','rr'),
            ('front_left','front_right','rear_left','rear_right'),
            ('wheel_speed_fl','wheel_speed_fr','wheel_speed_rl','wheel_speed_rr'),
        ]
        if self._ws_fields is None:
            for names in candidates:
                if all(hasattr(msg, n) for n in names):
                    self._ws_fields = names
                    self.get_logger().info(f'WheelSpeedReport fields: {names}')
                    break
            # Fallback: some drivers expose a single vehicle speed
            if self._ws_fields is None:
                single_candidates = ['speed','vehicle_speed','vehicle_velocity','longitudinal_velocity']
                for n in single_candidates:
                    if hasattr(msg, n):
                        v = float(getattr(msg, n))
                        return [v, v, v, v]
                raise AttributeError('Unknown WheelSpeedReport field layout')
        return [float(getattr(msg, n)) for n in self._ws_fields]

    # ---- callbacks ----
    def on_dbw_enabled(self, msg: Bool):
        self.dbw_enabled = bool(msg.data)
        cm = ControlModeReport()
        cm.stamp = self.get_clock().now().to_msg()
        cm.mode = ControlModeReport.AUTONOMOUS if self.dbw_enabled else ControlModeReport.MANUAL
        self.pub_mode.publish(cm)

    def on_wheel_speeds(self, msg: WheelSpeedReport):
        try:
            fl, fr, rl, rr = self._extract_wheel_speeds(msg)
            v = (fl + fr + rl + rr) / 4.0
            if self.ws_units == 'kph':
                v = v / 3.6
        except Exception as e:
            self.get_logger().warn(f'WheelSpeedReport parse failed: {e}')
            return
        vr = VelocityReport()
        vr.header.stamp = self.get_clock().now().to_msg()
        vr.longitudinal_velocity = v
        vr.lateral_velocity = 0.0
        vr.heading_rate = 0.0
        self.pub_vel.publish(vr)

    def on_steer_report(self, msg: SteeringReport):
        sr = AwSteeringReport()
        sr.stamp = self.get_clock().now().to_msg()
        # Dataspeed provides steering wheel angle (rad) → convert to tire angle
        try:
            swa = float(msg.steering_wheel_angle)
        except Exception:
            swa = 0.0
        sr.steering_tire_angle = swa / max(1e-6, self.ratio)
        self.pub_steer.publish(sr)
        if self.pub_steer_gate is not None:
            self.pub_steer_gate.publish(sr)

    def on_gear_report(self, msg: GearReport):
        try:
            dbw_val = _extract_dbw_gear_value(msg.state)
        except Exception as e:
            self.get_logger().warn(f'GearReport parse failed: {e}')
            return
        aw_val = int(_dbw_to_aw_gear(dbw_val))
        gr = AwGearReport()
        gr.stamp = self.get_clock().now().to_msg()
        gr.report = aw_val  # MUST be int
        self.pub_gear.publish(gr)

    def on_misc_report(self, msg: Misc1Report):
        # Turn indicators
        ts = 0
        if hasattr(msg, 'turn_signal'):
            ts = _coerce_uint8(getattr(msg, 'turn_signal'))
        tr = TurnIndicatorsReport()
        tr.stamp = self.get_clock().now().to_msg()
        # DBW TurnSignal: NONE=0, LEFT=1, RIGHT=2, HAZARD=3
        if ts == 1:
            tr.report = TurnIndicatorsReport.LEFT
        elif ts == 2:
            tr.report = TurnIndicatorsReport.RIGHT
        else:
            tr.report = TurnIndicatorsReport.DISABLE
        self.pub_turn.publish(tr)

        # Hazards (either explicit bool or encoded as ts==3)
        hazard_attr_names = ('hazard_lights', 'hazards', 'hazard', 'hazard_enable')
        hazard_flag = any(bool(getattr(msg, n, False)) for n in hazard_attr_names) or (ts == 3)
        hz = HazardLightsReport()
        hz.stamp = self.get_clock().now().to_msg()
        hz.report = HazardLightsReport.ENABLE if hazard_flag else HazardLightsReport.DISABLE
        self.pub_haz.publish(hz)

    def on_brake_report(self, msg: BrakeReport):
        pass  # optional diagnostics

    def on_throttle_report(self, msg: ThrottleReport):
        pass  # optional diagnostics

def main():
    rclpy.init()
    node = DbwToAutoware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

