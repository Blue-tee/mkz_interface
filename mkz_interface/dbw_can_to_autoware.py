#!/usr/bin/env python3
"""
dbw_can_to_autoware.py
Bridge: Dataspeed DBW1 reports -> Autoware vehicle status topics.

- Vehicle speed comes from dbw_ford_msgs/SteeringReport.speed (m/s)
- Steering status is steering_tire_angle = steering_wheel_angle / ratio
- Hazard status is a SHIM: mirrors the latched hazard command (from hazard_shim_topic)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool, UInt8

from autoware_vehicle_msgs.msg import (
    VelocityReport,
    SteeringReport as AwSteeringReport,
    GearReport as AwGearReport,
    TurnIndicatorsReport,
    HazardLightsReport,
    ControlModeReport,
)

from dbw_ford_msgs.msg import SteeringReport, GearReport, Misc1Report


def _const(msg_cls, name: str, default: int) -> int:
    return int(getattr(msg_cls, name, default))


def _dbw_gear_to_aw(dbw_gear: int) -> int:
    return {1: 22, 2: 20, 3: 1, 4: 2, 5: 23}.get(int(dbw_gear), 0)


class DbwCanToAutoware(Node):
    def __init__(self) -> None:
        super().__init__("dbw_can_to_autoware")

        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        qos_tl = QoSProfile(depth=1)
        qos_tl.history = QoSHistoryPolicy.KEEP_LAST
        qos_tl.reliability = QoSReliabilityPolicy.RELIABLE
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.declare_parameter("steering_wheel_to_tire_ratio", 14.8)

        # NOTE: Smooth longitudinal behavior is handled in autoware_to_dbw_can.py.

        # DBW topics
        self.declare_parameter("dbw_dbw_enabled", "/vehicle/dbw_enabled")
        self.declare_parameter("dbw_steering_report", "/vehicle/steering_report")
        self.declare_parameter("dbw_gear_report", "/vehicle/gear_report")
        self.declare_parameter("dbw_misc_report", "/vehicle/misc_1_report")

        # Hazard shim input
        self.declare_parameter("hazard_shim_topic", "/mkz_interface/hazard_cmd_latched")

        # Autoware status topics
        self.declare_parameter("aw_velocity_status", "/vehicle/status/velocity_status")
        self.declare_parameter("aw_steering_status", "/vehicle/status/steering_status")
        self.declare_parameter("aw_gear_status", "/vehicle/status/gear_status")
        self.declare_parameter("aw_turn_status", "/vehicle/status/turn_indicators_status")
        self.declare_parameter("aw_hazard_status", "/vehicle/status/hazard_lights_status")
        self.declare_parameter("aw_control_mode_status", "/vehicle/status/control_mode")
        self.declare_parameter("aw_steering_status_for_gate", "/vehicle_cmd_gate/input/steering")

        self.ratio = float(self.get_parameter("steering_wheel_to_tire_ratio").value)

        # pubs
        self.pub_vel = self.create_publisher(VelocityReport, self.get_parameter("aw_velocity_status").value, qos)
        self.pub_steer = self.create_publisher(AwSteeringReport, self.get_parameter("aw_steering_status").value, qos)
        self.pub_gate_steer = self.create_publisher(AwSteeringReport, self.get_parameter("aw_steering_status_for_gate").value, qos)
        self.pub_gear = self.create_publisher(AwGearReport, self.get_parameter("aw_gear_status").value, qos)
        self.pub_turn = self.create_publisher(TurnIndicatorsReport, self.get_parameter("aw_turn_status").value, qos)
        self.pub_haz = self.create_publisher(HazardLightsReport, self.get_parameter("aw_hazard_status").value, qos)
        self.pub_mode = self.create_publisher(ControlModeReport, self.get_parameter("aw_control_mode_status").value, qos)

        # subs
        self.create_subscription(Bool, self.get_parameter("dbw_dbw_enabled").value, self._on_enabled, qos_tl)
        self.create_subscription(SteeringReport, self.get_parameter("dbw_steering_report").value, self._on_steer, qos)
        self.create_subscription(GearReport, self.get_parameter("dbw_gear_report").value, self._on_gear, qos)
        self.create_subscription(Misc1Report, self.get_parameter("dbw_misc_report").value, self._on_misc, qos)
        self.create_subscription(UInt8, self.get_parameter("hazard_shim_topic").value, self._on_hazard_shim, qos)

        self.hazard_latched = 0
        self.last_dbw_gear = 0  # latest DBW gear state (used to sign velocity in reverse)

        self.get_logger().info("MKZ DBW1 state bridge ready (publishing Autoware vehicle status topics).")

    def _on_hazard_shim(self, msg: UInt8) -> None:
        self.hazard_latched = int(msg.data)

    def _on_enabled(self, msg: Bool) -> None:
        cm = ControlModeReport()
        cm.stamp = self.get_clock().now().to_msg()
        cm.mode = _const(ControlModeReport, "AUTONOMOUS", 1) if bool(msg.data) else _const(ControlModeReport, "MANUAL", 0)
        self.pub_mode.publish(cm)

    def _on_steer(self, msg: SteeringReport) -> None:
        tire = float(msg.steering_wheel_angle) / max(1e-6, self.ratio)

        sr = AwSteeringReport()
        sr.stamp = self.get_clock().now().to_msg()
        sr.steering_tire_angle = float(tire)
        self.pub_steer.publish(sr)
        self.pub_gate_steer.publish(sr)

        vr = VelocityReport()
        vr.header.stamp = self.get_clock().now().to_msg()
        vr.header.frame_id = "base_link"
        # Dataspeed SteeringReport.speed is a magnitude. Use latest gear report to sign it in reverse.
        signed_speed = float(msg.speed)
        if int(self.last_dbw_gear) == 2:  # 2 = REVERSE (DBW GearReport)
            signed_speed = -abs(signed_speed)
        else:
            signed_speed = abs(signed_speed)
        vr.longitudinal_velocity = signed_speed
        vr.lateral_velocity = 0.0
        vr.heading_rate = 0.0
        self.pub_vel.publish(vr)

        hz = HazardLightsReport()
        hz.stamp = self.get_clock().now().to_msg()
        hz.report = _const(HazardLightsReport, "ENABLE", 1) if self.hazard_latched != 0 else _const(HazardLightsReport, "DISABLE", 0)
        self.pub_haz.publish(hz)

    def _on_gear(self, msg: GearReport) -> None:
        dbw = 0
        try:
            dbw = int(msg.state.gear)
        except Exception:
            try:
                dbw = int(msg.state)
            except Exception:
                dbw = 0

        self.last_dbw_gear = int(dbw)

        gr = AwGearReport()
        gr.stamp = self.get_clock().now().to_msg()
        gr.report = int(_dbw_gear_to_aw(dbw))
        self.pub_gear.publish(gr)

    def _on_misc(self, msg: Misc1Report) -> None:
        ts = 0
        if hasattr(msg, "turn_signal"):
            try:
                ts = int(msg.turn_signal.value) if hasattr(msg.turn_signal, "value") else int(msg.turn_signal)
            except Exception:
                ts = 0

        tr = TurnIndicatorsReport()
        tr.stamp = self.get_clock().now().to_msg()

        if ts == 1:
            tr.report = _const(TurnIndicatorsReport, "ENABLE_LEFT", 1)
        elif ts == 2:
            tr.report = _const(TurnIndicatorsReport, "ENABLE_RIGHT", 2)
        else:
            tr.report = _const(TurnIndicatorsReport, "DISABLE", 0)

        self.pub_turn.publish(tr)


def main() -> None:
    rclpy.init()
    node = DbwCanToAutoware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
