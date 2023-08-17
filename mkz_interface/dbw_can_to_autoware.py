import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from dbw_ford_msgs.msg import Misc1Report
from dbw_ford_msgs.msg import SteeringReport as SteeringReport_dbw
from dbw_ford_msgs.msg import GearReport as GearReport_dbw
from autoware_auto_vehicle_msgs.msg import VelocityReport, TurnIndicatorsReport, GearReport, SteeringReport, ControlModeReport, HazardLightsReport

class Dbw_can_to_Autoware(Node):
    def __init__(self):
        super().__init__('dbw_can_to_autoware')

        # Publishers
        self.pub_velocity = self.create_publisher(VelocityReport,'/vehicle/status/velocity_status', 10)
        self.pub_turn_signal = self.create_publisher(TurnIndicatorsReport,'/vehicle/status/turn_indicators_status', 10)
        self.pub_hazard_signal = self.create_publisher(HazardLightsReport, '/vehicle/status/hazard_lights_status', 10)
        self.pub_gear = self.create_publisher(GearReport,'/vehicle/status/gear_status', 10)
        self.pub_steering = self.create_publisher(SteeringReport,'/vehicle/status/steering_status', 10)
        self.pub_control_mode = self.create_publisher(ControlModeReport,'/vehicle/status/control_mode', 10)

        # Subscribers
        self.subscription_twist = self.create_subscription(TwistStamped, '/vehicle/twist', self.callback_twist, 10)
        self.subscription_turn_signal = self.create_subscription(Misc1Report, '/vehicle/misc_1_report', self.callback_turn_signal, 10)
        self.subscription_gear = self.create_subscription(GearReport_dbw, '/vehicle/gear_report', self.callback_gear, 10)
        self.subscription_steering = self.create_subscription(SteeringReport_dbw, '/vehicle/steering_report', self.callback_steering, 10)
        self.subscription_control_mode = self.create_subscription(Bool, '/vehicle/dbw_enabled', self.callback_control_mode, 10)

        #self.timer = self.create_timer(0.1, self.publish_all_msgs)

    def callback_twist(self, data):
        msg = VelocityReport()
        msg.header = data.header
        msg.longitudinal_velocity = data.twist.linear.x
        msg.lateral_velocity = data.twist.linear.y
        msg.heading_rate = data.twist.angular.z

        self.pub_velocity.publish(msg)

    def callback_turn_signal(self, data):
        turn_msg = TurnIndicatorsReport()
        turn_msg.stamp = self.get_clock().now().to_msg()
        # msg.stamp = data.header.stamp
        conversion_dict = {0:1, 1:2, 2:3}
        turn_msg.report = conversion_dict[data.turn_signal.value]
        self.pub_turn_signal.publish(turn_msg)

        hazard_msg = HazardLightsReport()
        hazard_msg.stamp = turn_msg.stamp
        if data.turn_signal.value == 3:
            hazard_msg.report = 2
        else:
            hazard_msg.report = 1

        self.pub_hazard_signal.publish(hazard_msg)

    def callback_gear(self, data):
        msg = GearReport()
        msg.stamp = self.get_clock().now().to_msg()
        conversion_dict = {0:0,1:22,2:20,3:1,4:2, 5:23}
        msg.report = conversion_dict[data.state.gear]

        self.pub_gear.publish(msg)

    def callback_steering(self, data):
        msg = SteeringReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.steering_tire_angle = data.steering_wheel_angle

        self.pub_steering.publish(msg)

    def callback_control_mode(self, data):
        msg = ControlModeReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = data.data

        self.pub_control_mode.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    converter_node = Dbw_can_to_Autoware()
    converter_node.get_logger().info("CAN to Autoware Node starts")

    rclpy.spin(converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

