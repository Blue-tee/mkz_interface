import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist
from dbw_ford_msgs.msg import GearCmd, BrakeCmd, SteeringCmd, ThrottleCmd, MiscCmd
from dataspeed_ulc_msgs.msg import UlcCmd
import dataspeed_ulc_msgs, dbw_ford_msgs
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand, GearCommand, HazardLightsCommand
from autoware_auto_control_msgs.msg import AckermannControlCommand
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped


class Autoware_to_Dbw_can(Node):
    def __init__(self):
        super().__init__('autoware_to_dbw_can')

        # Publishers
        self.pub_ulc = self.create_publisher(UlcCmd,'/vehicle/ulc_cmd', 10)
        #self.pub_brake = self.create_publisher(BrakeCmd,'/vehicle/brake_cmd', 10)
        #self.pub_throttle = self.create_publisher(ThrottleCmd,'/vehicle/throttle_cmd', 10)
        #self.pub_misc = self.create_publisher(MiscCmd,'/vehicle/misc_cmd', 10)
        #self.pub_gear = self.create_publisher(GearCmd,'/vehicle/status/gear_cmd', 10)
        self.pub_steering = self.create_publisher(SteeringCmd,'/vehicle/steering_cmd', 10)
        #self.pub_control_mode = self.create_publisher(Empty,'/vehicle/enable', 10)

        # Subscribers
        self.subscription_control = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.callback_control, 10)
        self.subscription_velocity = self.create_subscription(VelocityReport, '/vehicle/status/velocity_status', self.callback_velocity, 10)
        self.subscription_emergency = self.create_subscription(VehicleEmergencyStamped, '/control/command/emergency_cmd', self.callback_emergency, 10)
        #self.subscription_gear = self.create_subscription(GearCommand, '/control/command/gear_cmd', self.callback_gear, 10)
        self.subscription_hazard= self.create_subscription(HazardLightsCommand, '/control/command/hazard_lights_cmd', self.callback_hazard, 10)
        #self.subscription_turn_signal = self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.callback_turn_signal, 10)

 
        self.misc_msg = MiscCmd()
        self.gear_msg = GearCmd()
        self.wheel_base = 2.8498
        self.current_velocity = 0

    def callback_control(self, data):
        ulc_msg = UlcCmd()
        ulc_msg.header.stamp = data.stamp
        #ulc_msg.header.frame_id = "base_link"
        ulc_msg.clear = True
        ulc_msg.enable_pedals = True
        ulc_msg.enable_steering = False
        ulc_msg.enable_shifting = True
        ulc_msg.shift_from_park = True

        ulc_msg.pedals_mode = dataspeed_ulc_msgs.msg.UlcCmd.SPEED_MODE
        ulc_msg.accel_cmd = 0.0
        ulc_msg.coast_decel = False
        ulc_msg.steering_mode = dataspeed_ulc_msgs.msg.UlcCmd.YAW_RATE_MODE

        ulc_msg.linear_velocity = data.longitudinal.speed
        ulc_msg.yaw_command = (math.tan(data.lateral.steering_tire_angle) * self.current_velocity ) / self.wheel_base

        ulc_msg.linear_accel = 0.0
        ulc_msg.linear_decel = 0.0
        ulc_msg.angular_accel = 0.0
        ulc_msg.lateral_accel = 0.0
        ulc_msg.jerk_limit_throttle = 0.0
        ulc_msg.jerk_limit_brake = 0.0

        self.pub_ulc.publish(ulc_msg)

        steering_msg = SteeringCmd()
        steering_msg.enable = True
        steering_msg.ignore = False
        steering_msg.count = False
        steering_msg.cmd_type = dbw_ford_msgs.msg.SteeringCmd.CMD_ANGLE
        steering_msg.steering_wheel_angle_cmd = data.lateral.steering_tire_angle
        steering_msg.steering_wheel_angle_velocity = 0.0
        self.pub_steering.publish(steering_msg)

    def callback_velocity(self, data):
        self.current_velocity = data.longitudinal_velocity

    def callback_emergency(self, data):
        pass

    def callback_gear(self, data):
        self.gear_msg.header.stamp = self.get_clock().now().to_msg()
        conversion_dict = {0:0, 1:3, 2:4, 20:2, 22:1, 23:5}
        self.gear_msg.cmd.gear = conversion_dict[data.command]

    def callback_hazard(self, data):
        pass


    def callback_turn_signal(self, data):
        self.misc_msg.header.stamp = self.get_clock().now().to_msg()
        conversion_dict = {0:0,1:0, 2:1, 3:2}
        self.misc_msg.cmd.value = conversion_dict[data.command]

    def publish_all_msgs(self):
        self.pub_brake.publish(self.brake_msg)
        self.pub_throttle.publish(self.throttle_msg)
        self.pub_misc.publish(self.misc_msg)
        self.pub_gear.publish(self.gear_msg)
        self.pub_steering.publish(self.steering_msg)
        self.pub_control_mode.publish(self.control_mode_msg)


def main(args=None):
    rclpy.init(args=args)
    converter_node = Autoware_to_Dbw_can()
    converter_node.get_logger().info("Autoware to CAN Node starts")

    rclpy.spin(converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


