mkz_interface

To build mkz_interface package only.

cd ~/autoware_mkz_ws
colcon build --merge-install --symlink-install --packages-select mkz_interface
# colcon build --symlink-install --packages-select mkz_interface

Keep in mind this package is named mkz_interface, whereas the other package in vehicle is mkz_vehicle_launch


To launch mkz_interface.
Source and launch


source install/setup.bash
source ~/autoware_mkz_ws/install/setup.bash
ros2 launch mkz_interface interface.launch.py

# Engage (if you gate on engage)
ros2 topic pub --once /vehicle_cmd_gate/output/engage autoware_vehicle_msgs/msg/Engage "{engage: true}"

To turn steering wheel,brake,accel (test)
ros2 topic pub --rate 20 /vehicle_cmd_gate/output/command/control_cmd   autoware_control_msgs/msg/Control   "{lateral: {steering_tire_angle: 0.00}, longitudinal: {acceleration: 0.00}}"

To use turn signals
ros2 topic pub /vehicle_cmd_gate/output/command/turn_indicators_cmd autoware_vehicle_msgs/msg/TurnIndicatorsCommand "{command: 3}"

To change gears (D 2-19) (R 20,21) (P 1) (N 3)
ros2 topic pub --once /vehicle_cmd_gate/output/command/gear_cmd \
  autoware_vehicle_msgs/msg/GearCommand "{command: 2}" 


ros2 interface show dbw_ford_msgs/msg/ThrottleCmd
ros2 interface show dbw_ford_msgs/msg/BrakeCmd

