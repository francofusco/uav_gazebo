cmake_minimum_required(VERSION 2.8.3)
project(uav_gazebo_msgs)
add_compile_options(-std=c++11)

# find required packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  dynamic_reconfigure
)

add_message_files(
  FILES
  ControlMode.msg
  PositionYawControl.msg
  VelocityYawRateControl.msg
  ThrustAttitudeControl.msg
  ThrustVelocityControl.msg
  ThrustTorqueControl.msg
)

add_service_files(
  FILES
  GetMode.srv
  SwitchMode.srv
  Takeoff.srv
  Land.srv
)

generate_dynamic_reconfigure_options(
  cfg/ControlGains.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
