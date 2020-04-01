cmake_minimum_required(VERSION 2.8.3)
project(uav_gazebo_msgs)
add_compile_options(-std=c++11)

# find required packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ControlMode.msg
  PositionYawControl.msg
  VelocityYawRateControl.msg
  ThrustAttitudeControl.msg
  ThrustVelocityControl.msg
  ThrustTorqueControl.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
