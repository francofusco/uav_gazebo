cmake_minimum_required(VERSION 2.8.3)
project(uav_gazebo)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  geometry_msgs
  roscpp
  angles
  uav_gazebo_msgs
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugin SHARED src/${PROJECT_NAME}_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} -Wl,--no-undefined)

install(
  TARGETS ${PROJECT_NAME}_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
