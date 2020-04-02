Description to be written!

## Example

Start Gazebo:
```
roslaunch gazebo_ros empty_world.launch
```

Spawn a drone in Gazebo, enable position/yaw control and move it around
(remember to make `example_control` executable with `chmod +x example_control`):
```
roslaunch uav_gazebo spawn.launch
rostopic pub -1 /drone/switch_mode uav_gazebo_msgs/ControlMode "mode: 1"
rosrun uav_gazebo example_control
```
