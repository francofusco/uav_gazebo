Description to be written!

## Example

In three different consoles:

```
roslaunch gazebo_ros empty_world.launch

roslaunch uav_gazebo spawn.launch

rostopic pub -1 /command uav_gazebo_msgs/PositionYawControl "position: {x: 0.0, y: 0.0, z: 1.0}
velocity: {x: 0.0, y: 0.0, z: 0.0}
acceleration: {x: 0.0, y: 0.0, z: 0.0}
yaw: 0.0
yaw_rate: 0.0"
```
