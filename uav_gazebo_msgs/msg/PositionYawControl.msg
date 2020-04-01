# Message definition to control the position and yaw rotation of the drone.

# Position setpoint to be reached by the drone.
geometry_msgs/Vector3 position

# Optional desired velocity and acceleration. They can be useful if you want
# to track a given trajectory. Simply set them to zero if you just want the
# drone to reach a fixed position.
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

# Desired yaw (Z) angle of the drone.
float64 yaw

# Optional yaw "velocity".
float64 yaw_rate
