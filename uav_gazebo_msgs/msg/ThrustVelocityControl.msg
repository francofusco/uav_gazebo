# Message definition to control a drone by setting the vertical thrust and an
# angular velocity.

# Vertical thrust.
float64 thrust

# Desired angular velocity (should be expressed in the world frame).
geometry_msgs/Vector3 velocity

# Optional desired acceleration (in the world frame).
geometry_msgs/Vector3 acceleration
