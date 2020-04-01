# Message definition to directly control the drone via a 4D wrench.

# Vertical thrust.
float64 thrust

# Angular moments (should be expressed in the world frame).
geometry_msgs/Vector3 torque
