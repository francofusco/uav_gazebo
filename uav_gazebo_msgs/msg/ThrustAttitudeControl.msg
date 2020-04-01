# Message definition to control the drone via a desired thrust and attitude.

# Vertical thrust to be exerted by the drone.
float64 thrust

# Desired attitude expressed as xyz Euler angles. Note that the resulting
# orientation matrix corresponds to Rx*Ry*Ry.
geometry_msgs/Vector3 attitude

# Optional angle rates. They can be useful if you want to track a given
# trajectory.
geometry_msgs/Vector3 attitude_rates
