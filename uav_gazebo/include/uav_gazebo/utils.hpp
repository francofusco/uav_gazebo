#pragma once

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

/// Utility function to copy a pose from Gazebo to a ROS message.
inline void pose2transform(const ignition::math::Pose3d& pose, geometry_msgs::Transform& transform)
{
  transform.translation.x = pose.Pos().X();
  transform.translation.y = pose.Pos().Y();
  transform.translation.z = pose.Pos().Z();
  transform.rotation.w = pose.Rot().W();
  transform.rotation.x = pose.Rot().X();
  transform.rotation.y = pose.Rot().Y();
  transform.rotation.z = pose.Rot().Z();
}

/// Utility function to copy a pose from Gazebo to a ROS message.
inline void pose2pose(const ignition::math::Pose3d& pose, geometry_msgs::Pose& transform)
{
  transform.position.x = pose.Pos().X();
  transform.position.y = pose.Pos().Y();
  transform.position.z = pose.Pos().Z();
  transform.orientation.w = pose.Rot().W();
  transform.orientation.x = pose.Rot().X();
  transform.orientation.y = pose.Rot().Y();
  transform.orientation.z = pose.Rot().Z();
}

/// Utility function to evaluate the gains for a 2nd order control.
void secondOrderGains(double wn, double xi, double& kp, double& kd) {
  kp = wn*wn;
  kd = 2*xi*wn;
}

/// Extract the x-y-z Euler angles from a rotation matrix.
/** @param R input rotation matrix.
  * @param euler Euler angles corresponding to the input matrix.
  * @param inner_solution allows to switch between the two existing solutions.
  *   If inner_solution is true, then the middle angle (Y) will be in the range
  *   `[-pi/2,pi/2]`, otherwise it will be in the other half of the unit circle.
  */
inline ignition::math::Vector3d rotation2euler_xyz(
  const ignition::math::Matrix3d& R,
  bool inner_solution=true
)
{
  double a = std::asin(R(0,2));
  const double sgn(inner_solution ? 1. : -1.);
  return ignition::math::Vector3d(
    std::atan2(-sgn*R(1,2), sgn*R(2,2)),
    inner_solution ? a : ( a>0 ? M_PI-a : -M_PI-a ),
    std::atan2(-sgn*R(0,1), sgn*R(0,0))
  );
}

/// Convert a 3D vector from geometry_msgs to ignition format.
inline void vector2vector(const geometry_msgs::Vector3& in, ignition::math::Vector3d& out) {
  out.X() = in.x;
  out.Y() = in.y;
  out.Z() = in.z;
}

/// Convert a 3D vector from geometry_msgs to ignition format.
inline void vector2vector(const ignition::math::Vector3d& in, geometry_msgs::Vector3& out) {
  out.x = in.X();
  out.y = in.Y();
  out.x = in.Z();
}

/// Convert a 3D vector from geometry_msgs to ignition format.
inline ignition::math::Vector3d vector2vector(const geometry_msgs::Vector3& in) {
  return ignition::math::Vector3d(in.x, in.y, in.z);
}
