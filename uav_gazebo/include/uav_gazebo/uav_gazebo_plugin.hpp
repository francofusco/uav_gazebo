#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <std_msgs/Empty.h>

#include <uav_gazebo_msgs/PositionYawControl.h>
#include <uav_gazebo_msgs/VelocityYawRateControl.h>
#include <uav_gazebo_msgs/ThrustAttitudeControl.h>
#include <uav_gazebo_msgs/ThrustVelocityControl.h>
#include <uav_gazebo_msgs/ThrustTorqueControl.h>
#include <uav_gazebo_msgs/ControlMode.h>


namespace gazebo {

namespace im = ignition::math;

/// Plugin to control a UAV in Gazebo.
class UavGazeboPlugin : public ModelPlugin {
public:
  static const double gravity;
  static const double fz_tol;

  /// Constructor.
  UavGazeboPlugin() : ModelPlugin() { }

  /// Initializes the plugin.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  /// Compute the control to be applied to the drone.
  void update();

  /// Auxiliary function to do position control.
  void doPositionYawControl(
    const im::Vector3d& p_star,
    const im::Vector3d& v_star,
    const im::Vector3d& vd_star,
    const double yaw_star,
    const double yaw_rate_star
  );

  /// Auxiliary function to do velocity control.
  void doVelocityYawRateControl(
    const im::Vector3d& v_star,
    const im::Vector3d& vd_star,
    const double yaw_rate_star
  );

  /// Auxiliary function to evaluate the desired thrust and attitude from a 3D acceleration.
  void accelerationToThrustAndAttitude(
    const im::Vector3d& acceleration,
    double& thrust,
    im::Vector3d& rho
  );

  /// Auxiliary function to do thrust and attitude control.
  void doThrustAttitudeControl(
    double thrust,
    const im::Vector3d& rho_star,
    const im::Vector3d& rhod_star,
    bool yaw_rate_only
  );

  /// Auxiliary function to do thrust and angular velocity control.
  void doThrustVelocityControl(
    double thrust,
    const im::Vector3d& w_star,
    const im::Vector3d& wd_star
  );

  /// Auxiliary function to do thrust and angular acceleration control.
  void doThrustAngularAccelerationControl(
    double thrust,
    const im::Vector3d& wd
  );

  /// Auxiliary function to do low-level control of the drone.
  void doThrustTorqueControl(
    double thrust,
    const im::Vector3d& torque
  );

  /// Prints a warning if the current control mode is differs from the given one.
  void warnIfDifferentMode(unsigned int mode);

  /// Callback to get desired position and yaw.
  void positionYawCB(const uav_gazebo_msgs::PositionYawControl& msg);

  /// Callback to get desired velocity and yaw rate.
  void velocityYawrateCB(const uav_gazebo_msgs::VelocityYawRateControl& msg);

  /// Callback to get desired thrust and attitude.
  void thrustAttitudeCB(const uav_gazebo_msgs::ThrustAttitudeControl& msg);

  /// Callback to get desired thrust and angular velocity.
  void thrustVelocityCB(const uav_gazebo_msgs::ThrustVelocityControl& msg);

  /// Callback to get desired thrust and torque.
  void thrustTorqueCB(const uav_gazebo_msgs::ThrustTorqueControl& msg);

  /// The model this plugin was instanciated for.
  physics::ModelPtr model_;
  /// The link that contains the dynamic properties of the drone.
  physics::LinkPtr link_;
  /// Connection to perform control at each iteration.
  event::ConnectionPtr update_connection_;

  im::Matrix3d I; ///< Inertia of the drone.
  double M; ///< Mass of the drone.
  im::Pose3d pose; ///< Pose of the drone.
  im::Vector3d linvel; ///< Linear velocity of the drone (in the world frame).
  im::Vector3d angvel; ///< Angular velocity of the drone (in the world frame).

  // Variables to store the current control mode and setpoints
  uav_gazebo_msgs::ControlMode control;
  uav_gazebo_msgs::PositionYawControl position_yaw;
  uav_gazebo_msgs::VelocityYawRateControl velocity_yawrate;
  uav_gazebo_msgs::ThrustAttitudeControl thrust_attitude;
  uav_gazebo_msgs::ThrustVelocityControl thrust_velocity;
  uav_gazebo_msgs::ThrustTorqueControl thrust_torque;

  // TEMP: following values should be regrouped, eg, in a dynamic reconfigure CFG
  double pos_kp, pos_kd, vel_k;
  double ang_kp, ang_kd;
  double yaw_kp, yaw_kd, yaw_rate_k;
  double angvel_k;

  /// Node Handle to connect with ROS
  ros::NodeHandle nh_;
  /// ROS subscriber to get the current command
  ros::Subscriber cmd_sub_;
};

} // end of namespace
