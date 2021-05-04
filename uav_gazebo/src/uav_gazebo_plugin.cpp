#include "uav_gazebo/uav_gazebo_plugin.hpp"
#include "uav_gazebo/utils.hpp"
#include <angles/angles.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#define _LOG_NAME_ "uav_gazebo_plugin"



namespace gazebo {

const double UavGazeboPlugin::gravity = 9.81;
const double UavGazeboPlugin::fz_tol = 0.01;


void UavGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  // this plugin needs ROS, so check if it is running in a valid node
  if(!ros::isInitialized()) {
    ROS_ERROR_NAMED(_LOG_NAME_, "UavGazeboPlugin: ros::init not "
      "called, cannot create connection with master.");
    return;
  }

  // +++ MODEL INFO +++
  model_ = model;
  link_ = model_->GetLink();
  auto inertial = link_->GetInertial();

  // This plugin works for systems whose center of gravity is located at the
  // origin of the link (I could create a workaround, but I am a little lazy...)
  if(inertial->CoG().Length() > 1e-9) {
    ROS_ERROR_NAMED(_LOG_NAME_, "The CoG of link %s should be zero",
      link_->GetName().c_str());
    return;
  }

  // mass and inertia tensor
  M = inertial->Mass();
  Ib = inertial->MOI();

  ROS_DEBUG_STREAM_NAMED(_LOG_NAME_, "Mass: " << M << std::endl << "Inertia:" << std::endl << Ib);

  // +++ ROS CONNECTION +++
  if(sdf->HasElement("rosNamespace")) {
    std::string ns = sdf->GetElement("rosNamespace")->Get<std::string>();
    nh_ = ros::NodeHandle(ns);
  }

  // tf publisher to provide the pose of the drone
  tf_.reset( new tf2_ros::TransformBroadcaster() );

  // topics for drone control
  control.mode = control.INACTIVE; // the drone starts idle
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
  get_mode_srv_ = nh_.advertiseService("get_mode", &UavGazeboPlugin::getControlMode, this);
  switch_mode_srv_ = nh_.advertiseService("switch_mode", &UavGazeboPlugin::switchControlMode, this);
  pos_yaw_sub_ = nh_.subscribe("position_yaw/command", 1, &UavGazeboPlugin::positionYawCB, this);
  vel_yawrate_sub_ = nh_.subscribe("velocity_yawrate/command", 1, &UavGazeboPlugin::velocityYawrateCB, this);
  thr_att_sub_ = nh_.subscribe("thrust_attitude/command", 1, &UavGazeboPlugin::thrustAttitudeCB, this);
  thr_vel_sub_ = nh_.subscribe("thrust_velocity/command", 1, &UavGazeboPlugin::thrustVelocityCB, this);
  thr_trq_sub_ = nh_.subscribe("thrust_torque/command", 1, &UavGazeboPlugin::thrustTorqueCB, this);

  // dynamic reconfigure
  gains_server_.reset(
    new dynamic_reconfigure::Server<uav_gazebo_msgs::ControlGainsConfig>(
      ros::NodeHandle(nh_, "gains")
    )
  );
  gains_server_->setCallback(boost::bind(&UavGazeboPlugin::setGains, this, _1, _2));

  // +++ SIMULATION SETUP +++
  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&UavGazeboPlugin::update, this));

  ROS_INFO_NAMED(_LOG_NAME_, _LOG_NAME_ " initialized (model: %s, "
    "inertial link: %s)", model_->GetName().c_str(), link_->GetName().c_str());
}


void UavGazeboPlugin::doPositionYawControl(
  const im::Vector3d& p_star,
  const im::Vector3d& v_star,
  const im::Vector3d& vd_star,
  const double yaw_star,
  const double yaw_rate_star
)
{
  im::Vector3d acc = vd_star + pos_kd * (v_star-linvel) + pos_kp * (p_star-pose.Pos());
  double thrust;
  im::Vector3d rho(0, 0, yaw_star);
  accelerationToThrustAndAttitude(acc, thrust, rho);
  doThrustAttitudeControl(thrust, rho, im::Vector3d(0, 0, yaw_rate_star), false);
}


void UavGazeboPlugin::doVelocityYawRateControl(
  const im::Vector3d& v_star,
  const im::Vector3d& vd_star,
  const double yaw_rate_star,
  bool body_frame
)
{
  im::Vector3d acc =
    body_frame
    ? pose.Rot() * ( vd_star + linvel_k * (v_star-pose.Inverse().Rot()*linvel) )
    : vd_star + linvel_k * (v_star-linvel);
  double thrust;
  im::Vector3d rho;
  accelerationToThrustAndAttitude(acc, thrust, rho);
  doThrustAttitudeControl(thrust, rho, im::Vector3d(0, 0, yaw_rate_star), true);
}


void UavGazeboPlugin::accelerationToThrustAndAttitude(
  const im::Vector3d& acceleration,
  double& thrust,
  im::Vector3d& rho
)
{
  // add the gravity acceleration and multiply by the mass to get the force
  im::Vector3d f3 = M * ( acceleration + im::Vector3d(0,0,gravity) );
  // make sure that the force points upward
  if(f3.Z() < fz_tol)
    f3.Z() = fz_tol;
  // evaluate the thrust and desired attitude angles
  thrust = f3.Length();
  rho.X() = std::atan2(-f3.Y(), f3.Z());
  rho.Y() = std::asin(f3.X()/thrust);
}


void UavGazeboPlugin::doThrustAttitudeControl(
  double thrust,
  const im::Vector3d& rho_star,
  const im::Vector3d& rhod_star,
  bool yaw_rate_only
)
{
  // extract current attitude
  im::Vector3d rho = rotation2euler_xyz(im::Matrix3d(pose.Rot()));

  // matrix to map angular rates into angular velocity
  double sp = std::sin(rho.X());
  double cp = std::cos(rho.X());
  double st = std::sin(rho.Y());
  double ct = std::cos(rho.Y());
  im::Matrix3d D(
    1,  0, st,
    0, cp, -sp*ct,
    0, sp,  cp*ct
  );

  // angular rates from current velocity
  im::Vector3d rhod = D.Inverse() * angvel;

  // derivative of D
  im::Matrix3d Dd(
    0, 0, rhod.Y()*ct,
    0, -rhod.X()*sp,  rhod.Y()*st*sp-rhod.X()*ct*cp,
    0,  rhod.X()*ct, -rhod.Y()*st*cp-rhod.X()*sp*ct
  );

  // evaluate the attitude commands
  im::Vector3d rhodd;
  rhodd.X() = att_kd * (rhod_star.X()-rhod.X()) + att_kp * (rho_star.X()-rho.X());
  rhodd.Y() = att_kd * (rhod_star.Y()-rhod.Y()) + att_kp * (rho_star.Y()-rho.Y());
  rhodd.Z() = yaw_rate_only ?
    yaw_rate_k * (rhod_star.Z() - rhod.Z())
    :
    yaw_kd * (rhod_star.Z() - rhod.Z()) + yaw_kp * angles::shortest_angular_distance(rho.Z(), rho_star.Z());

  // evaluate the angular acceleration corresponding to the current rate values
  im::Vector3d wd_cmd = D * rhodd + Dd * rhod;

  doThrustAngularAccelerationControl(thrust, wd_cmd);
}


void UavGazeboPlugin::doThrustVelocityControl(
  double thrust,
  const im::Vector3d& w_star,
  const im::Vector3d& wd_star
)
{
  doThrustAngularAccelerationControl(
    thrust,
    wd_star + angvel_k * (w_star - angvel)
  );
}


void UavGazeboPlugin::doThrustAngularAccelerationControl(
  double thrust,
  const im::Vector3d& wd
)
{
  // Inertia tensor in world frame coordinates:
  //  Iw = wRb * Ib * bRw
  im::Matrix3d Iw = im::Matrix3d(pose.Rot()) * Ib * im::Matrix3d(pose.Inverse().Rot());
  // angular moment
  doThrustTorqueControl(
    thrust,
    Iw*wd + angvel.Cross(Iw*angvel)
  );
}


void UavGazeboPlugin::doThrustTorqueControl(
  double thrust,
  const im::Vector3d& torque
)
{
  // check that the thrust is positive, if not just "cut it"
  if(thrust < 0) {
    ROS_WARN_THROTTLE_NAMED(1.0, _LOG_NAME_, "Received negative thrust command, setting it to 0");
    thrust = 0;
  }
  // set the wrench
  link_->AddRelativeForce(im::Vector3d(0,0,thrust));
  link_->AddTorque(torque);
}


void UavGazeboPlugin::update() {
  // get current state of the drone
  pose = link_->WorldPose();
  linvel = link_->WorldLinearVel();
  angvel = link_->WorldAngularVel();

  // publish current pose on /tf
  geometry_msgs::TransformStamped tf_msg;
  pose2transform(pose, tf_msg.transform);
  tf_msg.child_frame_id = link_->GetName();
  tf_msg.header.frame_id = "world";
  tf_msg.header.stamp = ros::Time::now();
  tf_->sendTransform(tf_msg);

  // publish the pose and velocity of the drone
  nav_msgs::Odometry odom_msg;
  odom_msg.header = tf_msg.header;
  odom_msg.child_frame_id = tf_msg.child_frame_id;
  pose2pose(pose, odom_msg.pose.pose);
  vector2vector(linvel, odom_msg.twist.twist.linear);
  vector2vector(angvel, odom_msg.twist.twist.angular);
  odom_pub_.publish(odom_msg);

  // do drone control
  switch(control.mode) {
    case control.INACTIVE:
      doThrustTorqueControl(0, im::Vector3d());
      break;
    case control.POSITION_YAW:
      doPositionYawControl(
        vector2vector(position_yaw.position),
        vector2vector(position_yaw.velocity),
        vector2vector(position_yaw.acceleration),
        position_yaw.yaw,
        position_yaw.yaw_rate
      );
      break;
    case control.VELOCITY_YAWRATE:
      doVelocityYawRateControl(
        vector2vector(velocity_yawrate.velocity),
        vector2vector(velocity_yawrate.acceleration),
        velocity_yawrate.yaw_rate,
        velocity_yawrate.body_frame
      );
      break;
    case control.THRUST_ATTITUDE:
      doThrustAttitudeControl(
        thrust_attitude.thrust,
        vector2vector(thrust_attitude.attitude),
        vector2vector(thrust_attitude.attitude_rates),
        false
      );
      break;
    case control.THRUST_VELOCITY:
      doThrustVelocityControl(
        thrust_velocity.thrust,
        vector2vector(thrust_velocity.velocity),
        vector2vector(thrust_velocity.acceleration)
      );
      break;
    case control.THRUST_TORQUE:
      doThrustTorqueControl(
        thrust_torque.thrust,
        vector2vector(thrust_torque.torque)
      ); break;
    default:
      ROS_ERROR_NAMED(_LOG_NAME_, "Reached invalid point in switch "
        "clause - file %s, line %d", __FILE__, __LINE__);
  }
}


void UavGazeboPlugin::warnIfDifferentMode(unsigned int mode) {
  if(control.mode != mode) {
    ROS_WARN_THROTTLE_NAMED(1.0, _LOG_NAME_, "Received message for control mode "
      "%u, but the current control mode is %u", mode, control.mode);
  }
}

void UavGazeboPlugin::positionYawCB(const uav_gazebo_msgs::PositionYawControl& msg) {
  warnIfDifferentMode(control.POSITION_YAW);
  position_yaw = msg;
}

void UavGazeboPlugin::velocityYawrateCB(const uav_gazebo_msgs::VelocityYawRateControl& msg) {
  warnIfDifferentMode(control.VELOCITY_YAWRATE);
  velocity_yawrate = msg;
}

void UavGazeboPlugin::thrustAttitudeCB(const uav_gazebo_msgs::ThrustAttitudeControl& msg) {
  warnIfDifferentMode(control.THRUST_ATTITUDE);
  thrust_attitude = msg;
}

void UavGazeboPlugin::thrustVelocityCB(const uav_gazebo_msgs::ThrustVelocityControl& msg) {
  warnIfDifferentMode(control.THRUST_VELOCITY);
  thrust_velocity = msg;
}

void UavGazeboPlugin::thrustTorqueCB(const uav_gazebo_msgs::ThrustTorqueControl& msg) {
  warnIfDifferentMode(control.THRUST_TORQUE);
  thrust_torque = msg;
}


bool UavGazeboPlugin::getControlMode(
  uav_gazebo_msgs::GetMode::Request& request,
  uav_gazebo_msgs::GetMode::Response& response
)
{
  response.mode.mode = control.mode;
  return true;
}


bool UavGazeboPlugin::switchControlMode(
  uav_gazebo_msgs::SwitchMode::Request& request,
  uav_gazebo_msgs::SwitchMode::Response& response
)
{
  if(request.mode.mode == control.mode) {
    ROS_INFO_NAMED(_LOG_NAME_, "The drone is already in the desired control "
      "mode %u", control.mode);
  }
  else {
    ROS_INFO_NAMED(_LOG_NAME_, "Switching from control mode %u to %u",
      control.mode, request.mode.mode);
    control.mode = request.mode.mode;
  }
  response.success = true;
  return true;
}


void UavGazeboPlugin::setGains(uav_gazebo_msgs::ControlGainsConfig &cfg, uint32_t level) {
  secondOrderGains(cfg.Wp, cfg.Xp, pos_kp, pos_kd);
  linvel_k = cfg.Kv;
  secondOrderGains(cfg.Wa, cfg.Xa, att_kp, att_kd);
  secondOrderGains(cfg.Wy, cfg.Xy, yaw_kp, yaw_kd);
  yaw_rate_k = cfg.Ky;
  angvel_k = cfg.Kw;

  ROS_DEBUG_STREAM_NAMED(_LOG_NAME_, "Received dynamic reconfigure request."
    << std::endl << "Position controller: kp=" << pos_kp << "  kd=" << pos_kd
    << std::endl << "Linear velocity controller: k=" << linvel_k
    << std::endl << "Attitude controller: kp=" << att_kp << "  kd=" << att_kd
    << std::endl << "Yaw controller: kp=" << yaw_kp << "  kd=" << yaw_kd
    << std::endl << "Angular velocity controller: k=" << angvel_k
  );
}


GZ_REGISTER_MODEL_PLUGIN(UavGazeboPlugin)

} // end of namespace
