#include "uav_gazebo/uav_gazebo_plugin.hpp"
#include "uav_gazebo/utils.hpp"
#include <angles/angles.h>

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
  I = inertial->MOI();

  ROS_DEBUG_STREAM_NAMED(_LOG_NAME_, "Mass: " << M << std::endl << "Inertia:" << std::endl << I);

  // +++ CONTROL SETUP +++
  control.mode = control.POSITION_YAW;

  // TODO: CONFIGURATION VIA DYNAMIC RECONFIGURE
  double _wn;

  _wn = 0.5;
  pos_kd = 2*_wn;
  pos_kp = _wn*_wn;

  _wn = 3.;
  ang_kd = 2*_wn;
  ang_kp = _wn*_wn;

  vel_k = 1.0;

  _wn = 1.5;
  yaw_kd = 2*_wn;
  yaw_kp = _wn*_wn;

  yaw_rate_k = 2.0;

  angvel_k = 3.;

  // +++ ROS CONNECTION +++
  // TODO: custom namespace selected by the user (from URDF?)
  cmd_sub_ = nh_.subscribe("command", 1, &UavGazeboPlugin::positionYawCB, this);

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
  const double yaw_rate_star
)
{
  im::Vector3d acc = vd_star + vel_k * (v_star-linvel);
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
  rhodd.X() = ang_kd * (rhod_star.X()-rhod.X()) + ang_kp * (rho_star.X()-rho.X());
  rhodd.Y() = ang_kd * (rhod_star.Y()-rhod.Y()) + ang_kp * (rho_star.Y()-rho.Y());
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
  // angular moment
  doThrustTorqueControl(
    thrust,
    I*wd + angvel.Cross(I*angvel)
  );
}


void UavGazeboPlugin::doThrustTorqueControl(
  double thrust,
  const im::Vector3d& torque
)
{
  // set the wrench
  link_->AddRelativeForce(im::Vector3d(0,0,thrust));
  link_->AddTorque(torque);
}


void UavGazeboPlugin::update() {
  // get current state of the drone
  pose = link_->WorldPose();
  linvel = link_->WorldLinearVel();
  angvel = link_->WorldAngularVel();

  switch(control.mode) {
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
        velocity_yawrate.yaw_rate
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
    default: ROS_ERROR_NAMED(_LOG_NAME_, "REACHED INVALID POINT IN SWITCH CLAUSE");
  }
}








#ifdef SOMETHING_THAT_IS_NOT_DEFINED
void UavGazeboPlugin::update() {
  // get current state in the world frame
  pose_ = link_->WorldPose();
  linvel_ = link_->WorldLinearVel();
  angvel_ = link_->WorldAngularVel();

  im::Vector3d attitude_cmd(0,0,0);

  if(control.mode == control.POSITION_YAW) {
    // copy the comands
    auto& _position = position_yaw_.position;
    auto& _velocity = position_yaw_.velocity;
    im::Vector3d position_cmd(_position.x, _position.y, _position.z);
    im::Vector3d velocity_cmd(_velocity.x, _velocity.y, _velocity.z);
    // evaluate the force corresponding to the acceleration pseudo-control
    im::Vector3d force = M * ( pos_kp * (position_cmd - pose_.Pos()) + pos_kd * (velocity_cmd - linvel_) );
    force.Z() += M * gravity; // gravity compensation
    // make sure that the directional force points upward
    if(force.Z() < fz_tol) {
      force.Z() = fz_tol;
    }
    // convert the directional force into force & attitude commands
    f.Z() = force.Length();
    attitude_cmd.X() = std::atan2(-force.Y(), force.Z());
    attitude_cmd.Y() = std::asin(force.X()/f.Z());
    attitude_cmd.Z() = position_yaw_.yaw;
  }
  else {
    ROS_WARN_THROTTLE_NAMED(1.0, _LOG_NAME_, "Only POSITION_YAW control is available");
  }

  // Get current Euler angles from the pose
  // NOTE: do not use ignition's interface, since its Euler's angles are
  // Rz*Ry*Rx, not Rx*Ry*Rz
  im::Matrix3d R(pose_.Rot()); // current rotation matrix

  // Euler's angles from R
  im::Vector3d rho(
    std::atan2(-R(1,2), R(2,2)),
    std::asin(R(0,2)),
    std::atan2(-R(0,1), R(0,0))
  );

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

  // get angular rates from current velocity
  im::Vector3d rhod = D.Inverse() * angvel_;

  // derivative of D
  im::Matrix3d Dd(
    0, 0, rhod.Y()*ct,
    0, -rhod.X()*sp,  rhod.Y()*st*sp-rhod.X()*ct*cp,
    0,  rhod.X()*ct, -rhod.Y()*st*cp-rhod.X()*sp*ct
  );

  // attitude error (note: wrap around for the YAW!)
  im::Vector3d rho_err(
    attitude_cmd.X()-rho.X(),
    attitude_cmd.Y()-rho.Y(),
    angles::shortest_angular_distance(rho.Z(), attitude_cmd.Z())
  );

  // desired angular acceleration from desired and current attitude
  im::Vector3d wd =
    D * ( ang_kp * rho_err - ang_kd * rhod )
    + Dd * rhod;

  // angular moment
  tau = I * wd + angvel_.Cross(I*angvel_);

  // set the wrench
  f.X() = f.Y() = 0; // NOTE: this is just in case!
  link_->AddRelativeForce(f);
  link_->AddTorque(tau);
}
#endif






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


GZ_REGISTER_MODEL_PLUGIN(UavGazeboPlugin)

} // end of namespace
