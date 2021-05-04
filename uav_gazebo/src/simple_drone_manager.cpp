#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <uav_gazebo_msgs/PositionYawControl.h>
#include <uav_gazebo_msgs/ControlMode.h>
#include <uav_gazebo_msgs/GetMode.h>
#include <uav_gazebo_msgs/SwitchMode.h>
#include <uav_gazebo_msgs/Takeoff.h>
#include <uav_gazebo_msgs/Land.h>


double norm(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}


template<class T1, class T2>
double distance(const T1& a, const T2& b) {
  return norm(
    a.x - b.x,
    a.y - b.y,
    a.z - b.z
  );
}


class SimpleDroneManager {
public:
  SimpleDroneManager(ros::NodeHandle& nh);
private:
  ros::ServiceServer takeoff_srv_;
  ros::ServiceServer land_srv_;
  ros::ServiceClient get_mode_srv_;
  ros::ServiceClient switch_mode_srv_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pose_pub_;

  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose takeoff_pose_;

  void odometryCB(const nav_msgs::Odometry& msg);

  bool takeoff(
    uav_gazebo_msgs::Takeoff::Request& request,
    uav_gazebo_msgs::Takeoff::Response& response
  );

  bool land(
    uav_gazebo_msgs::Land::Request& request,
    uav_gazebo_msgs::Land::Response& response
  );
};


SimpleDroneManager::SimpleDroneManager(ros::NodeHandle& nh) {
  takeoff_srv_ = nh.advertiseService("takeoff", &SimpleDroneManager::takeoff, this);
  land_srv_ = nh.advertiseService("land", &SimpleDroneManager::land, this);
  get_mode_srv_ = nh.serviceClient<uav_gazebo_msgs::GetMode>("get_mode");
  switch_mode_srv_ = nh.serviceClient<uav_gazebo_msgs::SwitchMode>("switch_mode");
  odom_sub_ = nh.subscribe("odometry", 1, &SimpleDroneManager::odometryCB, this);
  cmd_pose_pub_ = nh.advertise<uav_gazebo_msgs::PositionYawControl>("position_yaw/command", 1);
  ROS_INFO_STREAM("Service " << takeoff_srv_ .getService() << " ready");
}


void SimpleDroneManager::odometryCB(const nav_msgs::Odometry& msg) {
  current_pose_ = msg.pose.pose;
}


bool SimpleDroneManager::takeoff(
  uav_gazebo_msgs::Takeoff::Request& request,
  uav_gazebo_msgs::Takeoff::Response& response
)
{
  ROS_INFO_STREAM("Takeoff request received:" << std::endl <<
    " - height: " << request.height << std::endl <<
    " - takeoff time: " << request.takeoff_time << std::endl <<
    " - idle time: " << request.idle_time);
  // store the initial position
  takeoff_pose_ = current_pose_;
  // target position to be reached
  auto target_position = takeoff_pose_.position;
  target_position.z += request.height;
  // pre-fill the response in case of failures
  response.residual = distance(current_pose_.position, target_position);
  response.success = false;
  // get current mode
  uav_gazebo_msgs::GetMode current_mode;
  if(!get_mode_srv_.call(current_mode)) {
    ROS_ERROR_STREAM("Failed to call service '" <<
      get_mode_srv_.getService() << "'");
    return false;
  }
  // check if this is the correct mode
  if(current_mode.response.mode.mode != uav_gazebo_msgs::ControlMode::INACTIVE) {
    if(request.force_takeoff) {
      // not in correct mode, but let's force a switch anyway
      ROS_WARN_STREAM("Forcing drone into mode " <<
        uav_gazebo_msgs::ControlMode::POSITION_YAW << " even if it is not "
        "currently inactive");
    }
    else {
      ROS_ERROR_STREAM("Drone is not in mode " <<
        uav_gazebo_msgs::ControlMode::INACTIVE << " (INACTIVE). It is in mode "
        << current_mode.response.mode.mode << " instead.");
      return true;
    }
  }
  // switch to POSITION_YAW mode
  uav_gazebo_msgs::SwitchMode switch_mode;
  switch_mode.request.mode.mode = uav_gazebo_msgs::ControlMode::POSITION_YAW;
  if(!switch_mode_srv_.call(switch_mode)) {
    ROS_ERROR_STREAM("Failed to call service '" <<
      switch_mode_srv_.getService() << "'");
    return false;
  }
  else if(!switch_mode.response.success) {
    ROS_ERROR_STREAM("Service '" << switch_mode_srv_.getService() << "' "
      "reported failure");
    return true;
  }

  // go to the takeoff position
  uav_gazebo_msgs::PositionYawControl command;
  command.position.x = takeoff_pose_.position.x;
  command.position.y = takeoff_pose_.position.y;
  command.position.z = takeoff_pose_.position.z;
  command.yaw = 2*std::asin(takeoff_pose_.orientation.z);
  auto takeoff_time = request.takeoff_time > 0 ? request.takeoff_time : 4*std::max(1.0, std::fabs(request.height));
  command.velocity.z = request.height / takeoff_time;

  // timing variables
  ros::Rate rate(50);
  auto start = ros::Time::now();

  // main loop: change the setpoint
  ROS_INFO("Taking off");
  while(ros::ok()) {
    ros::spinOnce();
    auto rho = (ros::Time::now() - start).toSec() / takeoff_time;
    if(rho > 0.9) {
      command.velocity.z = 0;
      command.position.z = target_position.z;
    }
    else {
      command.position.z = takeoff_pose_.position.z + rho * request.height;
    }
    cmd_pose_pub_.publish(command);
    rate.sleep();
    if(rho > 1)
      break;
  }

  // wait for the quadrotor to stabilize
  ROS_INFO("Hovering");
  constexpr int NUM_SPINS = 5;
  for(unsigned int i=0; i<NUM_SPINS; i++) {
    ros::Duration(request.idle_time/NUM_SPINS).sleep();
    ros::spinOnce();
  }

  // takeoff completed!
  response.residual = distance(current_pose_.position, target_position);
  response.success = true;
  ROS_INFO_STREAM("Takeoff completed. Residual: " << response.residual);
  return true;
}


bool SimpleDroneManager::land(
  uav_gazebo_msgs::Land::Request& request,
  uav_gazebo_msgs::Land::Response& response
)
{
  ROS_INFO_STREAM("Land request received:" << std::endl <<
    " - land time: " << request.land_time << std::endl <<
    " - idle time: " << request.idle_time << std::endl <<
    " - maximum residual: " << request.maximum_residual);
  // store the current initial position
  auto initial_pose = current_pose_;
  // pre-fill the response in case of failures
  response.landed_forcefully = false;
  response.success = false;
  // switch to POSITION_YAW mode
  uav_gazebo_msgs::SwitchMode switch_mode;
  switch_mode.request.mode.mode = uav_gazebo_msgs::ControlMode::POSITION_YAW;
  if(!switch_mode_srv_.call(switch_mode)) {
    ROS_ERROR_STREAM("Failed to call service '" <<
      switch_mode_srv_.getService() << "'");
    return false;
  }
  else if(!switch_mode.response.success) {
    ROS_ERROR_STREAM("Service '" << switch_mode_srv_.getService() << "' "
      "reported failure");
    return true;
  }

  // go to the landing position
  uav_gazebo_msgs::PositionYawControl command;
  command.position.x = initial_pose.position.x;
  command.position.y = initial_pose.position.y;
  command.position.z = initial_pose.position.z;
  command.yaw = 2*std::asin(initial_pose.orientation.z);
  auto land_time = request.land_time > 0 ? request.land_time : 4*std::max(1.0, std::fabs(initial_pose.position.z-takeoff_pose_.position.z));
  command.velocity.z = - (initial_pose.position.z - takeoff_pose_.position.z) / land_time;

  // timing variables
  ros::Rate rate(50);
  auto start = ros::Time::now();

  // slowly descend towards the goal
  ROS_INFO("Descending");
  while(ros::ok()) {
    ros::spinOnce();
    auto rho = (ros::Time::now() - start).toSec() / land_time;
    if(rho > 0.9) {
      command.velocity.z = 0;
      command.position.z = takeoff_pose_.position.z;
    }
    else {
      command.position.z = rho * takeoff_pose_.position.z + (1-rho) * initial_pose.position.z;
    }
    cmd_pose_pub_.publish(command);
    rate.sleep();
    if(rho > 1)
      break;
  }

  // wait for the quadrotor to stabilize
  ROS_INFO("Hovering");
  constexpr int NUM_SPINS = 5;
  for(unsigned int i=0; i<NUM_SPINS; i++) {
    ros::Duration(request.idle_time/NUM_SPINS).sleep();
    ros::spinOnce();
  }

  // takeoff completed!
  auto residual = distance(current_pose_.position, command.position);
  if(residual > request.maximum_residual) {
    if(request.force_landing) {
      // did not reach the expected position, but let's force a switch anyway
      ROS_WARN_STREAM("Forcing landing even if the expected position has not "
        "been reached (residual: " << residual << ")");
      response.landed_forcefully = true;
    }
    else {
      ROS_ERROR_STREAM("Drone did not reach the expected position. Residual: "
        << residual);
      return true;
    }
  }

  // switch off the drone
  switch_mode.request.mode.mode = uav_gazebo_msgs::ControlMode::INACTIVE;
  if(!switch_mode_srv_.call(switch_mode)) {
    ROS_ERROR_STREAM("Failed to call service '" <<
      switch_mode_srv_.getService() << "'");
    return false;
  }
  else if(!switch_mode.response.success) {
    ROS_ERROR_STREAM("Service '" << switch_mode_srv_.getService() << "' "
      "reported failure");
    return true;
  }

  response.success = true;
  ROS_INFO_STREAM("Landing completed.");
  return true;
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_drone_manager");
  ros::NodeHandle pnh("~");
  std::string drone_namespace;
  pnh.param<std::string>("drone_namespace", drone_namespace, "drone");
  ros::NodeHandle drone_nh(drone_namespace);
  SimpleDroneManager manager(drone_nh);
  ros::Duration(5).sleep();
  ros::spin();
  return EXIT_SUCCESS;
}
