/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);
  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Publisher camera pose cmd
  pubPose_ = nh_->advertise<geometry_msgs::PoseStamped>("ardrone/camera_pose", 1);

  pose_msg_sequence = 0;

  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  DroneStatus status = droneStatus();
  if (status != 4 && status != 3 && status != 7) {
    return false;
  }

  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up,
                           double rotateLeft)
{
  // ARdrone -> move
  geometry_msgs::Twist moveMsg;
  moveMsg.linear.x = forward; // updownarrowkeys
  moveMsg.linear.y = left; // leftrightarrowkeys
  moveMsg.linear.z = up; // WSupandodwn
  moveMsg.angular.z = rotateLeft; //AD left right YAW
  pubMove_.publish(moveMsg);

  return true;
}
// Move the drone manually.
void Autopilot::publishTag(const Frontend::Detection & detection){
    geometry_msgs::PoseStamped poseMsg; // quaternions float64
    std_msgs::Header header;
    geometry_msgs::Pose pose;

    kinematics::Transformation T_TC = detection.T_CT.inverse();
    Eigen::Quaterniond q_quat = T_TC.q();
    Eigen::Vector3d r_point = T_TC.r();

    header.frame_id = "target";
    header.stamp.sec = (int) time(NULL);
    header.seq = pose_msg_sequence; pose_msg_sequence ++;
    poseMsg.header = header;

    geometry_msgs::Point point;

    point.x = r_point[0];
    point.y = r_point[1];
    point.z = r_point[2];

    geometry_msgs::Quaternion quat;

    quat.x = q_quat.x();
    quat.y = q_quat.y();
    quat.z = q_quat.z();
    quat.w = q_quat.w();

    pose.position = point;
    pose.orientation = quat;
    poseMsg.pose = pose;

    pubPose_.publish(poseMsg);

  }




}  // namespace arp
