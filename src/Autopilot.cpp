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
  // ros::Rate rate(1);
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
  if(status == DroneStatus::Flying || status == DroneStatus::Flying2
     || status == DroneStatus::Hovering)//state 3,4 or 7
  {
    return move(forward, left, up, rotateLeft);
  }
  return false;
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up,
                     double rotateLeft)
{
  // TODO: implement...
  geometry_msgs::Twist navMsg;
  navMsg.linear.x = forward;
  navMsg.linear.y = left;
  navMsg.linear.z = up;
  navMsg.angular.z = rotateLeft;
  pubMove_.publish(navMsg);
  return true;
}

}  // namespace arp