/*
 * Autopilot.hpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#ifndef ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_
#define ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_

#include <mutex>
#include <Eigen/Core>
#include <atomic>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

namespace arp {

/// \brief The autopilot highlevel interface for commanding the drone manually or automatically.
class Autopilot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Autopilot(ros::NodeHandle& nh);

  /// \brief These are reverse engineered AR Drone states.
  enum DroneStatus {
    Unknown = 0,
    Inited = 1,
    Landed = 2,
    Flying = 3,
    Hovering = 4,
    Test = 5, // ?
    TakingOff = 6,
    Flying2 = 7,
    Landing = 8,
    Looping = 9 // ?
  };

  /// \brief Get the drone status.
  /// \return The status.
  DroneStatus droneStatus();

  /// \brief Set to automatic control mode.
  void setManual();

  /// \brief Set to manual control mode.
  void setAutomatic();

  /// \brief Request flattrim calibration.
  /// \return True on success.
  /// \note This will only work when landed on ground.
  bool flattrimCalibrate();

  /// \brief Takeoff.
  /// \return True on success.
  /// \note This will only work when landed on ground.
  bool takeoff();

  /// \brief Land.
  /// \return True on success.
  /// \note This will only work when in flight.
  bool land();

  /// \brief Turn off all motors and reboot.
  /// \return True on success.
  /// \warning When in flight, this will let the drone drop down to the ground.
  bool estopReset();

  /// \brief Move the drone manually.
  /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
  /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
  /// \return True on success.
  /// \note This will only do something when in manual mode and flying.
  bool manualMove(double forward, double left, double up, double rotateLeft);

 protected:
  /// \brief Move the drone.
  /// @param[in] forward Forward tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] left Left tilt [-1,...,1] scaling the maximum tilt ROS parameter.
  /// @param[in] up Upward velocity [-1,...,1] scaling the maximum vertical speed ROS parameter.
  /// @param[in] rotateLeft Left yaw velocity [-1,...,1] scaling the maximum yaw rate ROS parameter.
  /// \return True on success.
  bool move(double forward, double left, double up, double rotateLeft);

  /// \brief Obtain the last navdata package (callback).
  /// @param[out] navdata The navdata message.
  void navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg);

  ros::NodeHandle * nh_;  ///< ROS node handle.
  ros::Publisher pubReset_;  ///< The reset publisher -- use to reset the drone (e-stop).
  ros::Publisher pubTakeoff_;  ///< Publish to take-off the drone.
  ros::Publisher pubLand_;  ///< Publish to land the drone.
  ros::Publisher pubNav_; 
  ros::ServiceClient srvFlattrim_;  ///< To request a flat trim calibration.
  ardrone_autonomy::Navdata lastNavdata_; ///< Store navdata as it comes in asynchronously.
  std::mutex navdataMutex_; ///< We need to lock navdata access due to asynchronous arrival.
  ros::Subscriber subNavdata_; ///< The subscriber for navdata.
};

} // namespace arp



#endif /* ARDRONE_PRACTICALS_INCLUDE_ARP_AUTOPILOT_HPP_ */
