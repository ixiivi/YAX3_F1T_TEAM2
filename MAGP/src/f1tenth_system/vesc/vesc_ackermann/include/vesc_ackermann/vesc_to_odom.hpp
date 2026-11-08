// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN__VESC_TO_ODOM_HPP_
#define VESC_ACKERMANN__VESC_TO_ODOM_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class VescToOdom : public rclcpp::Node
{
public:
  explicit VescToOdom(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double wheelbase_;
  bool publish_tf_;

  // odometry state
  double x_, y_, yaw_;
  /** Yaw rate from the IMU gyro instead of the servo command (tyre slip and servo lag make the
   *  servo estimate turn too much above about 2 m/s) */
  bool use_imu_yaw_;
  double imu_gyro_scale_;   ///< gyro unit to rad/s (the VESC driver reports deg/s)
  double imu_gyro_bias_;    ///< in gyro units; re-estimated whenever the car stands still
  double imu_bias_alpha_;   ///< low-pass weight of one standing-still sample in the bias
  double imu_timeout_;      ///< [s] older gyro data falls back to the servo estimate
  double last_gyro_;
  rclcpp::Time last_gyro_time_;
  bool have_gyro_;
  Float64::SharedPtr last_servo_cmd_;  ///< Last servo position commanded value
  VescStateStamped::SharedPtr last_state_;  ///< Last received state message

  // ROS services
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<Float64>::SharedPtr servo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // ROS callbacks
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void servoCmdCallback(const Float64::SharedPtr servo);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VESC_TO_ODOM_HPP_
