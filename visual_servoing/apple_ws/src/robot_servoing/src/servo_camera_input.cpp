/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_keyboard_input.cpp
 *      Project   : moveit_servo
 *      Created   : 05/31/2021
 *      Author    : Adam Pettinger, V Mohammed Ibrahim
 */

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Some constants used in the Servo Teleop demo
namespace
{
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string POSE_TOPIC = "/aruco_single/pose";
const size_t ROS_QUEUE_SIZE = 10;
const std::string PLANNING_FRAME_ID = "base_link";
const std::string EE_FRAME_ID = "tool0";
}  // namespace


using std::placeholders::_1;

// Converts received object locations to Twist commands for Servo
class VisualServo : public rclcpp::Node
{
public:
  VisualServo();

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::string command_frame_id_;
};

VisualServo::VisualServo() : Node("servo_camera_input"), command_frame_id_{ "tool0" }
{
  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(POSE_TOPIC, 10, std::bind(&VisualServo::pose_callback, this, _1));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualServo>());
  rclcpp::shutdown();
  return 0;
}

void VisualServo::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{
  float dead_zone = 0.05;
  float max_twist = 1.0;
  float P = 10;
  float target_x = 0.0;
  float target_y = 0.0;

  // calculate error
  float error_x = msg->pose.position.x - target_x;
  float error_y = msg->pose.position.y - target_y;

  // create twist proportional to measured error
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  twist_msg->twist.linear.x = P*error_y * -1.0;
  twist_msg->twist.linear.y = P*error_x;
  bool publish_twist = true;

  // if (msg->pose.position.y > 1.0 * dead_zone)
  // {
  //   twist_msg->twist.linear.x = -0.5;
  //   publish_twist = true;
  // }
  // if (msg->pose.position.y < -1.0 * dead_zone)
  // {
  //   twist_msg->twist.linear.x = 0.5;
  //   publish_twist = true;
  // }
  // if (msg->pose.position.x > 1.0 * dead_zone)
  // {
  //   twist_msg->twist.linear.y = 0.5;
  //   publish_twist = true;
  // }
  // if (msg->pose.position.x < 1.0 * dead_zone)
  // {
  //   twist_msg->twist.linear.y = -0.5;
  //   publish_twist = true;
  // }

  if (publish_twist)
  {
    twist_msg->header.stamp = this->now();
    twist_msg->header.frame_id = command_frame_id_;
    twist_pub_->publish(std::move(twist_msg));
  }
}