#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
using std::placeholders::_1;

// Subscriber class for subscribing to the pose topic
class Subscriber : public rclcpp::Node {
public:
  Subscriber() : Node("aruco_robot_controller") {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "aruco_single/pose", 10, std::bind(&Subscriber::topic_callback, this, _1));
  }

  geometry_msgs::msg::Pose getPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return last_pose_;
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped &msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    last_pose_ = msg.pose;
    RCLCPP_INFO(this->get_logger(), "Pose updated: x=%f, y=%f, z=%f",
                last_pose_.position.x, last_pose_.position.y, last_pose_.position.z);
  }

  mutable std::mutex pose_mutex_;
  geometry_msgs::msg::Pose last_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create a subscriber node and spin it in a separate thread
  auto subscriber_node = std::make_shared<Subscriber>();
  std::thread subscriber_thread([&]() { rclcpp::spin(subscriber_node); });

  // Create the MoveIt node
  auto const node = std::make_shared<rclcpp::Node>(
    "aruco_robot_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("aruco_robot_controller");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Create the Planning Scene Interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Retrieve the pose from the subscriber
  geometry_msgs::msg::Pose target_pose = subscriber_node->getPose();
  auto x = move_group_interface.getPlanningFrame();
  RCLCPP_INFO(logger, "ABC: %s", x.c_str());
  // Use position-only IK by setting the target position
  move_group_interface.setPositionTarget(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z
  );
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    RCLCPP_INFO(logger, "Planning successful! Executing the plan...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown subscriber thread and ROS
  rclcpp::shutdown();
  subscriber_thread.join();

  return 0;
}
