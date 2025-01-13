#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// Function to add a ground plane to the planning scene
void addGroundPlane(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
  moveit_msgs::msg::CollisionObject ground_plane;
  ground_plane.id = "ground_plane";
  ground_plane.header.frame_id = "world";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {10.0, 10.0, 0.01}; // Length, width, height

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.z = -0.005; 

  ground_plane.primitives.push_back(box);
  ground_plane.primitive_poses.push_back(box_pose);
  ground_plane.operation = ground_plane.ADD;

  planning_scene_interface.applyCollisionObject(ground_plane);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "move_arm",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_arm");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Create the Planning Scene Interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Add the ground plane to the planning scene
  addGroundPlane(planning_scene_interface);

  // Set a target Position Only (Ignore Orientation)
  auto const target_position = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0;
    msg.position.y = 0.5;
    msg.position.z = 0.7;
    return msg;
  }();

  // Use position-only IK by setting the target position
  move_group_interface.setPositionTarget(
    target_position.position.x,
    target_position.position.y,
    target_position.position.z
  );

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
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

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
