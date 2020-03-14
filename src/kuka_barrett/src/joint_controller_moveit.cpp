#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_controller_moveit");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

	// The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools.loadRemoteControl();

	// Setup Move group
  static const std::string PLANNING_GROUP = "iiwa_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group.setGoalPositionTolerance(0.05);
	move_group.setGoalOrientationTolerance(0.05);
	move_group.setPlanningTime(10);

  tf2::Quaternion quaternion;
	quaternion.setRPY( M_PI, 0, M_PI_2 );

	// TARGET POSE 1
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = quaternion.getW();
	target_pose1.orientation.x = quaternion.getX();
	target_pose1.orientation.y = quaternion.getY();
	target_pose1.orientation.z = quaternion.getZ();
  target_pose1.position.x = 0;
  target_pose1.position.y = -0.68;
  target_pose1.position.z = 1.5;
	move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 	move_group.plan(my_plan);

	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("iiwa_palnning", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
	std::string nextButtonMsg = "Press 'next' in the RvizVisualToolsGui window to execute plan";
	visual_tools.prompt(nextButtonMsg);
  move_group.move();

  // TARGET POSE 2
  // same as previous but lower on the z-axis (approaching tool)
	geometry_msgs::Pose target_pose2;
	target_pose2 = target_pose1;
	target_pose2.position.z = 1.25;
	move_group.setPoseTarget(target_pose2);

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("iiwa_palnning", "Visualizing plan 2 (pose goal) %s", success ? "SUCCESS" : "FAILED");
	visual_tools.prompt(nextButtonMsg);
	move_group.move();

  ros::shutdown();
  return 0;
}
