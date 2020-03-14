#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "iiwa_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  tf2::Quaternion quaternion;
	quaternion.setRPY( M_PI_2, 0, 0 );

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = quaternion.getW();
	target_pose1.orientation.x = quaternion.getX();
	target_pose1.orientation.y = quaternion.getY();
	target_pose1.orientation.z = quaternion.getZ();
  target_pose1.position.x = 0;
  target_pose1.position.y = -0.8;
  target_pose1.position.z = 1.2;
  move_group.setGoalPositionTolerance(0.05);
  move_group.setGoalOrientationTolerance(0.05);
  move_group.setPoseTarget(target_pose1);
	move_group.setPlanningTime(10);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 	move_group.plan(my_plan);

  move_group.move();

  ros::shutdown();
  return 0;
}
