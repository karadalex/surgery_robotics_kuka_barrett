#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>

using namespace std;

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
	move_group.setGoalPositionTolerance(0.00005);
	move_group.setGoalOrientationTolerance(0.00005);
//	move_group.setGoalPositionTolerance(0.5);
//	move_group.setGoalOrientationTolerance(0.5);
	move_group.setPlanningTime(10);
	move_group.allowReplanning(true);

	vector<vector<float>> path;
	// X Y Z Roll Pitch Yaw
	// Pick position 1
	path.push_back({0, -0.68, 1.5, M_PI, 0, M_PI_2});
	path.push_back({0, -0.68, 1.19, M_PI, 0, M_PI_2});
	path.push_back({0, -0.68, 1.5, M_PI, 0, M_PI_2});
	// Fulcrum position 1
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});
	path.push_back({0.06, 0.70, 1.38, M_PI, -0.59, 0});
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});
	// Pick position 2
	path.push_back({0.2, -0.68, 1.5, M_PI, 0, M_PI_2});
	path.push_back({0.2, -0.68, 1.19, M_PI, 0, M_PI_2});
	path.push_back({0.2, -0.68, 1.5, M_PI, 0, M_PI_2});
	// Fulcrum position 2
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});
	path.push_back({0.06, 0.70, 1.38, M_PI, -0.59, 0});
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});

	geometry_msgs::Pose target_pose;
	tf2::Quaternion quaternion;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	for (int i = 0; i < path.size(); ++i) {
		vector<float> pose = path.at(i);
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];
		quaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.w = quaternion.getW();
		target_pose.orientation.x = quaternion.getX();
		target_pose.orientation.y = quaternion.getY();
		target_pose.orientation.z = quaternion.getZ();
		move_group.setPoseTarget(target_pose);
		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("iiwa_planning", "Visualizing plan %d (pose goal) %s", i, success ? "SUCCESS" : "FAILED");
		std::string nextButtonMsg = "Press 'next' in the RvizVisualToolsGui window to execute plan";
		// visual_tools.prompt(nextButtonMsg);
		move_group.move();
	}


  ros::shutdown();
  return 0;
}
