#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>

using namespace std;


geometry_msgs::Pose getPoseFromPathPoint(vector<float> pathPoint) {
	geometry_msgs::Pose target_pose;
	tf2::Quaternion quaternion;

	target_pose.position.x = pathPoint[0];
	target_pose.position.y = pathPoint[1];
	target_pose.position.z = pathPoint[2];

	quaternion.setRPY(pathPoint[3], pathPoint[4], pathPoint[5]);
	target_pose.orientation.w = quaternion.getW();
	target_pose.orientation.x = quaternion.getX();
	target_pose.orientation.y = quaternion.getY();
	target_pose.orientation.z = quaternion.getZ();

	return target_pose;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_planner1");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

	// Setup Move group
  static const std::string PLANNING_GROUP = "iiwa_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group.setGoalPositionTolerance(0.000005);
	move_group.setGoalOrientationTolerance(0.000005);
	move_group.setPlanningTime(5);
	move_group.allowReplanning(true);
	move_group.setNumPlanningAttempts(6);

	// The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("world");
	visual_tools.deleteAllMarkers();
	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools.loadRemoteControl();
	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools.trigger();

	// Raw pointers are frequently used to refer to the planning group for improved performance.
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	vector<vector<float>> path;

	// X Y Z Roll Pitch Yaw
	// path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a signularity and cant be calculated from the numerical IK
	path.push_back({0, 0, 2.26, 0, 0, 0}); // Home position

	// Pick position 1
	path.push_back({0, -0.68, 1.5, M_PI, 0, -M_PI_2});
	path.push_back({0, -0.68, 1.30, M_PI, 0, -M_PI_2});
	path.push_back({0, -0.68, 1.5, M_PI, 0, -M_PI_2});

	// TCP position for point above fulcrum 1
	path.push_back({-0.320971, 0.681543, 1.656761, 0.019169, 0.783348, 0.073975});

	geometry_msgs::Pose target_pose;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	for (int i = 0; i < path.size(); ++i) {
		// Arm Kinematics (KUKA iiwa)
		vector<float> pathPoint = path.at(i);

		target_pose = getPoseFromPathPoint(pathPoint);

		move_group.setPoseTarget(target_pose);

		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("robot_planner1", "Visualizing plan %d (pose goal) %s", i, success ? "SUCCESS" : "FAILED");
		// Visualize plan
		ROS_INFO_NAMED("robot_planner1", "Visualizing plan %d as trajectory line", i);
		visual_tools.publishAxisLabeled(target_pose, "pose");
		visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();

		move_group.execute(my_plan);
	}


	// Approaching Fulcrum point - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose start_pose = getPoseFromPathPoint(path.at(path.size()-1)); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(start_pose);

	vector<float> insertedPoseAtFulcrum = {-0.113231, 0.681543, 1.449251, 0.019169, 0.783348, 0.073975};
	path.push_back(insertedPoseAtFulcrum);
	target_pose = getPoseFromPathPoint(insertedPoseAtFulcrum);
	waypoints.push_back(target_pose);

	// The approaching motion needs to be slower.
	// We reduce the speed of the robot arm via a scaling factor
	// of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
	move_group.setMaxVelocityScalingFactor(0.5);

	// We want the Cartesian path to be interpolated at a resolution of 1 mm
	// which is why we will specify 0.01 as the max step in Cartesian
	// translation.  We will specify the jump threshold as 0.0, effectively disabling it.
	// WARNING - disabling the jump threshold while operating real hardware can cause
	// large unpredictable motions of redundant joints and could be a safety issue
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.001;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("robot_planner1", "Visualizing plan for insertion movement (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

	// Visualize the plan in RViz
	visual_tools.deleteAllMarkers();
	visual_tools.publishText(target_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	for (std::size_t i = 0; i < waypoints.size(); ++i)
		visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
	visual_tools.trigger();

	moveit::planning_interface::MoveGroupInterface::Plan insertion_plan;
	bool success = (move_group.plan(insertion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (success) {
		insertion_plan.trajectory_ = trajectory;
		// move_group.execute(insertion_plan);
	}
	ROS_INFO_NAMED("robot_planner1", "Executing insertion motion plan %s", success ? "SUCCESS" : "FAILED");


	// Reverse insertion movement - Remove tool from trocar
	std::vector<geometry_msgs::Pose> reverse_waypoints;
	reverse_waypoints.push_back(target_pose);
	reverse_waypoints.push_back(start_pose);
	moveit_msgs::RobotTrajectory reverse_trajectory;
	fraction = move_group.computeCartesianPath(reverse_waypoints, eef_step, jump_threshold, reverse_trajectory);
	ROS_INFO_NAMED("robot_planner1", "Visualizing plan for reverse insertion movement (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
	moveit::planning_interface::MoveGroupInterface::Plan reverse_insertion_plan;
	success = (move_group.plan(reverse_insertion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	reverse_insertion_plan.trajectory_ = reverse_trajectory;
	move_group.execute(reverse_insertion_plan);
	ROS_INFO_NAMED("robot_planner1", "Executing reverse insertion motion plan %s", success ? "SUCCESS" : "FAILED");

  ros::shutdown();
  return 0;
}


