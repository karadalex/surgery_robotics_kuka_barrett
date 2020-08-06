#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_planner1");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

	// Setup Move group
  static const std::string PLANNING_GROUP = "iiwa_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	// move_group.setGoalPositionTolerance(0.000005);
	// move_group.setGoalOrientationTolerance(0.000005);
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

	// Fulcrum Frame 1 pose
	geometry_msgs::Pose fulcrum1_pose;
	fulcrum1_pose.position.x = 0.126597;
	fulcrum1_pose.position.y = 0.700751;
	fulcrum1_pose.position.z = 1.293635;
	tf2::Quaternion fulcrum1_rpy_quaternion;
	fulcrum1_rpy_quaternion.setRPY(0.0, -0.591161, 0.0);
	fulcrum1_pose.orientation.w = fulcrum1_rpy_quaternion.getW();
	fulcrum1_pose.orientation.x = fulcrum1_rpy_quaternion.getX();
	fulcrum1_pose.orientation.y = fulcrum1_rpy_quaternion.getY();
	fulcrum1_pose.orientation.z = fulcrum1_rpy_quaternion.getZ();
	// Publish Fulcrum 1 Reference Frame in RViz
	visual_tools.publishAxisLabeled(fulcrum1_pose, "Fulcrum Frame 1", rvt::MEDIUM);
	// Publish fulcrum pose in Fulcrum ROS Topic
	// TODO

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

	// TCP position for fulcrum 1
	// path.push_back({-0.349826, 0.673821, 1.572205, 0.018372, 0.735106, 0.072819});
	path.push_back({-0.320971, 0.681543, 1.656761, 0.019169, 0.783348, 0.073975});

	geometry_msgs::Pose target_pose;
	tf2::Quaternion quaternion;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	for (int i = 0; i < path.size(); ++i) {
		// Arm Kinematics (KUKA iiwa)
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
	geometry_msgs::Pose start_pose = target_pose;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(start_pose);

	// vector<float> poseValues = {-0.133848, 0.684161, 1.376295, 0.018485, 0.735344, 0.072922};
	// vector<float> poseValues = {-0.192705, 0.684161, 1.429469, 0.018485, 0.735344, 0.072922};
	vector<float> poseValues = {-0.195960, 0.681543, 1.532168, 0.019169, 0.783348, 0.073975};
	path.push_back(poseValues);
	target_pose.position.x = poseValues[0];
	target_pose.position.y = poseValues[1];
	target_pose.position.z = poseValues[2];
	quaternion.setRPY(poseValues[3], poseValues[4], poseValues[5]);
	target_pose.orientation.w = quaternion.getW();
	target_pose.orientation.x = quaternion.getX();
	target_pose.orientation.y = quaternion.getY();
	target_pose.orientation.z = quaternion.getZ();
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
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
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

	// std::string nextButtonMsg = "Press 'next' in the RvizVisualToolsGui window to continue with pivoting motion";
	// visual_tools.prompt(nextButtonMsg);

	// Pivoting Trajectory
	// std::vector<geometry_msgs::Pose> pivot_traj1_waypts;
	// vector<vector<float>> pivot_path1;
	// pivot_path1.push_back({-0.185241, 0.756977, 1.419682, -0.176089, 0.717969, -0.218691});
	// pivot_path1.push_back({-0.232891, 0.750714, 1.336278, -0.145829, 0.435512, -0.164146});
	// pivot_path1.push_back({-0.201695, 0.603255, 1.333467, 0.063977, 0.453583, 0.315894});
	// for (int i = 0; i < pivot_path1.size(); ++i) {
	// 	vector<float> pose = path.at(i);
	// 	geometry_msgs::Pose pivot_pose;
	// 	pivot_pose.position.x = pose[0];
	// 	pivot_pose.position.y = pose[1];
	// 	pivot_pose.position.z = pose[2];
	// 	quaternion.setRPY(pose[3], pose[4], pose[5]);
	// 	pivot_pose.orientation.w = quaternion.getW();
	// 	pivot_pose.orientation.x = quaternion.getX();
	// 	pivot_pose.orientation.y = quaternion.getY();
	// 	pivot_pose.orientation.z = quaternion.getZ();
	// 	pivot_traj1_waypts.push_back(pivot_pose);
	// }
	// // reverse_waypoints.push_back(start_pose);
	// moveit_msgs::RobotTrajectory pivot_traj1;
	// fraction = move_group.computeCartesianPath(pivot_traj1_waypts, eef_step, jump_threshold, pivot_traj1);
	// ROS_INFO_NAMED("robot_planner1", "Pivoting motion trajectory (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
	//
	// visual_tools.publishText(text_pose, "Pivot motion", rvt::WHITE, rvt::XLARGE);
	// visual_tools.publishPath(pivot_traj1_waypts, rvt::LIME_GREEN, rvt::SMALL);
	// for (std::size_t i = 0; i < waypoints.size(); ++i)
	// 	visual_tools.publishAxisLabeled(pivot_traj1_waypts[i], "pt" + std::to_string(i), rvt::SMALL);
	// visual_tools.trigger();
	//
	// moveit::planning_interface::MoveGroupInterface::Plan pivot_plan;
	// success = (move_group.plan(pivot_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	// if (success) {
	// 	pivot_plan.trajectory_ = pivot_traj1;
	// 	move_group.execute(pivot_plan);
	// }
	// ROS_INFO_NAMED("robot_planner1", "Executing Pivoting motion motion plan %s", success ? "SUCCESS" : "FAILED");


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


