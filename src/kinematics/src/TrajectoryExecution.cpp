//
// Created by karadalex on 9/8/20.
//

#include "kinematics/TrajectoryExecution.h"

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


TrajectoryExecution::TrajectoryExecution(const string PLANNING_GROUP, double pos_tolerance, double orient_tolerance,
																				 int plan_time_sec, bool replanning, int plan_attempts,
																				 const string base_frame, const string plannerId) {
	// Setup Move group
	move_group = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
	move_group.setGoalPositionTolerance(pos_tolerance);
	move_group.setGoalOrientationTolerance(orient_tolerance);
	move_group.setPlanningTime(plan_time_sec);
	move_group.allowReplanning(replanning);
	move_group.setNumPlanningAttempts(plan_attempts);
	move_group.setPlannerId(plannerId);

	// Try to set Elbow up constraint to avoid collision with trocar mounting dock
	// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial.rst
	// https://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#planning-with-path-constraints
	// moveit_msgs::OrientationConstraint ocm;
	// ocm.link_name = "iiwa_link_3";
	// ocm.header.frame_id = "world";
	// ocm.orientation.w = 1.0;
	// ocm.absolute_x_axis_tolerance = M_PI_2;
	// ocm.absolute_y_axis_tolerance = M_PI_2;
	// ocm.absolute_z_axis_tolerance = M_PI_2;
	// ocm.weight = 1.0;
	// moveit_msgs::Constraints test_constraints;
	// test_constraints.orientation_constraints.push_back(ocm);
	// move_group.setPathConstraints(test_constraints);

	// The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
	visual_tools = moveit_visual_tools::MoveItVisualTools(base_frame);
	visual_tools.deleteAllMarkers();
	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools.loadRemoteControl();
	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.75;
	namespace rvt = rviz_visual_tools;
	visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);
	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools.trigger();

	// Raw pointers are frequently used to refer to the planning group for improved performance.
	joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
}


void TrajectoryExecution::executePath(vector<vector<float>> path, const char* traj_name) {
	namespace rvt = rviz_visual_tools;

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
		ROS_INFO_NAMED("robot_planner1", "Planning time for path %s was %.6f seconds", traj_name, my_plan.planning_time_);
		ROS_INFO_NAMED("robot_planner1", "Executing %s plan %s", traj_name, success ? "SUCCESS" : "FAILED");
	}
}


void TrajectoryExecution::executePath(vector<geometry_msgs::Pose> path, const char* traj_name) {
	namespace rvt = rviz_visual_tools;

	geometry_msgs::Pose target_pose;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	for (int i = 0; i < path.size(); ++i) {
		target_pose = path.at(i);

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

		ROS_INFO_NAMED("robot_planner1", "Planning time for path %s was %.6f seconds", traj_name, my_plan.planning_time_);
		ROS_INFO_NAMED("robot_planner1", "Executing %s plan %s", traj_name, success ? "SUCCESS" : "FAILED");
	}
}

void TrajectoryExecution::moveToTarget(geometry_msgs::Pose target, const char* traj_name) {
	namespace rvt = rviz_visual_tools;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	move_group.setPoseTarget(target);

	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("robot_planner1", "Visualizing plan %s (pose goal) %s", traj_name, success ? "SUCCESS" : "FAILED");
	// Visualize plan
	ROS_INFO_NAMED("robot_planner1", "Visualizing plan %s as trajectory line", traj_name);
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();

	move_group.execute(my_plan);

	ROS_INFO_NAMED("robot_planner1", "Planning time for path %s was %.6f seconds", traj_name, my_plan.planning_time_);
	ROS_INFO_NAMED("robot_planner1", "Executing %s plan %s", traj_name, success ? "SUCCESS" : "FAILED");
}


void TrajectoryExecution::executeCartesianPath(vector<geometry_msgs::Pose> waypoints, const char* traj_name) {
	namespace rvt = rviz_visual_tools;

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
	ROS_INFO_NAMED("robot_planner1", "Visualizing plan for %s (Cartesian path) (%.2f%% achieved)", traj_name, fraction * 100.0);

	// Visualize the plan in RViz
	// visual_tools.deleteAllMarkers();
	visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
	for (std::size_t i = 0; i < waypoints.size(); ++i)
		visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
	visual_tools.trigger();

	moveit::planning_interface::MoveGroupInterface::Plan insertion_plan;
	bool success = (move_group.plan(insertion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("robot_planner1", "Planning time for path %s was %.6f seconds", traj_name, insertion_plan.planning_time_);
	if (success) {
		insertion_plan.trajectory_ = trajectory;
		move_group.execute(insertion_plan);
	}
	ROS_INFO_NAMED("robot_planner1", "Executing %s plan %s", traj_name, success ? "SUCCESS" : "FAILED");
}
