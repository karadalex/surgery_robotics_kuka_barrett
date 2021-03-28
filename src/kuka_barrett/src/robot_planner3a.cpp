//
// Created by karadalex on 11/8/20.
//

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"
#include "trajectory/CircleTrajectory.h"
#include "kinematics/Pose.h"
#include <trajectory_msgs/JointTrajectory.h>


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner3a");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher kuka_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory > ("/arm_controller/command", 1000);

	// Load robot
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	// Create Kinematic state
	moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("iiwa_arm");

	// Set end-effector state
	kinematic_state->setToRandomPositions(joint_model_group);

	// Setup Move group
	static const std::string PLANNING_GROUP = "iiwa_arm";
	double pos_tolerance = 0.000005;
	double orient_tolerance = 0.000005;
	int plan_time_sec = 5;
	bool replanning = true;
	int plan_attempts = 6;
	const string base_frame = "world";
	TrajectoryExecution traj1 = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame);

	// X Y Z Roll Pitch Yaw
	vector<vector<float>> path1;
	// path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a signularity and cant be calculated from the numerical IK
	path1.push_back({0, 0, 2.26, 0, 0, 0}); // Home position
	// TCP position for point above fulcrum 1
	path1.push_back({0.505761, 0.339868, 1.884992, 0.024080, 1.242948, -1.444770});
	traj1.executePath(path1);

	// Approaching Fulcrum point 1 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose fulcrumAbovePose1 = getPoseFromPathPoint(path1.at(path1.size()-1)); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(fulcrumAbovePose1);
	vector<float> fulcrumInsertedPoseFloat1 = {0.515165, 0.265558, 1.664860, 0.023993, 1.242965, -1444829};
	geometry_msgs::Pose fulcrumInsertedPose1 = getPoseFromPathPoint(fulcrumInsertedPoseFloat1);
	path2.push_back(fulcrumInsertedPose1);
	traj1.executeCartesianPath(path2, "insertion movement");
	// Reverse insertion movement - Remove tool from trocar1
	// std::vector<geometry_msgs::Pose> path3;
	// path3.push_back(fulcrumInsertedPose1);
	// path3.push_back(fulcrumAbovePose1);
	// traj1.executeCartesianPath(path3, "reverse insertion movement");

	// Get transformation matrix of reference frame {F} (Fulcrum reference frame) w.r.t. to the universal reference frame {U}
	Pose* FPose = new Pose(0.531483, 0.058691, 1.398114, -0.018658, -0.594264, 0.009121);
	Eigen::Matrix4d U_T_F = FPose->pose;


	Eigen::Vector3f circleTrajCenter;
	// Initialize vector with known values https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html
	// values are given in x, y, z order
	// circleTrajCenter << 0.259807, 0.689203, 1.174661;
	circleTrajCenter << 0, 0, 0.2; // Coordinates of desired circle w.r.t. to {F} reference frame
	CircleTrajectory* circleTrajectory = new CircleTrajectory(circleTrajCenter, 0.2);
	vector<geometry_msgs::Pose> circle_waypoints = circleTrajectory->getCartesianWaypoints(20, U_T_F);

	// Path to circle
	std::vector<geometry_msgs::Pose> path3;
	path3.push_back(fulcrumInsertedPose1);
	path3.push_back(circle_waypoints.at(0));
	traj1.executeCartesianPath(path3, "Path to circle");
	traj1.executeCartesianPath(circle_waypoints, "Circular Trajectory");
	traj1.executeCartesianPath(path3, "Path to circle");

	// trajectory_msgs::JointTrajectory kuka_msg;
	// kuka_msg.header.seq = 0;
	// kuka_msg.header.stamp.sec = 0;
	// kuka_msg.header.stamp.nsec = 0;
	// kuka_msg.header.frame_id = "";
	// kuka_msg.joint_names = joint_model_group->getVariableNames();
	// vector<Eigen::Isometry3d> eigen_waypoints = circleTrajectory->eigen_waypoints;
	// kuka_msg.points.resize(eigen_waypoints.size());
	// for (int i = 0; i < eigen_waypoints.size(); ++i) {
	// 	Eigen::Isometry3d pose = eigen_waypoints.at(i);
	// 	std::vector<double> joint_values;
	// 	double timeout = 0.1;
	// 	bool found_ik = kinematic_state->setFromIK(joint_model_group, pose, timeout);
	// 	if (found_ik) {
	// 		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	// 		kuka_msg.points[i].positions.resize(kuka_msg.joint_names.size());
	// 		kuka_msg.points[i].velocities.resize(kuka_msg.joint_names.size());
	// 		kuka_msg.points[i].effort.resize(kuka_msg.joint_names.size());
	// 		for (int j = 0; j < 7; ++j) {
	// 			kuka_msg.points[i].positions[j] = joint_values[j];
	// 			kuka_msg.points[i].velocities[j] = 0.0;
	// 		}
	// 		// To be reached 1 second after starting along the trajectory
	// 		kuka_msg.points[i].time_from_start = ros::Duration(3*(i+1));
	// 	} else {
	// 		for (int j = 0; j < 7; ++j) {
	// 			kuka_msg.points[i].positions[j] = 0.0;
	// 			kuka_msg.points[i].velocities[j] = 0.0;
	// 		}
	// 		ROS_INFO("Did not find IK solution");
	// 	}
	// }
	// kuka_publisher.publish(kuka_msg);

	ros::shutdown();
	return 0;
}


