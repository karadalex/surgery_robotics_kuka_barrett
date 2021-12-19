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
#include "kinematics/utils.h"


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner3a");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Setup Move group
	static const std::string PLANNING_GROUP = "iiwa_arm";
	double pos_tolerance = 0.000005;
	double orient_tolerance = 0.000005;
	int plan_time_sec = 5;
	bool replanning = true;
	int plan_attempts = 6;
	const string base_frame = "world";
	TrajectoryExecution traj1 = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, node_handle);

	// X Y Z Roll Pitch Yaw
	vector<vector<float>> path1;
	// path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a signularity and cant be calculated from the numerical IK
	path1.push_back({0, 0, 2.26, 0, 0, 0}); // Home position
	// TCP position for point above fulcrum 1
	path1.push_back({0.529857, -0.051310, 1.964421, 0.0, M_PI_2, M_PI_2});
	traj1.moveToTarget(getPoseFromPathPoint(path1.at(1)));

	// Approaching Fulcrum point 1 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose fulcrumAbovePose1 = getPoseFromPathPoint(path1.at(path1.size()-1)); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(fulcrumAbovePose1);
	vector<float> fulcrumInsertedPoseFloat1 = {0.529857, -0.051310, 1.741838, 0.0, M_PI_2, M_PI_2};
	geometry_msgs::Pose fulcrumInsertedPose1 = getPoseFromPathPoint(fulcrumInsertedPoseFloat1);
	path2.push_back(fulcrumInsertedPose1);
	traj1.executeCartesianPath(path2, "insertion movement");
	// Reverse insertion movement - Remove tool from trocar1
	// std::vector<geometry_msgs::Pose> path3;
	// path3.push_back(fulcrumInsertedPose1);
	// path3.push_back(fulcrumAbovePose1);
	// traj1.executeCartesianPath(path3, "reverse insertion movement");

	// Get transformation matrix of reference frame {F} (Fulcrum reference frame) w.r.t. to the universal reference frame {U}
	Pose* FPose = new Pose(0.529996, 0.059271, 1.398114, 0, 0.0, -0.271542);
	Eigen::Matrix4d U_T_F = FPose->pose;
	Eigen::Matrix4d T_7_TCP = Eigen::Matrix4d::Zero(4, 4);
	T_7_TCP(0, 2) = 1; T_7_TCP(1, 0) = 1;
	T_7_TCP(2, 1) = 1; T_7_TCP(3, 3) = 1;
	Eigen::Matrix4d T_TCP_7 = T_7_TCP.inverse();

	Eigen::Vector3f circleTrajCenter;
	// Initialize vector with known values https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html
	// values are given in x, y, z order
	// circleTrajCenter << 0.259807, 0.689203, 1.174661;
	circleTrajCenter << 0.0, 0.0, -0.1; // Coordinates of desired circle w.r.t. to {F} reference frame
	CircleTrajectory* circleTrajectory = new CircleTrajectory(circleTrajCenter, 0.1);
	vector<geometry_msgs::Pose> circle_waypoints = circleTrajectory->getCartesianWaypoints(20, U_T_F, T_TCP_7);
	vector<geometry_msgs::Pose> transformed_waypoints = fulcrumEffectTransformation(circle_waypoints, 0.4, U_T_F, T_TCP_7);
	for (int j = 0; j < circle_waypoints.size(); ++j) {
		transformed_waypoints.push_back(circle_waypoints.at(j));
	}

	// Path to circle
	std::vector<geometry_msgs::Pose> path3;
	path3.push_back(fulcrumInsertedPose1);
	path3.push_back(transformed_waypoints.at(0));
	traj1.executeCartesianPath(path3, "Path to circle");

	traj1.executeCartesianPath(transformed_waypoints, "Circular Trajectory");

	traj1.executeCartesianPath(path3, "Path to circle");


	ros::shutdown();
	return 0;
}


