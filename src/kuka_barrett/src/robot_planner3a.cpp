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
#include <moveit_visual_tools/moveit_visual_tools.h>


using namespace std;
namespace rvt = rviz_visual_tools;

typedef geometry_msgs::PoseWithCovarianceStamped covPose;


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

	moveit_visual_tools::MoveItVisualTools visual_tools = moveit_visual_tools::MoveItVisualTools(base_frame);

	// X Y Z Roll Pitch Yaw
	vector<vector<float>> preparation_path;
	// TCP pose around home position, such that the robot arm starts in an elbow-up configuration
	preparation_path.push_back({0.1, 0.087249, 1.8, 2.952052, 1.311528, -1.750799});
	traj1.moveToTarget(getPoseFromPathPoint(preparation_path.at(0)));

	std::vector<geometry_msgs::Pose> path1;
	preparation_path.push_back({0.1, 0.087249, 1.953192, 2.952052, 1.311528, -1.750799});
	path1.push_back(getPoseFromPathPoint(preparation_path.at(0)));
	path1.push_back(getPoseFromPathPoint(preparation_path.at(1)));
	traj1.executeCartesianPath(path1, "elbow-up preparation path");

	// TCP pose for point above fulcrum 1
	preparation_path.push_back({0.552931, 0.087249, 1.953192, 2.952052, 1.311528, -1.750799});
	path1.clear();
	path1.push_back(getPoseFromPathPoint(preparation_path.at(1)));
	path1.push_back(getPoseFromPathPoint(preparation_path.at(2)));
	traj1.executeCartesianPath(path1, "movement towards above fulcrum point 1");

	// Approaching Fulcrum point 1 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose fulcrumAbovePose1 = path1.at(path1.size()-1); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(fulcrumAbovePose1);
	vector<float> fulcrumInsertedPoseFloat1 = {0.544067, 0.038585, 1.756649, 2.951693, 1.311653, -1.751226};
	geometry_msgs::Pose fulcrumInsertedPose1 = getPoseFromPathPoint(fulcrumInsertedPoseFloat1);
	path2.push_back(fulcrumInsertedPose1);
	traj1.executeCartesianPath(path2, "insertion movement");

	Eigen::Vector3f circleTrajCenter;
	// Initialize vector with known values https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html
	// values are given in x, y, z order
	// circleTrajCenter << 0.259807, 0.689203, 1.174661;
	circleTrajCenter << 0.0, 0.0, -0.1; // Coordinates of desired circle w.r.t. to {F} reference frame
	CircleTrajectory* circleTrajectory = new CircleTrajectory(circleTrajCenter, 0.1);
	vector<geometry_msgs::Pose> circle_waypoints = circleTrajectory->getCartesianWaypoints(20, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
	vector<geometry_msgs::Pose> transformed_waypoints = fulcrumEffectTransformation(circle_waypoints, 0.4, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());

	boost::shared_ptr<covPose const> sharedPoseMsg;
	covPose pose_msg;
	sharedPoseMsg = ros::topic::waitForMessage<covPose>("fulcrum/estimated/frame2", node_handle);
	if(sharedPoseMsg != nullptr){
		pose_msg = *sharedPoseMsg;
	}
	auto fp = pose_msg.pose.pose;
	auto _p = fp.position;
	tf2::Vector3 pa = tf2::Vector3(_p.x, _p.y, _p.z);
	auto _q = fp.orientation;
	tf2::Quaternion qa = tf2::Quaternion(_q.x, _q.y, _q.z, _q.w);
	tf2::Quaternion rot1 = tf2::Quaternion();
	rot1.setRPY(0, M_PI_2, 0);

	tf2::Matrix3x3 identity3x3 = tf2::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
	vector<geometry_msgs::Pose> circle_waypoints2, transformed_waypoints2;
	for (auto circle_point_pose: transformed_waypoints) {
		geometry_msgs::Pose new_pose;
		auto cp = circle_point_pose.position;
		double px = circle_point_pose.position.x;
		double py = circle_point_pose.position.y;
		double pz = circle_point_pose.position.z;
		tf2::Vector3 dxa = tf2::Vector3(px, py, pz);
		double th = atan2(sqrt(px*px + py*py), pz);
		double phi = atan2(py, px);
		tf2::Quaternion rot2 = tf2::Quaternion();
		rot2.setRPY(0, 0, phi);
		tf2::Quaternion rot3 = tf2::Quaternion();
		rot3.setRPY(0, M_PI-th, 0);
		tf2::Quaternion rot4 = tf2::Quaternion();
		// tf2::Transform Td = tf2::Transform(identity3x3, tf2::Vector3(0, - 0.094 - 0.022, -0.05)) * tf2::Transform(qa, pa) * tf2::Transform(rot2, dxa) * tf2::Transform(rot3, tf2::Vector3(0, 0, 0));
		tf2::Transform Td = tf2::Transform(qa, pa) * tf2::Transform(rot2, dxa) * tf2::Transform(rot3, tf2::Vector3(0, 0, 0));
		tf2::Transform tfa = Td;
		tf2::toMsg(tfa, new_pose);
		transformed_waypoints2.push_back(new_pose);
	}

	for (auto circle_point_pose: circle_waypoints) {
		geometry_msgs::Pose new_pose;
		auto cp = circle_point_pose.position;
		double px = circle_point_pose.position.x;
		double py = circle_point_pose.position.y;
		double pz = circle_point_pose.position.z;
		tf2::Vector3 dxa = tf2::Vector3(px, py, pz);
		double th = atan2(sqrt(px*px + py*py), pz);
		double phi = atan2(py, px);
		tf2::Quaternion rot2 = tf2::Quaternion();
		rot2.setRPY(0, 0, phi);
		tf2::Quaternion rot3 = tf2::Quaternion();
		rot3.setRPY(0, -th, 0);
		tf2::Quaternion rot4 = tf2::Quaternion();
		// tf2::Transform Td = tf2::Transform(identity3x3, tf2::Vector3(0, - 0.094 - 0.022, -0.05)) * tf2::Transform(qa, pa) * tf2::Transform(rot2, dxa) * tf2::Transform(rot3, tf2::Vector3(0, 0, 0));
		tf2::Transform Td = tf2::Transform(qa, pa) * tf2::Transform(rot2, dxa) * tf2::Transform(rot3, tf2::Vector3(0, 0, 0));
		tf2::Transform tfa = Td;
		tf2::toMsg(tfa, new_pose);
		circle_waypoints2.push_back(new_pose);
	}

	// Path to circle
	std::vector<geometry_msgs::Pose> path3;
	path3.push_back(fulcrumInsertedPose1);
	path3.push_back(circle_waypoints2.at(0));

	// traj1.executePath(path3, "Path to circle");
	traj1.moveToTarget(circle_waypoints2.at(0));
	traj1.executeCartesianPath(transformed_waypoints2, "Circular Trajectory", true);

	traj1.visualizeCartesianPath(circle_waypoints2, "Original taskspace circular trajectory", true);

	ros::shutdown();
	return 0;
}


