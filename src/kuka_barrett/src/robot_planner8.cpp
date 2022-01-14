/**
 * Robot Planner 8
 *
 * Author: Alexios Karadimos
 */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include "kinematics/TrajectoryExecution.h"
#include "kinematics/Pose.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "trajectory/LineSegTrajectory.h"


using namespace std;
namespace rvt = rviz_visual_tools;

typedef geometry_msgs::PoseWithCovarianceStamped covPose;


void fulcrumFrameUpdate(const covPose::ConstPtr &msg) {
	// TODO
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner8");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// https://answers.ros.org/question/293890/how-to-use-waitformessage-properly/?answer=294479#post-id-294479
	// https://answers.ros.org/question/191012/problem-with-subscribe-and-callback-function/?answer=191017#post-id-191017
	boost::shared_ptr<covPose const> sharedPoseMsg;
	covPose pose_msg;
	sharedPoseMsg = ros::topic::waitForMessage<covPose>("fulcrum/estimated/frame2", node_handle);
	if(sharedPoseMsg != nullptr){
		pose_msg = *sharedPoseMsg;
	}

	// Not yet used, added mostly so that the topic dependency appears in the rqt_graph
	ros::Subscriber sub_fulcrum = node_handle.subscribe("fulcrum/estimated/frame2", 1000, fulcrumFrameUpdate);

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
	geometry_msgs::Pose trajStartPose;
	auto _p = pose_msg.pose.pose.position;
	tf2::Vector3 pa = tf2::Vector3(_p.x, _p.y, _p.z);
	auto _q = pose_msg.pose.pose.orientation;
	tf2::Quaternion qa = tf2::Quaternion(_q.x, _q.y, _q.z, _q.w);
	tf2::Quaternion rot1 = tf2::Quaternion();
	rot1.setRPY(0, M_PI_2, M_PI_2);
	tf2::Quaternion rot2 = tf2::Quaternion();
	rot2.setRPY(0, 0, M_PI_2);
	tf2::Vector3 dxa = tf2::Vector3(-0.5 - 0.05, 0, - 0.094 - 0.022);
	tf2::Transform Td = tf2::Transform(tf2::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), dxa);
	tf2::Transform tfa = tf2::Transform(qa, pa) * tf2::Transform(rot1, tf2::Vector3(0, 0, 0)) * Td;
	tf2::toMsg(tfa, trajStartPose);

	path1.clear();
	path1.push_back(getPoseFromPathPoint(preparation_path.at(1)));
	path1.push_back(trajStartPose);
	traj1.executeCartesianPath(path1, "movement towards above fulcrum point 1");

	// Approaching Fulcrum point 1 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose fulcrumAbovePose1 = path1.at(path1.size()-1); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(fulcrumAbovePose1);
	geometry_msgs::Pose fulcrumInsertedPose1;
	tf2::Vector3 dxb = tf2::Vector3(0.2, 0, 0);
	tf2::Transform Tdb = tf2::Transform(tf2::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), dxb);
	tf2::Transform tfb = tfa * Tdb;
	tf2::toMsg(tfb, fulcrumInsertedPose1);
	path2.push_back(fulcrumInsertedPose1);
	traj1.executeCartesianPath(path2, "insertion movement");

	ros::Duration(1).sleep();

	// Retraction
	path2.clear();
	path2.push_back(fulcrumInsertedPose1);
	path2.push_back(trajStartPose);
	traj1.executeCartesianPath(path2, "retraction movement");

	ros::shutdown();
	return 0;
}
