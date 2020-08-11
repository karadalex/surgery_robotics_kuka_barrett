#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner1");
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
	TrajectoryExecution traj1 = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame);

	// X Y Z Roll Pitch Yaw
	vector<vector<float>> path1;
	// path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a signularity and cant be calculated from the numerical IK
	path1.push_back({0, 0, 2.26, 0, 0, 0}); // Home position
	// TCP position for point above fulcrum 1
	path1.push_back({-0.320971, 0.681543, 1.656761, 0.019169, 0.783348, 0.073975});
	traj1.executePath(path1);

	// Approaching Fulcrum point 1 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose fulcrumAbovePose1 = getPoseFromPathPoint(path1.at(path1.size()-1)); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(fulcrumAbovePose1);
	vector<float> fulcrumInsertedPoseFloat1 = {-0.113231, 0.681543, 1.449251, 0.019169, 0.783348, 0.073975};
	geometry_msgs::Pose fulcrumInsertedPose1 = getPoseFromPathPoint(fulcrumInsertedPoseFloat1);
	path2.push_back(fulcrumInsertedPose1);
	traj1.executeCartesianPath(path2, "insertion movement");
	// Reverse insertion movement - Remove tool from trocar1
	std::vector<geometry_msgs::Pose> path3;
	path3.push_back(fulcrumInsertedPose1);
	path3.push_back(fulcrumAbovePose1);
	traj1.executeCartesianPath(path3, "reverse insertion movement");

	// Go above fulcrum point 2
	std::vector<geometry_msgs::Pose> path4;
	path4.push_back(fulcrumAbovePose1);
	vector<float> fulcrumAbovePoseFloat2 = {0.039294, 0.683934, 1.844547, 0.040475, 1.222129, 0.098161};
	geometry_msgs::Pose fulcrumAbovePose2 = getPoseFromPathPoint(fulcrumAbovePoseFloat2);
	path4.push_back(fulcrumAbovePose2);
	traj1.executeCartesianPath(path4, "movement towards above fulcrum point 2");

	// Approaching Fulcrum point 2 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	std::vector<geometry_msgs::Pose> path5;
	path5.push_back(fulcrumAbovePose2);
	vector<float> fulcrumInsertedPoseFloat2Intermediary = {0.123900, 0.689796, 1.787654, 0.067338, 1.367060, 0.125863};
	geometry_msgs::Pose fulcrumInsertedPose2Intermediary = getPoseFromPathPoint(fulcrumInsertedPoseFloat2Intermediary);
	path5.push_back(fulcrumInsertedPose2Intermediary);
	vector<float> fulcrumInsertedPoseFloat2 = {0.149236, 0.693004, 1.664076, 0.067349, 1.367062, 0.125869};
	geometry_msgs::Pose fulcrumInsertedPose2 = getPoseFromPathPoint(fulcrumInsertedPoseFloat2);
	path5.push_back(fulcrumInsertedPose2);
	traj1.executeCartesianPath(path5, "insertion movement");
	// Reverse insertion movement - Remove tool from trocar2
	std::vector<geometry_msgs::Pose> path6;
	path6.push_back(fulcrumInsertedPose2);
	path6.push_back(fulcrumInsertedPose2Intermediary);
	path6.push_back(fulcrumAbovePose2);
	traj1.executeCartesianPath(path6, "reverse insertion movement");

	// Go above fulcrum point 3
	std::vector<geometry_msgs::Pose> path7;
	path7.push_back(fulcrumAbovePose2);
	vector<float> fulcrumAbovePoseFloat3 = {0.160094, 0.682022, 1.844775, 0.031093, 1.162609, 0.087358};
	geometry_msgs::Pose fulcrumAbovePose3 = getPoseFromPathPoint(fulcrumAbovePoseFloat3);
	path7.push_back(fulcrumAbovePose3);
	traj1.executeCartesianPath(path7, "movement towards above fulcrum point 3");

	// Approaching Fulcrum point 3 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	std::vector<geometry_msgs::Pose> path8;
	path8.push_back(fulcrumAbovePose3);
	vector<float> fulcrumInsertedPoseFloat3Intermediary = {0.205306, 0.685986, 1.739838, 0.030799, 1.162898, 0.086868};
	geometry_msgs::Pose fulcrumInsertedPose3Intermediary = getPoseFromPathPoint(fulcrumInsertedPoseFloat3Intermediary);
	path8.push_back(fulcrumInsertedPose3Intermediary);
	vector<float> fulcrumInsertedPoseFloat3Intermediary2 = {0.274535, 0.689496, 1.783499, 0.055889, 1.351853, 0.0113223};
	geometry_msgs::Pose fulcrumInsertedPose3Intermediary2 = getPoseFromPathPoint(fulcrumInsertedPoseFloat3Intermediary2);
	path8.push_back(fulcrumInsertedPose3Intermediary2);
	vector<float> fulcrumInsertedPoseFloat3 = {0.301965, 0.692619, 1.659563, 0.055821, 1.351665, 0.113147};
	geometry_msgs::Pose fulcrumInsertedPose3 = getPoseFromPathPoint(fulcrumInsertedPoseFloat3);
	path8.push_back(fulcrumInsertedPose3);
	traj1.executeCartesianPath(path8, "insertion movement");
	// Reverse insertion movement - Remove tool from trocar2
	std::vector<geometry_msgs::Pose> path9;
	path9.push_back(fulcrumInsertedPose3);
	path9.push_back(fulcrumInsertedPose3Intermediary2);
	path9.push_back(fulcrumInsertedPose3Intermediary);
	path9.push_back(fulcrumAbovePose3);
	traj1.executeCartesianPath(path9, "reverse insertion movement");

	ros::shutdown();
	return 0;
}


