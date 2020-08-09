#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
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
	// Above Pick position 1
	path1.push_back({0, -0.68, 1.5, M_PI, 0, -M_PI_2});
	traj1.executePath(path1);

	// Approach slowly pick position 1 with straight line segment (cartesain)
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(getPoseFromPathPoint({0, -0.68, 1.5, M_PI, 0, -M_PI_2}));
	path2.push_back(getPoseFromPathPoint({0, -0.68, 1.30, M_PI, 0, -M_PI_2}));
	traj1.executeCartesianPath(path2, "approaching pick position 1");
	std::vector<geometry_msgs::Pose> path3;
	path3.push_back(getPoseFromPathPoint({0, -0.68, 1.30, M_PI, 0, -M_PI_2}));
	path3.push_back(getPoseFromPathPoint({0, -0.68, 1.5, M_PI, 0, -M_PI_2}));
	traj1.executeCartesianPath(path3, "moving away from pick position 1");

	// TCP position for point above fulcrum 1
	std::vector<vector<float>> path4;
	// path4.push_back({0, -0.68, 1.5, M_PI, 0, -M_PI_2});
	path4.push_back({-0.320971, 0.681543, 1.656761, 0.019169, 0.783348, 0.073975});
	traj1.executePath(path4);

	// Approaching Fulcrum point - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose start_pose = getPoseFromPathPoint(path4.at(path4.size()-1)); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path5;
	path5.push_back(start_pose);
	vector<float> insertedPoseAtFulcrum = {-0.113231, 0.681543, 1.449251, 0.019169, 0.783348, 0.073975};
	geometry_msgs::Pose target_pose = getPoseFromPathPoint(insertedPoseAtFulcrum);
	path5.push_back(target_pose);
	traj1.executeCartesianPath(path5, "insertion movement");

	// Reverse insertion movement - Remove tool from trocar
	std::vector<geometry_msgs::Pose> path6;
	path6.push_back(target_pose);
	path6.push_back(start_pose);
	traj1.executeCartesianPath(path6, "reverse insertion movement");

  ros::shutdown();
  return 0;
}


