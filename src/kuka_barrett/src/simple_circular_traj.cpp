//
// Created by karadalex on 23/8/20.
//

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"
#include "trajectory/CircleTrajectory.h"


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_circular_traj");
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
	vector<geometry_msgs::Pose> path1;
	// path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a signularity and cant be calculated from the numerical IK
	path1.push_back(getPoseFromPathPoint({0, 0, 2.26, 0, 0, 0})); // Home position
	path1.push_back(getPoseFromPathPoint({0.4, 0, 1.8, 0, 0, 0})); // Home position
	traj1.executeCartesianPath(path1, "Lower down from home position");

	Eigen::Vector3f circleTrajCenter;
	// Initialize vector with known values https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html
	// values are given in x, y, z order
	// circleTrajCenter << 0.259807, 0.689203, 1.174661;
	circleTrajCenter << 0.4, 0, 1.8;
	CircleTrajectory* circleTrajectory = new CircleTrajectory(circleTrajCenter, 0.2);
	vector<geometry_msgs::Pose> circle_waypoints = circleTrajectory->getCartesianWaypoints(30);

	// Path to circle
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(path1.at(1));
	path2.push_back(circle_waypoints.at(0));
	traj1.executeCartesianPath(path2, "Path to circle");

	traj1.executeCartesianPath(circle_waypoints, "Circular Trajectory");


	ros::shutdown();
	return 0;
}
