#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner4");
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
	// TCP position for point above cube
	path1.push_back({0.004149, -0.719461, 1.366577, 3.139543, 0.022613, -1.657719});
	traj1.executePath(path1);

	// Slowly approaching Cube
	// Move in a line segment while approaching cube
	geometry_msgs::Pose aboveCubePose = getPoseFromPathPoint(path1.at(path1.size()-1)); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(aboveCubePose);
	vector<float> onCubeFloatPose = {0.004149, -0.719461, 1.202148, 3.139543, 0.022613, -1.657719};
	geometry_msgs::Pose onCubePose = getPoseFromPathPoint(onCubeFloatPose);
	path2.push_back(onCubePose);
	traj1.executeCartesianPath(path2, "approaching cube");

	ros::shutdown();
	return 0;
}


