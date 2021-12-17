#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner2b");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Setup Move group
	static const std::string PLANNING_GROUP = "iiwa_arm";
	double pos_tolerance = 0.0005;
  double orient_tolerance = 0.0005;
	int plan_time_sec = 5;
	bool replanning = true;
	int plan_attempts = 6;
	const string base_frame = "world";
	const string plannerId = "RRTConnect";
	// const string plannerId = "RRTstar";
	TrajectoryExecution traj1 = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, plannerId);

	// X Y Z Roll Pitch Yaw
	vector<vector<float>> preparation_path;
	// path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a signularity and cant be calculated from the numerical IK
	preparation_path.push_back({0, 0, 2.26, 0, 0, 0}); // Home position
	// TCP pose around home position, such that the robot arm starts in an elbow-up configuration
	preparation_path.push_back({0.103454, 0.359961, 1.902053, -3.083623, 1.113981, -1.494644});
	std::vector<geometry_msgs::Pose> path0;
	path0.push_back(getPoseFromPathPoint(preparation_path.at(0)));
	path0.push_back(getPoseFromPathPoint(preparation_path.at(1)));
	traj1.executeCartesianPath(path0, "preparation path for elbow-up configuration");

	// TCP pose for point above fulcrum 1
	preparation_path.push_back({0.503454, 0.359961, 1.902053, -3.083623, 1.113981, -1.494644});
	std::vector<geometry_msgs::Pose> path1;
	path1.push_back(getPoseFromPathPoint(preparation_path.at(1)));
	path1.push_back(getPoseFromPathPoint(preparation_path.at(2)));
	traj1.executeCartesianPath(path1, "movement towards above fulcrum point 1");

	// Approaching Fulcrum point 1 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	geometry_msgs::Pose fulcrumAbovePose1 = path1.at(1); // Start insertion trajectory from last target point of previous trajectory
	std::vector<geometry_msgs::Pose> path2;
	path2.push_back(fulcrumAbovePose1);
	vector<float> fulcrumInsertedPoseFloat1 = {0.514371, 0.217107, 1.610499, -3.083623, 1.113981, -1.494644};
	geometry_msgs::Pose fulcrumInsertedPose1 = getPoseFromPathPoint(fulcrumInsertedPoseFloat1);
	path2.push_back(fulcrumInsertedPose1);
	traj1.executeCartesianPath(path2, "trocar 1 insertion movement");
	// Reverse insertion movement - Remove tool from trocar1
	std::vector<geometry_msgs::Pose> path3;
	path3.push_back(fulcrumInsertedPose1);
	path3.push_back(fulcrumAbovePose1);
	traj1.executeCartesianPath(path3, "trocar 1 reverse insertion movement");

	// Go above fulcrum point 2
	std::vector<geometry_msgs::Pose> path4;
	path4.push_back(fulcrumAbovePose1);
	vector<float> fulcrumAbovePoseFloat2 = {0.509293, 0.029401, 1.986541, -2.971997, 1.421569, -1.378815};
	geometry_msgs::Pose fulcrumAbovePose2 = getPoseFromPathPoint(fulcrumAbovePoseFloat2);
	path4.push_back(fulcrumAbovePose2);
	traj1.executeCartesianPath(path4, "movement towards above fulcrum point 2");

	// Approaching Fulcrum point 2 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	std::vector<geometry_msgs::Pose> path5;
	path5.push_back(fulcrumAbovePose2);
	vector<float> fulcrumInsertedPoseFloat2 = {0.517311, -0.011860, 1.706976, -2.972108, 1.421710, -1.378877};
	geometry_msgs::Pose fulcrumInsertedPose2 = getPoseFromPathPoint(fulcrumInsertedPoseFloat2);
	path5.push_back(fulcrumInsertedPose2);
	traj1.executeCartesianPath(path5, "trocar 2 insertion movement");
	// Reverse insertion movement - Remove tool from trocar2
	std::vector<geometry_msgs::Pose> path6;
	path6.push_back(fulcrumInsertedPose2);
	path6.push_back(fulcrumAbovePose2);
	traj1.executeCartesianPath(path6, "trocar 2 reverse insertion movement");

	// Go above fulcrum point 3
	std::vector<geometry_msgs::Pose> path7;
	path7.push_back(fulcrumAbovePose2);
	vector<float> fulcrumAbovePoseFloat3 = {0.522168, -0.093628, 1.976414, 2.971154, 1.389222, 1.391472};
	geometry_msgs::Pose fulcrumAbovePose3 = getPoseFromPathPoint(fulcrumAbovePoseFloat3);
	path7.push_back(fulcrumAbovePose3);
	traj1.executePath(path7, "movement towards above fulcrum point 3");

	// Approaching Fulcrum point 3 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	std::vector<geometry_msgs::Pose> path8;
	path8.push_back(fulcrumAbovePose3);
	vector<float> fulcrumInsertedPoseFloat3 = {0.531381, -0.042863, 1.694900, 2.971154, 1.389222, 1.391472};
	geometry_msgs::Pose fulcrumInsertedPose3 = getPoseFromPathPoint(fulcrumInsertedPoseFloat3);
	path8.push_back(fulcrumInsertedPose3);
	traj1.executeCartesianPath(path8, "trocar 3 insertion movement");
	// Reverse insertion movement - Remove tool from trocar2
	std::vector<geometry_msgs::Pose> path9;
	path9.push_back(fulcrumInsertedPose3);
	path9.push_back(fulcrumAbovePose3);
	traj1.executeCartesianPath(path9, "trocar 3 reverse insertion movement");

	// Go above fulcrum point 4
	std::vector<geometry_msgs::Pose> path10;
	path10.push_back(fulcrumAbovePose3);
	vector<float> fulcrumAbovePoseFloat4 = {0.519565, -0.394395, 1.853448, 3.077139, 1.095558, 1.501610};
	geometry_msgs::Pose fulcrumAbovePose4 = getPoseFromPathPoint(fulcrumAbovePoseFloat4);
	path10.push_back(fulcrumAbovePose4);
	traj1.executePath(path10, "movement towards above fulcrum point 4");

	// Approaching Fulcrum point 4 - Insertion motion Cartesian path
	// Move in a line segment while approaching fulcrum point and entering body
	// std::vector<geometry_msgs::Pose> path11;
	// path11.push_back(fulcrumAbovePose4);
	// vector<float> fulcrumInsertedPoseFloat4 = {0.525619, -0.307192, 1.683568, 3.077139, 1.095558, 1.501610};
	// geometry_msgs::Pose fulcrumInsertedPose4 = getPoseFromPathPoint(fulcrumInsertedPoseFloat4);
	// path11.push_back(fulcrumInsertedPose4);
	// traj1.executeCartesianPath(path11, "insertion movement");
	// // Reverse insertion movement - Remove tool from trocar2
	// std::vector<geometry_msgs::Pose> path12;
	// path12.push_back(fulcrumInsertedPose3);
	// path12.push_back(fulcrumAbovePose3);
	// traj1.executeCartesianPath(path12, "reverse insertion movement");

	ros::shutdown();
	return 0;
}


