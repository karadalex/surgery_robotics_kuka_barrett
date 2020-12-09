/**
 * Robot Planner 4:
 * Planning a simple pick and place program using the gazebo world 2 (run it using roslaunch main program2.launch)
 * Code based on http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
 *
 * Author: Alexios Karadimos
 */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"
#include "kinematics/BarrettInv.h"


using namespace std;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
	posture.joint_names.resize(8);
	posture.joint_names = {
					"bh_j11_joint", "bh_j12_joint", "bh_j13_joint",
					"bh_j21_joint", "bh_j22_joint", "bh_j23_joint",
					"bh_j32_joint", "bh_j33_joint"
	};

	/* Set them as open, wide enough for the object to fit. */
	posture.points.resize(1);
	posture.points[0].positions.resize(8);
	for (int j = 0; j < posture.joint_names.size(); ++j)
		posture.points[0].positions[j] = 0.0;
	posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
	/* Add both finger joints of panda robot. */
	posture.joint_names.resize(8);
	posture.joint_names = {
					"bh_j11_joint", "bh_j12_joint", "bh_j13_joint",
					"bh_j21_joint", "bh_j22_joint", "bh_j23_joint",
					"bh_j32_joint", "bh_j33_joint"
	};

	/* Set them as closed. */
	posture.points.resize(1);
	posture.points[0].positions.resize(8);
	for (int j = 0; j < posture.joint_names.size(); ++j)
		posture.points[0].positions[j] = 0.8;
	posture.points[0].time_from_start = ros::Duration(0.5);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner4");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::WallDuration(1.0).sleep();
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface move_group("iiwa_arm");
	move_group.setPlanningTime(45.0);
	move_group.allowReplanning(true);
	move_group.setNumPlanningAttempts(3);

	ros::WallDuration(1.0).sleep();

	// // Setup Move group
	// static const std::string PLANNING_GROUP = "iiwa_arm";
	// double pos_tolerance = 0.00005;
	// double orient_tolerance = 0.00005;
	// int plan_time_sec = 5;
	// bool replanning = true;
	// int plan_attempts = 6;
	// const string base_frame = "world";
	// TrajectoryExecution traj1 = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame);
	//
	// // X Y Z Roll Pitch Yaw
	// vector<vector<float>> path1;
	// // path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a signularity and cant be calculated from the numerical IK
	// path1.push_back({0, 0, 2.26, 0, 0, 0}); // Home position
	// // TCP position for point above cube
	// path1.push_back({0.004149, -0.719461, 1.366577, 3.139543, 0.022613, -1.657719});
	// traj1.executePath(path1);
	//
	// // Slowly approaching Cube
	// // Move in a line segment while approaching cube
	// geometry_msgs::Pose aboveCubePose = getPoseFromPathPoint(path1.at(path1.size()-1));
	// std::vector<geometry_msgs::Pose> path2;
	// path2.push_back(aboveCubePose);
	// vector<float> onCubeFloatPose = {0.004149, -0.719461, 1.202148, 3.139543, 0.022613, -1.657719};
	// geometry_msgs::Pose onCubePose = getPoseFromPathPoint(onCubeFloatPose);
	// path2.push_back(onCubePose);
	// traj1.executeCartesianPath(path2, "approaching cube");

	// Pick Pipeline
	std::vector<moveit_msgs::Grasp> grasps;
	grasps.resize(1);

	// Setting grasp pose
	grasps[0].grasp_pose.header.frame_id = "world";
	tf2::Quaternion orientation;
	orientation.setRPY(3.139543, 0.022613, -1.657719);
	grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
	grasps[0].grasp_pose.pose.position.x = 0.004149;
	grasps[0].grasp_pose.pose.position.y = -0.719461;
	grasps[0].grasp_pose.pose.position.z = 1.302148;

	// Setting pre-grasp approach
	// Defined with respect to frame_id
	grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
	// Direction is set as positive x axis
	grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
	grasps[0].pre_grasp_approach.min_distance = 0.195;
	grasps[0].pre_grasp_approach.desired_distance = 0.215;

	// Setting post-grasp retreat
	// Defined with respect to frame_id
	grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
	// Direction is set as positive z axis
	grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
	grasps[0].post_grasp_retreat.min_distance = 0.1;
	grasps[0].post_grasp_retreat.desired_distance = 0.25;

	// Setting posture of eef before grasp
	openGripper(grasps[0].pre_grasp_posture);

	// Setting posture of eef during grasp
	closedGripper(grasps[0].grasp_posture);

	// Set support surface as table1.
	move_group.setSupportSurfaceName("table1");
	// Call pick to pick up the object using the grasps given
	move_group.pick("cube", grasps);

	ros::shutdown();
	return 0;
}


