#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include "kinematics/BarrettInv.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_controller_moveit");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

	// The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools.loadRemoteControl();

	// Setup Move group
  static const std::string PLANNING_GROUP = "iiwa_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group.setGoalPositionTolerance(0.00005);
	move_group.setGoalOrientationTolerance(0.00005);
  // move_group.setGoalPositionTolerance(0.5);
  // move_group.setGoalOrientationTolerance(0.5);
	move_group.setPlanningTime(10);
	move_group.allowReplanning(true);

	// Barrett hand publishers
	ros::Publisher bh_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory > ("/gripper_controller/command", 1000);

	vector<vector<float>> path;
	vector<vector<float>> grasps;
	// X Y Z Roll Pitch Yaw
	// Pick position 1
	path.push_back({0, -0.68, 1.5, M_PI, 0, M_PI_2});
	grasps.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0});
	path.push_back({0, -0.68, 1.19, M_PI, 0, M_PI_2});
	grasps.push_back({0.04, 0.04, 0.07, 0.04, 0.04, 0.07, 0.04, 0.04, 0.07});
	path.push_back({0, -0.68, 1.5, M_PI, 0, M_PI_2});
	grasps.push_back({0.04, 0.04, 0.07, 0.04, 0.04, 0.07, 0.04, 0.04, 0.07});
	// Fulcrum position 1
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});
	grasps.push_back({0.04, 0.04, 0.07, 0.04, 0.04, 0.07, 0.04, 0.04, 0.07});
	path.push_back({0.06, 0.70, 1.38, M_PI, -0.59, 0});
	grasps.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0});
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});
	grasps.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0});
	// Pick position 2
	path.push_back({0.2, -0.68, 1.5, M_PI, 0, M_PI_2});
	grasps.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0});
	path.push_back({0.2, -0.68, 1.19, M_PI, 0, M_PI_2});
	grasps.push_back({0.04, 0.04, 0.07, 0.04, 0.04, 0.07, 0.04, 0.04, 0.07});
	path.push_back({0.2, -0.68, 1.5, M_PI, 0, M_PI_2});
	grasps.push_back({0.04, 0.04, 0.07, 0.04, 0.04, 0.07, 0.04, 0.04, 0.07});
	// Fulcrum position 2
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});
	grasps.push_back({0.04, 0.04, 0.07, 0.04, 0.04, 0.07, 0.04, 0.04, 0.07});
	path.push_back({0.06, 0.70, 1.38, M_PI, -0.59, 0});
	grasps.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0});
	path.push_back({-0.14, 0.68, 1.68, M_PI, -0.59, 0});
	grasps.push_back({0, 0, 0, 0, 0, 0, 0, 0, 0});

	// Build Barrett Hand joint trajectory message
	trajectory_msgs::JointTrajectory bh_msg;
	bh_msg.header.seq = 0;
	bh_msg.header.stamp.sec = 0;
	bh_msg.header.stamp.nsec = 0;
	bh_msg.header.frame_id = "";
	bh_msg.joint_names.push_back("bh_j11_joint");
	bh_msg.joint_names.push_back("bh_j12_joint");
	bh_msg.joint_names.push_back("bh_j13_joint");
	bh_msg.joint_names.push_back("bh_j21_joint");
	bh_msg.joint_names.push_back("bh_j22_joint");
	bh_msg.joint_names.push_back("bh_j23_joint");
	bh_msg.joint_names.push_back("bh_j32_joint");
	bh_msg.joint_names.push_back("bh_j33_joint");
	bh_msg.points.resize(path.size());

	geometry_msgs::Pose target_pose;
	tf2::Quaternion quaternion;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	for (int i = 0; i < path.size(); ++i) {
		// Arm Kinematics (KUKA iiwa)
		vector<float> pose = path.at(i);
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];
		quaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.w = quaternion.getW();
		target_pose.orientation.x = quaternion.getX();
		target_pose.orientation.y = quaternion.getY();
		target_pose.orientation.z = quaternion.getZ();
		move_group.setPoseTarget(target_pose);
		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO_NAMED("iiwa_planning", "Visualizing plan %d (pose goal) %s", i, success ? "SUCCESS" : "FAILED");
		std::string nextButtonMsg = "Press 'next' in the RvizVisualToolsGui window to execute plan";
		// visual_tools.prompt(nextButtonMsg);
		move_group.move();

		// Gripper Kinematics (Barrett hand)
		vector<float> gripper_target = grasps.at(i);
		BarrettInv* gripper = new BarrettInv(gripper_target);
		// Barrett hand command values
		vector<float> bh_angles;
		bh_angles = gripper->solutionSet[0]; // Select first solution of the solution set
		// bh_angles = {0, 0.5f, 0.5f, 0, 0.5f, 0.5f, 0.5f, 0.5f};
		// bh_angles = {0, 0, 0, 0, 0, 0, 0, 0};
		cout << gripper->solutionSet[0][0] << " " << gripper->solutionSet[0][1] << " " << gripper->solutionSet[0][2] << endl;

		bh_msg.points[i].positions.resize(8);
		for (int joint = 0; joint < bh_msg.joint_names.size(); ++joint) {
			bh_msg.points[i].positions[joint] = bh_angles[joint];
		}

		// Velocities
		bh_msg.points[i].velocities.resize(8);
		bh_msg.points[i].effort.resize(8);
		for (int joint = 0; joint < bh_msg.joint_names.size(); ++joint)
		{
			bh_msg.points[i].velocities[joint]=0.0;
			bh_msg.points[i].effort[joint] = 0.0;
		}
		// To be reached 1 second after starting along the trajectory
		bh_msg.points[i].time_from_start = ros::Duration(i+1);

		bh_publisher.publish(bh_msg);
	}


  ros::shutdown();
  return 0;
}
