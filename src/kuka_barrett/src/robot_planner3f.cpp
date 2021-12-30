//
// Created by karadalex on 23/03/21.
//

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "trajectory/TrapezoidSingleJointTrajectory.h"


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner3f");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher kuka_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory > ("/arm_controller/command", 1000);

	// Load robot
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	// Create Kinematic state
	moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("iiwa_arm");

	// Set end-effector states to solve for
	tf2::Vector3 position1 = tf2::Vector3(0.1, 0.1, 1.95);
	tf2::Quaternion orientation;
	orientation.setRPY(2.952052, 1.311528, -1.750799);
	tf2::Transform pose1_tf = tf2::Transform(orientation, position1);
	geometry_msgs::Pose pose1, pose2;
	tf2::toMsg(pose1_tf, pose1);
	tf2::Vector3 position2 = tf2::Vector3(0.4, 0.1, 1.95);
	tf2::Transform pose2_tf = tf2::Transform(orientation, position2);
	tf2::toMsg(pose2_tf, pose2);

	// Prepare and initialize JointTrajectory ROS message
	trajectory_msgs::JointTrajectory kuka_msg;
	kuka_msg.header.seq = 0;
	kuka_msg.header.stamp.sec = 0;
	kuka_msg.header.stamp.nsec = 0;
	kuka_msg.header.frame_id = "";
	kuka_msg.joint_names = joint_model_group->getVariableNames();

	// Solve Inverse Kinematics
	double timeout = 5.0;
	bool found_ik1 = kinematic_state->setFromIK(joint_model_group, pose1, timeout);
	std::vector<double> joint_values1, joint_values2;
	if (found_ik1) {
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values1);
		for (std::size_t i = 0; i < kuka_msg.joint_names.size(); i++) {
			ROS_INFO("IK Solution, Point1, Joint %s: %f", kuka_msg.joint_names[i].c_str(), joint_values1[i]);
		}
	}
	bool found_ik2 = kinematic_state->setFromIK(joint_model_group, pose2, timeout);
	if (found_ik2) {
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values2);
		for (std::size_t i = 0; i < kuka_msg.joint_names.size(); i++) {
			ROS_INFO("IK Solution, Point2, Joint %s: %f", kuka_msg.joint_names[i].c_str(), joint_values2[i]);
		}
	}

	if (found_ik1 && found_ik2) {
		vector<TrapezoidSingleJointTrajectory*> joint_trajectories;
		for (int j = 0; j < 7; ++j) {
			double q1 = joint_values1[j];
			double q2 = joint_values2[j];
			double qdc, t1, t2, tau;
			qdc = 0.5;
			t1 = 0.0;
			t2 = 1.0;
			tau = 0.25;
			TrapezoidSingleJointTrajectory* trajectory = new TrapezoidSingleJointTrajectory(q1, q2, t1, t2, tau, qdc);
			trajectory->computeTrajectory(60);
			joint_trajectories.push_back(trajectory);
		}

		int waypoints_num = joint_trajectories.at(0)->position_waypoints.size();
		// int waypoints_num = 5;
		kuka_msg.points.resize(waypoints_num);
		for (int k = 0; k < waypoints_num; ++k) {
			kuka_msg.points[k].positions.resize(kuka_msg.joint_names.size());
			kuka_msg.points[k].velocities.resize(kuka_msg.joint_names.size());
			kuka_msg.points[k].accelerations.resize(kuka_msg.joint_names.size());
			kuka_msg.points[k].effort.resize(kuka_msg.joint_names.size());

			for (int j = 0; j < 7; ++j) {
				kuka_msg.points[k].positions[j] = joint_trajectories.at(j)->position_waypoints.at(k);
				kuka_msg.points[k].velocities[j] = joint_trajectories.at(j)->velocity_waypoints.at(k);
				kuka_msg.points[k].accelerations[j] = joint_trajectories.at(j)->acceleration_waypoints.at(k);
				// kuka_msg.points[k].accelerations[j] = 0.0;
				kuka_msg.points[k].effort[j] = 0.0;
			}
			// To be reached 1 second after starting along the trajectory
			// kuka_msg.points[k].time_from_start = ros::Duration(joint_trajectories.at(0)->time_waypoints.at(k));
			kuka_msg.points[k].time_from_start = ros::Duration(k+1);
		}

		kuka_publisher.publish(kuka_msg);

	} else {
		ROS_INFO("Did not find IK solution");
	}


	ros::shutdown();
	return 0;
}

