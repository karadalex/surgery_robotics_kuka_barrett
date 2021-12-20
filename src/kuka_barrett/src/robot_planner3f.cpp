//
// Created by karadalex on 23/03/21.
//

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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

	// Set end-effector state
	kinematic_state->setToRandomPositions(joint_model_group);
	const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("iiwa_link_ee");
	cout << end_effector_state.matrix() << endl;

	// Solve Inverse Kinematics
	double timeout = 0.1;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

	trajectory_msgs::JointTrajectory kuka_msg;
	kuka_msg.header.seq = 0;
	kuka_msg.header.stamp.sec = 0;
	kuka_msg.header.stamp.nsec = 0;
	kuka_msg.header.frame_id = "";
	kuka_msg.joint_names = joint_model_group->getVariableNames();
	kuka_msg.points.resize(1);

	// Now, we can print out the IK solution (if found):
	std::vector<double> joint_values;
	if (found_ik) {
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		kuka_msg.points[0].positions.resize(kuka_msg.joint_names.size());
		kuka_msg.points[0].velocities.resize(kuka_msg.joint_names.size());
		kuka_msg.points[0].effort.resize(kuka_msg.joint_names.size());
		for (int j = 0; j < 7; ++j) {
			kuka_msg.points[0].positions[j] = joint_values[j];
			kuka_msg.points[0].velocities[j] = 0.0;
			kuka_msg.points[0].effort[j] = 0.0;
		}
		// To be reached 1 second after starting along the trajectory
		kuka_msg.points[0].time_from_start = ros::Duration(3*(0+1));

		for (std::size_t i = 0; i < kuka_msg.joint_names.size(); ++i) {
			ROS_INFO("Joint %s: %f", kuka_msg.joint_names[i].c_str(), joint_values[i]);
		}

		// kuka_publisher.publish(kuka_msg);
	} else {
		ROS_INFO("Did not find IK solution");
	}

	kuka_publisher.publish(kuka_msg);


	ros::shutdown();
	return 0;
}


