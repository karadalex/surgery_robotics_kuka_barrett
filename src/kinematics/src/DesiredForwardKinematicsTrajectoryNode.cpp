#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "Eigen/Dense"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include "kinematics/DesiredForwardKinematicsTrajectory.h"
#include <moveit_msgs/RobotTrajectory.h>


using namespace Eigen;

class DesiredForwardKinematicsTrajectoryNode {
private:
	ros::NodeHandle n;
	ros::Publisher traj_pub;
	ros::Subscriber sub;

	robot_model_loader::RobotModelLoader* robot_model_loader;
	moveit::core::RobotModelPtr kinematic_model;
	moveit_msgs::RobotTrajectory robot_joint_trajectory;


	void jointTrajectoryCallback(const moveit_msgs::RobotTrajectory &msg) {
		robot_joint_trajectory = msg;

		kinematicStateCalculation();
	}

public:
	DesiredForwardKinematicsTrajectoryNode(ros::NodeHandle _nh) : n(_nh) {
		// Publish the desired forward kinemtaics trajectory and latch it
		traj_pub = n.advertise<kinematics::DesiredForwardKinematicsTrajectory>("desired_robot_trajectory_fwd", 10, true);

		// Joints state subscriber
		sub = n.subscribe("desired_robot_trajectory", 1000, &DesiredForwardKinematicsTrajectoryNode::jointTrajectoryCallback, this);

		// Construct a :moveit_core:`RobotModel` based on loaded robot_description
		robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
		kinematic_model = robot_model_loader->getModel();
		ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	}

	void kinematicStateCalculation() {
		kinematics::DesiredForwardKinematicsTrajectory desired_traj_msg;

		robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
		const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("iiwa_arm");
		const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

		for (auto & point : robot_joint_trajectory.joint_trajectory.points) {
			std::vector<double> joint_values = point.positions;

			kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

			// Get forward kinematics
			const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("iiwa_link_7");
			// Print end-effector pose. Remember that this is in the model frame
			// ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
			// ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

			// Build ROS Message for the Kinematic Transformation
			// see also https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20(C%2B%2B)
			geometry_msgs::TransformStamped tf_msg;
			tf_msg.header.stamp = ros::Time::now();
			tf_msg.header.frame_id = "world";
			tf_msg.child_frame_id = "iiwa_link_7";
			tf_msg.transform.translation.x = end_effector_state.translation().x();
			tf_msg.transform.translation.y = end_effector_state.translation().y();
			tf_msg.transform.translation.z = end_effector_state.translation().z();
			tf2::Quaternion q;
			auto rot_mat = end_effector_state.rotation(); // auto-inference matrix type
			// TODO: Consider singularity case where roll=+-90degrees
			// TODO: DRY refactoring, this code also exists in KinematicStateNode
			const double pitch = atan2(rot_mat.coeff(1, 0), rot_mat.coeff(0, 0));
			const double sy = sqrt(rot_mat.coeff(0,0)*rot_mat.coeff(0,0) + rot_mat.coeff(1,0)*rot_mat.coeff(1,0));
			const double roll = atan2(-rot_mat.coeff(2,0), sy);
			const double yaw = atan2(rot_mat.coeff(2,1), rot_mat.coeff(2,2));
			q.setRPY(roll, pitch, yaw);
			tf_msg.transform.rotation.x = q.x();
			tf_msg.transform.rotation.y = q.y();
			tf_msg.transform.rotation.z = q.z();
			tf_msg.transform.rotation.w = q.w();

			desired_traj_msg.desired_fwd_traj.push_back(tf_msg);
		}

		traj_pub.publish(desired_traj_msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "desired_robot_trajectory_fwd_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	DesiredForwardKinematicsTrajectoryNode* jn = new DesiredForwardKinematicsTrajectoryNode(n);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}