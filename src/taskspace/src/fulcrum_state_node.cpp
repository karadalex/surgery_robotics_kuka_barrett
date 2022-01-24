//
// Created by karadalex on 2/1/22.
//

#include "ros/ros.h"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace Eigen;
using namespace std;

typedef geometry_msgs::PoseWithCovarianceStamped covPose;

std::vector<double> joint_values = std::vector<double>(7, 0.0);

using Line3 = Hyperplane<double, 3>;
using Vec3 = Vector3d;

Vec3 f2;

ros::Subscriber sub, sub_fulcrum;
ros::Publisher pub;

robot_model_loader::RobotModelLoader* _robot_model_loader;
moveit::core::RobotModelPtr kinematic_model;
const robot_state::JointModelGroup* joint_model_group;

void jointStateCallback(const sensor_msgs::JointState &msg) {
	moveit_visual_tools::MoveItVisualTools visual_tools = moveit_visual_tools::MoveItVisualTools("world");

	const double* all_joint_values = msg.position.data();
	const string* all_joint_names = msg.name.data();
	// Joint names are the following but may not be with the following order
	// so it's important not to mix the gripper joints with the arm joints that are the ones
	// needed in this node:
	// name: [bh_j11_joint, bh_j12_joint, bh_j13_joint, bh_j21_joint, bh_j22_joint, bh_j23_joint,
	//   bh_j32_joint, bh_j33_joint, iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4,
	//   iiwa_joint_5, iiwa_joint_6, iiwa_joint_7]
	for (int i = 0; i < 15; i++) {
		string joint_name = all_joint_names[i];
		if (joint_name.substr(0, 4) == "iiwa") {
			char joint_ind_chr = joint_name[joint_name.size() - 1];
			int joint_ind = atoi(&joint_ind_chr) - 1;
			joint_values.at(joint_ind) = all_joint_values[i];
		}
	}

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
	const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("iiwa_link_7");

	auto translation = end_effector_state.translation();
	// Take into consideration only poses away from home position where z=2.26
	if (translation.z() < 2.2) {
		double x = translation.x() + 0.094 + 0.022;
		double y = translation.y();
		// double z = translation.z() - 0.094 - 0.022 - 0.008;
		double z = translation.z();
		auto rotation = end_effector_state.rotation();
		double ix = rotation.coeff(0,0);
		double iy = rotation.coeff(1,0);
		double iz = rotation.coeff(2,0);

		Vec3 a(x,y,z);
		Vec3 b(x+ix, y+iy, z+iz);
		// Vec3 f2(0.529996, 0.059271, 1.398114);

		// Show calculated line in rviz, uncomment for debugging
		// TODO: Instead of comment toggling enable/disable with topic or a parameter
		// visual_tools.publishLine(a, b);
		// visual_tools.trigger();

		Line3 ab = Line3::Through(a, b);
		float xy_dist_error = ab.absDistance(f2);
		// std::cout << "Transformation:\n" << end_effector_state.matrix() << '\n';
		// std::cout << "Distance from Fulcrum2: " << xy_dist_error << '\n';
		ROS_DEBUG("Line distance from fulcrum point 2: %f", xy_dist_error);

		std_msgs::Float64 xy_dist_error_msg;
		xy_dist_error_msg.data = xy_dist_error;
		pub.publish(xy_dist_error_msg);
	}
}

void fulcrumFrameUpdate(const covPose::ConstPtr &msg) {
	// TODO
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fulcrum_state_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// Wait to first get the fulcrum reference frame 2
	// before the jointStateCallback above is executed
	boost::shared_ptr<covPose const> sharedPoseMsg;
	covPose pose_msg;
	sharedPoseMsg = ros::topic::waitForMessage<covPose>("fulcrum/estimated/frame2", n);
	if(sharedPoseMsg != nullptr){
		pose_msg = *sharedPoseMsg;
		auto fp = pose_msg.pose.pose.position;
		f2 = Vec3(fp.x, fp.y, fp.x);
	}

	_robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
	kinematic_model = _robot_model_loader->getModel();
	joint_model_group = kinematic_model->getJointModelGroup("iiwa_arm");

	sub = n.subscribe("joint_states", 1000, jointStateCallback);
	sub_fulcrum = n.subscribe("fulcrum/estimated/frame2", 1000, fulcrumFrameUpdate);
	pub = n.advertise<std_msgs::Float64>("/fulcrum/error", 5);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}