#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "Eigen/Dense"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>
#include <string>


using namespace Eigen;

class JacobianNode {
private:
	ros::NodeHandle n;
	ros::Publisher jac_pub;
	ros::Subscriber sub;

	robot_model_loader::RobotModelLoader* robot_model_loader;
	moveit::core::RobotModelPtr kinematic_model;
	std::vector<double> joint_values;


	void jointStateCallback(const sensor_msgs::JointState &msg) {
		const double* all_joint_values = msg.position.data();
		// Joint names are the following with the following order
		// name: [bh_j11_joint, bh_j12_joint, bh_j13_joint, bh_j21_joint, bh_j22_joint, bh_j23_joint,
		//   bh_j32_joint, bh_j33_joint, iiwa_joint_1, iiwa_joint_2, iiwa_joint_3, iiwa_joint_4,
		//   iiwa_joint_5, iiwa_joint_6, iiwa_joint_7]
		for (int i = 8; i < 15; i++) {
			joint_values.at(i-8) = all_joint_values[i];
			ROS_INFO("Joint %d state: %f", i-7, joint_values.at(i-8));
		}

	}

public:
	JacobianNode(ros::NodeHandle _nh) : n(_nh) {
		jac_pub = n.advertise<std_msgs::Float64MultiArray>("jacobian_state", 1000);

		// Joints state subscriber
		sub = n.subscribe("joint_states", 1000, &JacobianNode::jointStateCallback, this);

		// Construct a :moveit_core:`RobotModel` based on loaded robot_description
		robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
		kinematic_model = robot_model_loader->getModel();
		ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

		joint_values = std::vector<double>(7, 0.0);
	}

	void jacobianCalculation() {
		// TODO: Calculate Jacobian
		robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
		const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("iiwa_arm");
		const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

		// Print joint values
		// std::vector<double> joint_values;
		// kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}

		// Get forward kinematics
		// kinematic_state->setp
		const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("iiwa_link_7");
		/* Print end-effector pose. Remember that this is in the model frame */
		ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
		ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

		// Get jacobian
		Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
		Eigen::MatrixXd jacobian;
		kinematic_state->getJacobian(joint_model_group,
																 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
																 reference_point_position, jacobian);
		std::cout << jacobian << "\n" << std::endl;

		// Convert Jacobian to 1D array
		std::vector<double> jacobian_array(jacobian.data(), jacobian.data() + jacobian.rows() * jacobian.cols());

		// Build ROS Message
		std_msgs::Float64MultiArray jac_msg;
		// set up dimensions
		// multiarray(i,j) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j] = data[data_offset + 7*i + 1*j]kill
		jac_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
		jac_msg.layout.dim[0].label = "rows";
		jac_msg.layout.dim[0].size = 6;
		jac_msg.layout.dim[0].stride = 7;
		jac_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
		jac_msg.layout.dim[1].label = "columns";
		jac_msg.layout.dim[1].size = 7;
		jac_msg.layout.dim[1].stride = 1;
		// copy in the data
		jac_msg.data.clear();
		jac_msg.data.insert(jac_msg.data.end(), jacobian_array.begin(), jacobian_array.end());

		jac_pub.publish(jac_msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jacobian_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	JacobianNode* jn = new JacobianNode(n);

	while (ros::ok())
	{
		jn->jacobianCalculation();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}