#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "Eigen/Dense"


using namespace Eigen;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jacobian_node");
	ros::NodeHandle n;

	ros::Publisher jac_pub = n.advertise<std_msgs::Float64MultiArray>("jacobian_state", 1000);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		// TODO: Calculate Jacobian
		// MatrixXf jacobian = MatrixXf::Identity(6,7);
		Matrix<float,Dynamic,Dynamic,RowMajor> jacobian = MatrixXf::Random(6,7);  // Caution: must use row-major, Eigen's default is column major
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

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}