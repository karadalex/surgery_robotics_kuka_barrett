#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "kinematics/Pose.h"
#include "kinematics/Iiwa14Inv.h"
#include "custom_math/Matrix.h"

using namespace std;

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "joint_controller");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	// Barrett hand publishers
	vector<ros::Publisher> bh_fingers_publishers;
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j11_position_controller/command",1000));
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j12_position_controller/command",1000));
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j13_position_controller/command",1000));
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j21_position_controller/command",1000));
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j22_position_controller/command",1000));
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j23_position_controller/command",1000));
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j32_position_controller/command",1000));
	bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j33_position_controller/command",1000));

	// Barrett hand command values
	vector<std_msgs::Float64> bh_msgs;
	vector<float> bh_angles;
	bh_angles = {0.2f, 1.7f, 2.0f, 0.9f, 1.7f, 2.0f, 1.7f, 2.0f};
	bh_angles.resize(9, 0);
	for (int i = 0; i < 8; ++i) {
		std_msgs::Float64 msg; msg.data = bh_angles[i];
		bh_msgs.push_back(msg);
	}

	// KUKA iiwa LBR 14 arm controller publisher
	ros::Publisher kuka_publisher = nh.advertise<trajectory_msgs::JointTrajectory > ("/arm_controller/command", 1000);

	// Create path and solve Inverse Kinematics for each point in path
	vector<Pose*> path;
	vector<Iiwa14Inv*> pathSolutions;

	path.push_back(new Pose(0, 0.68, 1.7, M_PI, 0, M_PI_2));
	path.push_back(new Pose(0, 0.68, 1.5, M_PI, 0, M_PI_2));
//	path.push_back(new Pose(0, -0.68, 1.4, M_PI, 0, M_PI_2));
//	path.push_back(new Pose(0, -0.68, 1.5, M_PI, 0, M_PI_2));
//	path.push_back(new Pose(0.0001,0.01,2.066,0,0,0));
//	path.push_back(new Pose(0.0001,0.01,2.166,0, 0, 0));
//	path.push_back(new Pose(0.0001,0.0001,2.425,0.0001, 0.0001, 0.0001));

	// Build iiwa arm joint trajectory message
	trajectory_msgs::JointTrajectory kuka_msg;
	kuka_msg.header.seq = 0;
	kuka_msg.header.stamp.sec = 0;
	kuka_msg.header.stamp.nsec = 0;
	kuka_msg.header.frame_id = "";

	kuka_msg.joint_names.push_back("iiwa_joint_1");
	kuka_msg.joint_names.push_back("iiwa_joint_2");
	kuka_msg.joint_names.push_back("iiwa_joint_3");
	kuka_msg.joint_names.push_back("iiwa_joint_4");
	kuka_msg.joint_names.push_back("iiwa_joint_5");
	kuka_msg.joint_names.push_back("iiwa_joint_6");
	kuka_msg.joint_names.push_back("iiwa_joint_7");

	kuka_msg.points.resize(path.size());


	int pathIndex = 0;
	while(ros::ok()){
		if (pathIndex < path.size()) {
			// for (int i = 0; i < 9; ++i) bh_fingers_publishers.at(i).publish(bh_msgs.at(i));

			Iiwa14Inv* pathSolution = new Iiwa14Inv(path.at(pathIndex));
			// pathSolution->validateSolution();
			pathSolutions.push_back(pathSolution);

			kuka_msg.points[pathIndex].positions.resize(7);
			vector<double> angles = pathSolutions.at(pathIndex)->solutionSet[5]; // Select first solution of the solution set
			pathSolutions.at(pathIndex)->validateSolution(angles);
			for (int i = 0; i < 7; ++i) {
				// vecf angles = {0, 0, 0, 0, 0, 0, 0};
				kuka_msg.points[pathIndex].positions[i] = angles[i];
			}

			// Velocities
			kuka_msg.points[pathIndex].velocities.resize(7);
			kuka_msg.points[pathIndex].effort.resize(7);
			for (int j = 0; j < 7; ++j)
			{
				kuka_msg.points[pathIndex].velocities[j]=0.0;
				kuka_msg.points[pathIndex].effort[j] = 0.0;
			}
			// To be reached 1 second after starting along the trajectory
			kuka_msg.points[pathIndex].time_from_start = ros::Duration(3*(pathIndex+1));

			kuka_publisher.publish(kuka_msg);

			pathIndex++;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}