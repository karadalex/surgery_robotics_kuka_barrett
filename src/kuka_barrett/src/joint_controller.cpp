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

  // Barrett hand publishers
  // TODO: clean up code with std::vector for publishers (D.R.Y)
  vector<ros::Publisher> bh_fingers_publishers;
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j11_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j12_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j13_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j21_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j22_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j23_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j31_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j32_position_controller/command",1000));
  bh_fingers_publishers.push_back(nh.advertise<std_msgs::Float64>("/bh_j33_position_controller/command",1000));

  // KUKA iiwa LBR 14 arm controller publisher
  ros::Publisher kuka_publisher = nh.advertise<trajectory_msgs::JointTrajectory > ("/arm_controller/command", 1000);

  ros::Rate loop_rate(1);

  // Barrett hand command values
  vector<std_msgs::Float64> bh_msgs;
  vector<float> bh_angles;
  bh_angles = {0.2f, 1.7f, 2.0f, 0.9f, 1.7f, 2.0f, 0.9f, 1.7f, 2.0f};
	for (int i = 0; i < 9; ++i) {
		std_msgs::Float64 msg; msg.data = bh_angles[i];
		bh_msgs.push_back(msg);
	}

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

  kuka_msg.points.resize(1);

  // Solve Inverse Kinematics
	Pose* M_U_TCP = new Pose(0,0,2.166,0,0,0);
	Iiwa14Inv* iiwa14Inv = new Iiwa14Inv(M_U_TCP);
	Matrix::printMatrix(M_U_TCP->pose, "M_U_TCP");


	// Send first angle-position point
  int ind = 0;
  kuka_msg.points[ind].positions.resize(7);
	kuka_msg.points[ind].positions[0] = iiwa14Inv->th1[0];
	kuka_msg.points[ind].positions[1] = iiwa14Inv->th2[0];
	kuka_msg.points[ind].positions[2] = iiwa14Inv->th3[0];
	kuka_msg.points[ind].positions[3] = iiwa14Inv->th4[0];
	kuka_msg.points[ind].positions[4] = iiwa14Inv->th5[0];
	kuka_msg.points[ind].positions[5] = iiwa14Inv->th6[0];
	kuka_msg.points[ind].positions[6] = iiwa14Inv->th7[0];

  // Home position (all angles 0)
  // for (int i = 0; i < 7; i++) {
  //   kuka_msg.points[ind].positions[i] = 0.0f;
  // }

  // Velocities
  kuka_msg.points[ind].velocities.resize(7);
  kuka_msg.points[ind].effort.resize(7);
	for (size_t j = 0; j < 7; ++j)
	{
		kuka_msg.points[ind].velocities[j]=0.0;
		kuka_msg.points[ind].effort[j] = 0.0;
	}
  // To be reached 1 second after starting along the trajectory
  kuka_msg.points[ind].time_from_start = ros::Duration(2.0);


  while(ros::ok()){
		for (int i = 0; i < 9; ++i) bh_fingers_publishers.at(i).publish(bh_msgs.at(i));

    kuka_publisher.publish(kuka_msg);

    ros::spinOnce();
    loop_rate.sleep();
	}

  return 0;
}




