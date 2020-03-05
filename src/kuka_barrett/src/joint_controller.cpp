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
  ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/bh_j11_position_controller/command",1000);
  ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/bh_j12_position_controller/command",1000);
  ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("/bh_j13_position_controller/command",1000);
  ros::Publisher pub4 = nh.advertise<std_msgs::Float64>("/bh_j21_position_controller/command",1000);
  ros::Publisher pub5 = nh.advertise<std_msgs::Float64>("/bh_j22_position_controller/command",1000);
  ros::Publisher pub6 = nh.advertise<std_msgs::Float64>("/bh_j23_position_controller/command",1000);
  ros::Publisher pub7 = nh.advertise<std_msgs::Float64>("/bh_j31_position_controller/command",1000);
  ros::Publisher pub8 = nh.advertise<std_msgs::Float64>("/bh_j32_position_controller/command",1000);
  ros::Publisher pub9 = nh.advertise<std_msgs::Float64>("/bh_j33_position_controller/command",1000);

  // KUKA iiwa LBR 14 arm controller publisher
  ros::Publisher pub10 = nh.advertise<trajectory_msgs::JointTrajectory > ("/arm_controller/command", 1000);

  ros::Rate loop_rate(1);

  // Barrett hand command values
  std_msgs::Float64 msg1;  msg1.data = 0.2f;
  std_msgs::Float64 msg2;  msg2.data = 1.7f;
  std_msgs::Float64 msg3;  msg3.data = 2.0f;
  std_msgs::Float64 msg4;  msg4.data = 0.9f;
  std_msgs::Float64 msg5;  msg5.data = 1.7f;
  std_msgs::Float64 msg6;  msg6.data = 2.0f;
  std_msgs::Float64 msg7;  msg7.data = 0.9f;
  std_msgs::Float64 msg8;  msg8.data = 1.7f;
  std_msgs::Float64 msg9;  msg9.data = 2.0f;

  // Build iiwa arm joint trajectory message
  trajectory_msgs::JointTrajectory msg10;
  msg10.header.seq = 0;
  msg10.header.stamp.sec = 0;
  msg10.header.stamp.nsec = 0;
  msg10.header.frame_id = "";

  msg10.joint_names.push_back("iiwa_joint_1");
  msg10.joint_names.push_back("iiwa_joint_2");
  msg10.joint_names.push_back("iiwa_joint_3");
  msg10.joint_names.push_back("iiwa_joint_4");
  msg10.joint_names.push_back("iiwa_joint_5");
  msg10.joint_names.push_back("iiwa_joint_6");
  msg10.joint_names.push_back("iiwa_joint_7");

  msg10.points.resize(1);

  // Solve Inverse Kinematics
	Pose* M_U_TCP = new Pose(0,0,2.18,0,0,0);
	Iiwa14Inv* iiwa14Inv = new Iiwa14Inv(M_U_TCP);
	Matrix::printMatrix(M_U_TCP->pose, "M_U_TCP");


	// Send first angle-position point
  int ind = 0;
  msg10.points[ind].positions.resize(7);
	msg10.points[ind].positions[0] = 0.0;
	msg10.points[ind].positions[1] = iiwa14Inv->th1[0];
	cout << "th1=" << iiwa14Inv->th1[1] << endl;
	msg10.points[ind].positions[2] = iiwa14Inv->th2[0];
	msg10.points[ind].positions[3] = iiwa14Inv->th3[0];
	msg10.points[ind].positions[4] = iiwa14Inv->th4[0];
	msg10.points[ind].positions[5] = iiwa14Inv->th5[0];
	msg10.points[ind].positions[6] = iiwa14Inv->th6[0];

  // Home position (all angles 0)
//	 for (int i = 0; i < 7; i++) {
//	 	msg10.points[ind].positions[i] = 0.0f;
//	 }

  // Velocities
  msg10.points[ind].velocities.resize(7);
  msg10.points[ind].effort.resize(7);
	for (size_t j = 0; j < 7; ++j)
	{
		msg10.points[ind].velocities[j]=0.0;
		msg10.points[ind].effort[j] = 0.0;
	}
  // To be reached 1 second after starting along the trajectory
  msg10.points[ind].time_from_start = ros::Duration(2.0);


  while(ros::ok()){
    pub1.publish(msg1);
    pub2.publish(msg2);
    pub3.publish(msg3);
    pub4.publish(msg4);
    pub5.publish(msg5);
    pub6.publish(msg6);
    pub7.publish(msg7);
    pub8.publish(msg8);
    pub9.publish(msg9);
    pub10.publish(msg10);

    ros::spinOnce();
    loop_rate.sleep();
    }
  return 0;
}




