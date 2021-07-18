//
// Created by karadalex on 23/03/21.
//

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"
#include "trajectory/CircleTrajectory.h"
#include "kinematics/Pose.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;


using namespace std;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner3f");
	Client client("follow_joint_trajectory", true); // true -> don't need ros::spin()
	client.waitForServer();
	control_msgs::FollowJointTrajectoryGoal goal;

	// Fill in goal here
	client.sendGoal(goal);
	client.waitForResult(ros::Duration(5.0));
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Trajectory completed");
	printf("Current State: %s\n", client.getState().toString().c_str());

	return 0;
}


