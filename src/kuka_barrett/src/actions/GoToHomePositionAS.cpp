#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/GoToHomePositionAction.h>
#include "kinematics/TrajectoryExecution.h"
#include <moveit/move_group_interface/move_group_interface.h>


class GoToHomePositionAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::GoToHomePositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::GoToHomePositionFeedback feedback_;
  kuka_barrett::GoToHomePositionResult result_;

  const std::string PLANNING_GROUP = "iiwa_arm";

public:

  GoToHomePositionAS(std::string name) :
    as_(nh_, name, boost::bind(&GoToHomePositionAS::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~GoToHomePositionAS(void) {}

  void executeCB(const kuka_barrett::GoToHomePositionGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // start executing the action
    ROS_INFO("Executing GoToHomePosition action");
		moveit::planning_interface::MoveGroupInterface move_group = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
		const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);;
		moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
		std::vector<double> joint_group_positions;
		current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		for (int j = 0; j < joint_group_positions.size(); ++j) {
			joint_group_positions[j] = 0;
		}
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		move_group.execute(my_plan);

    // publish the feedback
    feedback_.percent_complete = 100.0f;
    as_.publishFeedback(feedback_);

    if(success) {
      // set the action state to succeeded
      result_.result = 1;
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "GoToHomePosition");

  ROS_INFO("Started GoToHomePosition Action Server");

  GoToHomePositionAS goToHomePosition("GoToHomePosition");
  ros::spin();

  return 0;
}