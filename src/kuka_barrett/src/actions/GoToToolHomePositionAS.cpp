#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/GoToToolHomePositionAction.h>
#include "kinematics/TrajectoryExecution.h"


class GoToToolHomePositionAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::GoToToolHomePositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::GoToToolHomePositionFeedback feedback_;
  kuka_barrett::GoToToolHomePositionResult result_;

  const std::string PLANNING_GROUP = "iiwa_arm";
  double pos_tolerance = 0.000005;
  double orient_tolerance = 0.000005;
  int plan_time_sec = 5;
  bool replanning = true;
  int plan_attempts = 6;
  const string base_frame = "world";
  const string plannerId = "RRTConnect";

public:

  GoToToolHomePositionAS(std::string name) :
    as_(nh_, name, boost::bind(&GoToToolHomePositionAS::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~GoToToolHomePositionAS(void) {}

  void executeCB(const kuka_barrett::GoToToolHomePositionGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // start executing the action
    ROS_INFO("Executing GoToToolHomePosition action");
    TrajectoryExecution traj = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, nh_, plannerId);
    traj.moveToTarget(getPoseFromPathPoint({0.2, -0.5, 1.3, M_PI, 0, -M_PI_2}));

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
  ros::init(argc, argv, "GoToToolHomePosition");

  ROS_INFO("Started GoToToolHomePosition Action Server");

  GoToToolHomePositionAS goToToolHomePosition("GoToToolHomePosition");
  ros::spin();

  return 0;
}