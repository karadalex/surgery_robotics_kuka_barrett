#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/PickSurgicalToolAction.h>
#include "kinematics/TrajectoryExecution.h"


class PickSurgicalToolAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::PickSurgicalToolAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::PickSurgicalToolFeedback feedback_;
  kuka_barrett::PickSurgicalToolResult result_;

  const std::string PLANNING_GROUP = "iiwa_arm";
  double pos_tolerance = 0.000005;
  double orient_tolerance = 0.000005;
  int plan_time_sec = 5;
  bool replanning = true;
  int plan_attempts = 6;
  const string base_frame = "world";
  const string plannerId = "RRTConnect";

public:

  PickSurgicalToolAS(std::string name) :
    as_(nh_, name, boost::bind(&PickSurgicalToolAS::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~PickSurgicalToolAS(void) {}

  void executeCB(const kuka_barrett::PickSurgicalToolGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    vector<geometry_msgs::Pose> path;

    // start executing the action
    ROS_INFO("Executing PickSurgicalTool action");
    TrajectoryExecution traj = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, nh_, plannerId);
    path.push_back(getPoseFromPathPoint({0.02, -0.48, 1.35, M_PI, 0, -M_PI_2}));
    path.push_back(getPoseFromPathPoint({0.02, -0.48, 1.20, M_PI, 0, -M_PI_2}));
    traj.executeCartesianPath(path, "picking pipeline", false);

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
  ros::init(argc, argv, "PickSurgicalTool");

  ROS_INFO("Started PickSurgicalTool Action Server");

  PickSurgicalToolAS pickSurgicalTool("PickSurgicalTool");
  ros::spin();

  return 0;
}