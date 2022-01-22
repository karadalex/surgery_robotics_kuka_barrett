#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/GoToHomePositionAction.h>


class GoToHomePositionAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::GoToHomePositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::GoToHomePositionFeedback feedback_;
  kuka_barrett::GoToHomePositionResult result_;

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