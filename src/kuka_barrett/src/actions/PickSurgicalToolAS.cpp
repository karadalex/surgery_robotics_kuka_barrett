#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/PickSurgicalToolAction.h>
#include "kinematics/TrajectoryExecution.h"
#include <trajectory_msgs/JointTrajectory.h>


void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
	/* Add both finger joints of panda robot. */
	posture.joint_names.resize(8);
	posture.joint_names = {
					"bh_j11_joint", "bh_j12_joint", "bh_j13_joint",
					"bh_j21_joint", "bh_j22_joint", "bh_j23_joint",
					"bh_j32_joint", "bh_j33_joint"
	};

	/* Set them as closed. */
	posture.points.resize(1);
	posture.points[0].positions.resize(8);
  posture.points[0].positions = {0, 1.4, 1.0, 0, 1.4, 1.0, 1.4, 1.0};
	// for (int j = 0; j < posture.joint_names.size(); ++j)
	// 	posture.points[0].positions[j] = 0.7;
	posture.points[0].time_from_start = ros::Duration(0.5);
}
class PickSurgicalToolAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::PickSurgicalToolAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::PickSurgicalToolFeedback feedback_;
  kuka_barrett::PickSurgicalToolResult result_;

  const std::string PLANNING_GROUP = "iiwa_arm";
	const std::string GRIPPER_PLANNING_GROUP = "barrett_group";
  double pos_tolerance = 0.000005;
  double orient_tolerance = 0.000005;
  int plan_time_sec = 5;
  bool replanning = true;
  int plan_attempts = 6;
  const string base_frame = "world";
  const string plannerId = "RRTConnect";

	vector<string> gripper_joints = {
		"bh_j11_joint", "bh_j12_joint", "bh_j13_joint",
		"bh_j21_joint", "bh_j22_joint", "bh_j23_joint",
		"bh_j32_joint", "bh_j33_joint"
	};
	ros::Publisher barrett_publisher;

public:

  PickSurgicalToolAS(std::string name) :
    as_(nh_, name, boost::bind(&PickSurgicalToolAS::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
		barrett_publisher = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);
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
    path.push_back(getPoseFromPathPoint({0.02, -0.48, 1.25, M_PI, 0, -M_PI_2}));
    traj.executeCartesianPath(path, "picking pipeline", false);

    // Plan to close the gripper
		trajectory_msgs::JointTrajectory grasp;
		closedGripper(grasp);
		barrett_publisher.publish(grasp);

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