#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/PlaceSurgicalToolAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"
#include "kinematics/Pose.h"
#include <trajectory_msgs/JointTrajectory.h>
#include "kinematics/utils.h"
#include <moveit_visual_tools/moveit_visual_tools.h>


typedef geometry_msgs::PoseWithCovarianceStamped covPose;

void fulcrumFrameUpdate(const covPose::ConstPtr &msg) {
	// TODO
}

class PlaceSurgicalToolAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::PlaceSurgicalToolAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::PlaceSurgicalToolFeedback feedback_;
  kuka_barrett::PlaceSurgicalToolResult result_;

  const std::string PLANNING_GROUP = "iiwa_arm";
	const std::string GRIPPER_PLANNING_GROUP = "barrett_group";
  double pos_tolerance = 0.000005;
  double orient_tolerance = 0.000005;
  int plan_time_sec = 5;
  bool replanning = true;
  int plan_attempts = 6;
  const string base_frame = "world";
  const string plannerId = "RRTConnect";

public:

  PlaceSurgicalToolAS(std::string name) :
    as_(nh_, name, boost::bind(&PlaceSurgicalToolAS::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~PlaceSurgicalToolAS(void) {}

  void executeCB(const kuka_barrett::PlaceSurgicalToolGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    vector<geometry_msgs::Pose> path;

    // start executing the action
    ROS_INFO("Executing PlaceSurgicalTool action");
    
    boost::shared_ptr<covPose const> sharedPoseMsg;
    covPose pose_msg;
    string topic = "fulcrum/estimated/frame" + std::to_string(goal->trocar_id);
    sharedPoseMsg = ros::topic::waitForMessage<covPose>(topic, nh_);
    if(sharedPoseMsg != nullptr){
      pose_msg = *sharedPoseMsg;
    }
		ROS_INFO("Received trocar pose");

    TrajectoryExecution traj = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, nh_);

    // X Y Z Roll Pitch Yaw
    vector<vector<float>> preparation_path;
    // TCP pose around home position, such that the robot arm starts in an elbow-up configuration
    preparation_path.push_back({0.1, 0.087249, 1.8, 2.952052, 1.311528, -1.750799});
    traj.moveToTarget(getPoseFromPathPoint(preparation_path.at(0)), "elbow-up start pose");

    std::vector<geometry_msgs::Pose> path1;
    preparation_path.push_back({0.1, 0.087249, 1.953192, 2.952052, 1.311528, -1.750799});
    path1.push_back(getPoseFromPathPoint(preparation_path.at(0)));
    path1.push_back(getPoseFromPathPoint(preparation_path.at(1)));
    traj.executeCartesianPath(path1, "elbow-up preparation path");

    // TCP pose for point above fulcrum 1
    geometry_msgs::Pose trajStartPose;
    auto _p = pose_msg.pose.pose.position;
    tf2::Vector3 pa = tf2::Vector3(_p.x, _p.y, _p.z);
    auto _q = pose_msg.pose.pose.orientation;
    tf2::Quaternion qa = tf2::Quaternion(_q.x, _q.y, _q.z, _q.w);
    tf2::Quaternion rot1 = tf2::Quaternion();
    rot1.setRPY(0, M_PI_2, 0);
    tf2::Quaternion rot2 = tf2::Quaternion();
    rot2.setRPY(0, 0, M_PI_2);
    tf2::Vector3 dxa = tf2::Vector3(-0.5 - 0.05, 0, - 0.094 - 0.022);
    tf2::Transform Td = tf2::Transform(tf2::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), dxa);
    tf2::Transform tfa = tf2::Transform(qa, pa) * tf2::Transform(rot1, tf2::Vector3(0, 0, 0)) * Td;
    tf2::toMsg(tfa, trajStartPose);

    path1.clear();
    path1.push_back(getPoseFromPathPoint(preparation_path.at(1)));
    path1.push_back(trajStartPose);
    traj.executeCartesianPath(path1, "movement towards above fulcrum point");

    // Approaching Fulcrum point 2 - Insertion motion Cartesian path
    // Move in a line segment while approaching fulcrum point and entering body
    geometry_msgs::Pose fulcrumAbovePose1 = path1.at(path1.size()-1); // Start insertion trajectory from last target point of previous trajectory
    std::vector<geometry_msgs::Pose> path2;
    // path2.push_back(fulcrumAbovePose1);
    geometry_msgs::Pose fulcrumInsertedPose1;
    tf2::Vector3 dxb = tf2::Vector3(0.2, 0, 0);
    tf2::Transform Tdb = tf2::Transform(tf2::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), dxb);
    tf2::Transform tfb = tfa * Tdb;
    tf2::toMsg(tfb, fulcrumInsertedPose1);
    path2.push_back(fulcrumInsertedPose1);
    traj.executeCartesianPath(path2, "insertion movement");

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
  ros::init(argc, argv, "PlaceSurgicalTool");

  ROS_INFO("Started PlaceSurgicalTool Action Server");

  PlaceSurgicalToolAS placeSurgicalTool("PlaceSurgicalTool");
  ros::spin();

  return 0;
}