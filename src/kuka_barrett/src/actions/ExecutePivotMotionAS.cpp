#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/ExecutePivotMotionAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"
#include "trajectory/LineSegTrajectory.h"
#include "kinematics/Pose.h"
#include <trajectory_msgs/JointTrajectory.h>
#include "kinematics/utils.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <chrono>


using namespace std::chrono;
using namespace std;
namespace rvt = rviz_visual_tools;

typedef geometry_msgs::PoseWithCovarianceStamped covPose;

void fulcrumFrameUpdate(const covPose::ConstPtr &msg) {
	// TODO
}

class ExecutePivotMotionAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::ExecutePivotMotionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::ExecutePivotMotionFeedback feedback_;
  kuka_barrett::ExecutePivotMotionResult result_;

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

  ExecutePivotMotionAS(std::string name) :
    as_(nh_, name, boost::bind(&ExecutePivotMotionAS::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~ExecutePivotMotionAS(void) {}

  void executeCB(const kuka_barrett::ExecutePivotMotionGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    vector<geometry_msgs::Pose> path;

    // start executing the action
    ROS_INFO("Executing ExecutePivotMotion action, ASSUMING TOOL IS ALREADY INSERTED");
    
    boost::shared_ptr<covPose const> sharedPoseMsg;
    covPose pose_msg;
    string topic = "fulcrum/estimated/frame" + std::to_string(goal->trocar_id);
    sharedPoseMsg = ros::topic::waitForMessage<covPose>(topic, nh_);
    if(sharedPoseMsg != nullptr){
      pose_msg = *sharedPoseMsg;
    }
		ROS_INFO("Received trocar pose");

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

    TrajectoryExecution traj = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, nh_);

    // Start measuring time duration of the trajectory generation
	  auto traj_time_begin = high_resolution_clock::now();

    Eigen::Vector3f start, end;
    // Initialize vector with known values https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html
    // values are given in x, y, z order
    // points that define a line segment that is invariant under the fulcrum transformation
    // start << 0.0, 0.0, -0.1; // Coordinates of start point of the line segment
    // end << 0.0, 0.0, -0.3; // Coordinates of start point of the line segment
    start << 0.0, 0.000001, -0.1; // Coordinates of start point of the line segment
    end << 0.0, 0.1, -0.1; // Coordinates of start point of the line segment
    LineSegTrajectory* lineSegTrajectory = new LineSegTrajectory(start, end);
    vector<geometry_msgs::Pose> line_seg_waypoints = lineSegTrajectory->getCartesianWaypoints(10);
    vector<geometry_msgs::Pose> transformed_waypoints = fulcrumEffectTransformation(line_seg_waypoints, 0.4);

    tf2::Matrix3x3 identity3x3 = tf2::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
    vector<geometry_msgs::Pose> line_seg_waypoints2, transformed_waypoints2, transformed_waypoints3;
    for (auto line_seg_pose: transformed_waypoints) {
      geometry_msgs::Pose new_pose;
      auto cp = line_seg_pose.position;
      double px = line_seg_pose.position.x;
      double py = line_seg_pose.position.y;
      double pz = line_seg_pose.position.z;
      tf2::Vector3 dxa = tf2::Vector3(px, py, pz);
      double th = atan2(sqrt(px*px + py*py), pz);
      double phi = atan2(py, px);
      tf2::Quaternion rot2 = tf2::Quaternion();
      rot2.setRPY(M_PI-phi, 0, 0);
      tf2::Quaternion rot3 = tf2::Quaternion();
      rot3.setRPY(0, M_PI_2, 0);
      tf2::Quaternion rot4 = tf2::Quaternion();
      rot4.setRPY(0, 0, M_PI_2+th);
      tf2::Transform tfa = tf2::Transform(qa, pa) * tf2::Transform(rot2, dxa) * tf2::Transform(rot3, tf2::Vector3(0, 0, 0)) * tf2::Transform(rot4, tf2::Vector3(0, 0, 0));
      tf2::toMsg(tfa, new_pose);
      transformed_waypoints2.push_back(new_pose);
    }

    for (auto line_seg_pose: transformed_waypoints) {
      geometry_msgs::Pose new_pose;
      auto cp = line_seg_pose.position;
      double px = line_seg_pose.position.x;
      double py = line_seg_pose.position.y;
      double pz = line_seg_pose.position.z;
      tf2::Vector3 dxa = tf2::Vector3(px, py, pz);
      double th = atan2(sqrt(px*px + py*py), pz);
      double phi = atan2(py, px);
      // double xx = cos(th)*cos(phi); double xy = cos(th)*sin(phi); double xz = -sin(th);
      // double yx = -sin(phi); double yy = cos(phi); double yz = 0;
      // double zx = sin(th)*cos(phi); double zy = sin(th)*sin(phi); double zz = cos(th);
      // tf2::Matrix3x3 orientation = tf2::Matrix3x3(xx, xy, xz, yx, yy, yz, zx, zy, zz);
      tf2::Quaternion rot2 = tf2::Quaternion();
      rot2.setRPY(M_PI-phi, 0, 0);
      tf2::Quaternion rot3 = tf2::Quaternion();
      rot3.setRPY(0, M_PI_2, 0);
      tf2::Quaternion rot4 = tf2::Quaternion();
      rot4.setRPY(0, 0, M_PI_2+th);
      tf2::Transform tfa = tf2::Transform(qa, pa) * tf2::Transform(rot2, dxa) * tf2::Transform(rot3, tf2::Vector3(0, 0, 0)) * tf2::Transform(rot4, tf2::Vector3(0, 0, 0)) * tf2::Transform(identity3x3, tf2::Vector3(-0.05, 0, -0.094 - 0.022));
      tf2::toMsg(tfa, new_pose);
      transformed_waypoints3.push_back(new_pose);
    }

    for (auto line_seg_pose: line_seg_waypoints) {
      geometry_msgs::Pose new_pose;
      auto cp = line_seg_pose.position;
      double px = line_seg_pose.position.x;
      double py = line_seg_pose.position.y;
      double pz = line_seg_pose.position.z;
      tf2::Vector3 dxa = tf2::Vector3(px, py, pz);
      double th = atan2(sqrt(px*px + py*py), pz);
      double phi = atan2(py, px);
      tf2::Quaternion rot2 = tf2::Quaternion();
      rot2.setRPY(M_PI-phi, 0, 0);
      tf2::Quaternion rot3 = tf2::Quaternion();
      rot3.setRPY(0, M_PI_2, 0);
      tf2::Quaternion rot4 = tf2::Quaternion();
      rot4.setRPY(0, 0, -M_PI_2-th);
      tf2::Transform tfa = tf2::Transform(qa, pa) * tf2::Transform(rot2, dxa) * tf2::Transform(rot3, tf2::Vector3(0, 0, 0)) * tf2::Transform(rot4, tf2::Vector3(0, 0, 0));
      tf2::toMsg(tfa, new_pose);
      line_seg_waypoints2.push_back(new_pose);
    }

    auto traj_time_end = high_resolution_clock::now();;
    auto traj_duration = duration_cast<microseconds>(traj_time_end - traj_time_begin);
    ROS_INFO("Line segment trajectory generation calculated in %ld microseconds", traj_duration.count());

    // Path to circle
    std::vector<geometry_msgs::Pose> path3;
    path3.push_back(trajStartPose);
    path3.push_back(transformed_waypoints3.at(0));

    traj.visualizeCartesianPath(line_seg_waypoints2, "Original taskspace line segment trajectory", true);
    traj.visualizeCartesianPath(transformed_waypoints2, "Fulcrum transformed line segment trajectory", true);

    // traj.executeCartesianPath(path3, "Path approaching line segment start");
    traj.executeCartesianPath(transformed_waypoints3, "Line Segment transformed Trajectory", true);

    ros::Duration(1).sleep();

    // Plan reverse execution of trajectory
    // reverse the trajectory array
    reverse(transformed_waypoints3.begin(), transformed_waypoints3.end());
    // Erase first waypoint (bacuase robot is already there) so that the error:
    // "Trajectory message contains waypoints that are not strictly increasing in time." is fixed
    transformed_waypoints3.erase(transformed_waypoints3.begin());
    traj.executeCartesianPath(transformed_waypoints3, "Reverse Line Segment transformed Trajectory", false);

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
  ros::init(argc, argv, "ExecutePivotMotion");

  ROS_INFO("Started ExecutePivotMotion Action Server");

  ExecutePivotMotionAS placeSurgicalTool("ExecutePivotMotion");
  ros::spin();

  return 0;
}