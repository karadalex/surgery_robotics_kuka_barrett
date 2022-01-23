#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <kuka_barrett/ScanToolTableVisualServoAction.h>
#include "kinematics/TrajectoryExecution.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


class ScanToolTableVisualServoAS {

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuka_barrett::ScanToolTableVisualServoAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  kuka_barrett::ScanToolTableVisualServoFeedback feedback_;
  kuka_barrett::ScanToolTableVisualServoResult result_;

  ros::Subscriber sub;

  const std::string PLANNING_GROUP = "iiwa_arm";
  double pos_tolerance = 0.000005;
  double orient_tolerance = 0.000005;
  int plan_time_sec = 5;
  bool replanning = true;
  int plan_attempts = 6;
  const string base_frame = "world";
  const string plannerId = "RRTConnect";

  float tool_error_threshold;
  vector<vector<float>> path;

public:

  ScanToolTableVisualServoAS(std::string name) :
    as_(nh_, name, boost::bind(&ScanToolTableVisualServoAS::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~ScanToolTableVisualServoAS(void) {}

  void executeCB(const kuka_barrett::ScanToolTableVisualServoGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    tool_error_threshold = goal->tool_error_threshold;
    float error = 100.0;
    TrajectoryExecution traj = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, nh_, plannerId);
    // Start path from tool homeposition (from GoToToolHomePositionAS action server)
    path.push_back({0.209815, -0.770712, 1.363938, M_PI, 0, -M_PI_2});

     // start executing the action
    ROS_INFO("Executing ScanToolTableVisualServo action");

    while (error > tool_error_threshold) {
      boost::shared_ptr<geometry_msgs::Twist const> velMsg;
      geometry_msgs::Twist vel;
      velMsg = ros::topic::waitForMessage<geometry_msgs::Twist>("kuka_barrett/cmd_vel", nh_);
      if(velMsg != nullptr){
        vel = *velMsg;
      }

			vector<float> current_pose = path.at(path.size()-1);
			current_pose[0] += vel.linear.x;
			current_pose[1] += vel.linear.y;
			current_pose[2] += vel.linear.z;
			current_pose[3] += vel.angular.x;
			current_pose[4] += vel.angular.y;
			current_pose[5] += vel.angular.z;
      std::cout << vel.linear.x << "," << vel.linear.y << std::endl;
      vector<geometry_msgs::Pose> local_path;
      local_path.push_back(getPoseFromPathPoint(current_pose));
			traj.executeCartesianPath(local_path, "visual servoing");

			if (path.size() > 100) {
				// If path has more than 100 states stored in it, then do some cleanup to free up some memory
				path.clear();
				ROS_INFO_NAMED("ScanToolTableVisualServo", "Path cleared");
			}

			// save new pose in global path as well
			path.push_back(current_pose);

      // publish the feedback
      boost::shared_ptr<std_msgs::Float32 const> servoErrorMsg;
			std_msgs::Float32 servoError;
			servoErrorMsg = ros::topic::waitForMessage<std_msgs::Float32>("kuka_barrett/visual_servo/error", nh_);
      if(servoErrorMsg != nullptr){
				servoError = *servoErrorMsg;
        error = servoError.data;
      }
      feedback_.servo_error = error;
      as_.publishFeedback(feedback_);
      ROS_INFO("ScanToolTableVisualServo servo error: %.6f", error);
    }

    if (success) {
      // set the action state to succeeded
      result_.result = 1;
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ScanToolTableVisualServo");

  ROS_INFO("Started ScanToolTableVisualServo Action Server");

  ScanToolTableVisualServoAS scanToolTableVisualServo("ScanToolTableVisualServo");
  ros::spin();

  return 0;
}