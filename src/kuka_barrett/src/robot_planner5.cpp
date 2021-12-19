/**
 * Robot Planner 5:
 * Simple Visual servoing
 *
 * Author: Alexios Karadimos
 */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include "kinematics/TrajectoryExecution.h"
#include "kinematics/BarrettInv.h"


using namespace std;

class ServoPlanner
{
private:
	// Setup Move group
	const std::string PLANNING_GROUP = "iiwa_arm";
	double pos_tolerance = 0.001;
	double orient_tolerance = 0.001;
	double plan_time_sec = 0.1;
	bool replanning = true;
	int plan_attempts = 2;
	const string base_frame = "world";
	TrajectoryExecution traj1 = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, ros::NodeHandle());
	bool initialPathExecuted = false;

	// X Y Z Roll Pitch Yaw
	vector<vector<float>> path;

public:
	ServoPlanner(ros::NodeHandle node_handle) {
		traj1 = TrajectoryExecution(PLANNING_GROUP, pos_tolerance, orient_tolerance, plan_time_sec, replanning, plan_attempts, base_frame, node_handle);

		// path.push_back({0, 0, 2.262, 0, 0, 0}); // For z >= 2.261 the robot reaches end of workspace, which is a singularity and cant be calculated from the numerical IK
		path.push_back({0, 0, 2.26, 0, 0, 0}); // Home position
		// Start point
		path.push_back({0.209815, -0.770712, 1.363938, 3.141426, 0.022613, -1.599220});
		traj1.executePath(path);

		initialPathExecuted = true;
		ROS_INFO_NAMED("robot_planner5", "Initial pose reached");
	}

	void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
	{
		if (initialPathExecuted) {
			vector<vector<float>> local_path;
			vector<float> current_pose = path.at(path.size()-1);
			local_path.push_back(current_pose);
			current_pose[0] += vel->linear.x;
			current_pose[1] += vel->linear.y;
			current_pose[2] += vel->linear.z;
			current_pose[3] += vel->angular.x;
			current_pose[4] += vel->angular.y;
			current_pose[5] += vel->angular.z;
			local_path.push_back(current_pose);
			traj1.executePath(local_path);

			if (path.size() > 100) {
				// If path has more than 100 states stored in it, then do some cleanup to free up some memory
				path.clear();
				ROS_INFO_NAMED("robot_planner5", "Path cleared");
			}

			// save new pose in global path as well
			path.push_back(current_pose);
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner5");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	ros::Rate loop_rate(1000);
	spinner.start();

	auto* servoPlanner = new ServoPlanner(node_handle);
	// It is important to set queue size small or otherwise it will be very slow to handle all incoming messages!
	ros::Subscriber sub = node_handle.subscribe("kuka_barrett/cmd_vel", 2, &ServoPlanner::velocityCallback, servoPlanner);

	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


