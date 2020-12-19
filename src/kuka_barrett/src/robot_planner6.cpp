/**
 * Robot Planner 4:
 * Planning a simple pick and place program using the gazebo world 2 (run it using roslaunch main program2.launch)
 * Code based on http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
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

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
	posture.joint_names.resize(8);
	posture.joint_names = {
					"bh_j11_joint", "bh_j12_joint", "bh_j13_joint",
					"bh_j21_joint", "bh_j22_joint", "bh_j23_joint",
					"bh_j32_joint", "bh_j33_joint"
	};

	/* Set them as open, wide enough for the object to fit. */
	posture.points.resize(1);
	posture.points[0].positions.resize(8);
	for (int j = 0; j < posture.joint_names.size(); ++j)
		posture.points[0].positions[j] = 0.0;
	posture.points[0].time_from_start = ros::Duration(0.5);
}

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
	for (int j = 0; j < posture.joint_names.size(); ++j)
		posture.points[0].positions[j] = 0.8;
	posture.points[0].time_from_start = ros::Duration(0.5);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_planner6");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::WallDuration(1.0).sleep();
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface move_group("iiwa_arm");
	move_group.setPlanningTime(0.5);
	move_group.allowReplanning(true);
	move_group.setNumPlanningAttempts(3);
	// move_group.setPlannerId("RRTstar");
	move_group.setGoalPositionTolerance(0.0005);
	move_group.setGoalOrientationTolerance(0.0005);

	ros::WallDuration(1.0).sleep();

	// ****************************************************************************
	// Pick Pipeline
	// ****************************************************************************
	std::vector<moveit_msgs::Grasp> grasps;
	grasps.resize(1);

	// Setting grasp pose
	grasps[0].grasp_pose.header.frame_id = "world";
	tf2::Quaternion orientation;
	orientation.setRPY(-M_PI, 0, M_PI_2);
	grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
	grasps[0].grasp_pose.pose.position.x = -0.610318;
	grasps[0].grasp_pose.pose.position.y = -0.347105;
	grasps[0].grasp_pose.pose.position.z = 1.175;

	// Setting pre-grasp approach
	// Defined with respect to frame_id
	grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
	// Direction is set as positive x axis
	grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
	grasps[0].pre_grasp_approach.min_distance = 0.1;
	grasps[0].pre_grasp_approach.desired_distance = 0.2;

	// Setting post-grasp retreat
	// Defined with respect to frame_id
	grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
	// Direction is set as positive z axis
	grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
	grasps[0].post_grasp_retreat.min_distance = 0.1;
	grasps[0].post_grasp_retreat.desired_distance = 0.2;

	// Setting posture of eef before grasp
	openGripper(grasps[0].pre_grasp_posture);

	// Setting posture of eef during grasp
	closedGripper(grasps[0].grasp_posture);

	// Set support surface as table1.
	move_group.setSupportSurfaceName("table1");
	// Call pick to pick up the object using the grasps given
	move_group.pick("surgical_tool1", grasps);

	// ****************************************************************************
	// Place Pipeline
	// ****************************************************************************
	std::vector<moveit_msgs::PlaceLocation> place_location;
	place_location.resize(1);

	// Setting place location pose
	place_location[0].place_pose.header.frame_id = "world";
	orientation.setRPY(0, 0, M_PI / 2);
	place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
	// While placing it is the exact location of the center of the object.
	place_location[0].place_pose.pose.position.x = 0.494890;
	place_location[0].place_pose.pose.position.y = 0.465734;
	place_location[0].place_pose.pose.position.z = 1.655359;
	orientation.setRPY(-0.003199, -0.969072, 1.599642);
	place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

	// Setting pre-place approach
	// Defined with respect to frame_id
	place_location[0].pre_place_approach.direction.header.frame_id = "world";
	// Direction is set as negative z axis
	place_location[0].pre_place_approach.direction.vector.z = -1.0;
	place_location[0].pre_place_approach.min_distance = 0.1;
	place_location[0].pre_place_approach.desired_distance = 0.2;

	// Setting post-grasp retreat
	// Defined with respect to frame_id
	place_location[0].post_place_retreat.direction.header.frame_id = "world";
	// Direction is set as negative y axis
	place_location[0].post_place_retreat.direction.vector.z = 1.0;
	place_location[0].post_place_retreat.min_distance = 0.1;
	place_location[0].post_place_retreat.desired_distance = 0.2;

	// Setting posture of eef after placing object
	// Similar to the pick case
	openGripper(place_location[0].post_place_posture);

	move_group.setSupportSurfaceName("table1");
	move_group.place("surgical_tool1", place_location);

	ros::shutdown();
	return 0;
}


