#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <Eigen/Dense>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace Eigen;

void addObjectToScene(ros::Publisher *scenePub, string meshFullPackagePath, string modelName, geometry_msgs::Pose pose);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scene_objects2");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Visualization
	// ^^^^^^^^^^^^^
	// The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
	moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
	visual_tools.deleteAllMarkers();

	// ROS API
	// ^^^^^^^
	// The ROS API to the planning scene publisher is through a topic interface
	// using "diffs". A planning scene diff is the difference between the current
	// planning scene (maintained by the move_group node) and the new planning
	// scene desired by the user.
	//
	// Advertise the required topic
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// We create a publisher and wait for subscribers
	// Note that this topic may need to be remapped in the launch file
	ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	ros::WallDuration sleep_t(0.5);
	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		sleep_t.sleep();
	}
	// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	// Load mounting dock
	geometry_msgs::Pose mounting_pose;
	// 0.53 -0.025 1.25094 0 0 -1.57079633
	mounting_pose.position.x = 0.53;
	mounting_pose.position.y = -0.025;
	mounting_pose.position.z = 1.25094;
	tf2::Quaternion quaternion;
	quaternion.setRPY(0, 0, -1.57079633);
	mounting_pose.orientation.w = quaternion.getW();
	mounting_pose.orientation.x = quaternion.getX();
	mounting_pose.orientation.y = quaternion.getY();
	mounting_pose.orientation.z = quaternion.getZ();
	addObjectToScene(&planning_scene_diff_publisher, "package://kuka_barrett_gazebo/objects/mounting_dock/meshes/mounting_dock.stl", "mounting_dock", mounting_pose);

	// Load surgical tools
	// string toolPath = "package://kuka_barrett_gazebo/objects/surgical_tool/meshes/surgical_tool_collision.stl";
	// geometry_msgs::Pose genericToolPose;
	// genericToolPose.position.x = 0;
	// genericToolPose.position.y = -0.72;
	// genericToolPose.position.z = 1.0493;

	// geometry_msgs::Pose tool1pose = genericToolPose;
	// tool1pose.position.x = 0;
	// addObjectToScene(&planning_scene_diff_publisher, toolPath, "surgical_tool1", tool1pose);

	// geometry_msgs::Pose tool2pose = genericToolPose;
	// tool2pose.position.x = 0.2;
	// addObjectToScene(&planning_scene_diff_publisher, toolPath, "surgical_tool2", tool2pose);

	// geometry_msgs::Pose tool3pose = genericToolPose;
	// tool3pose.position.x = 0.4;
	// addObjectToScene(&planning_scene_diff_publisher, toolPath, "surgical_tool3", tool3pose);

	// geometry_msgs::Pose tool4pose = genericToolPose;
	// tool4pose.position.x = 0.6;
	// addObjectToScene(&planning_scene_diff_publisher, toolPath, "surgical_tool4", tool4pose);

	ros::shutdown();
	return 0;
}

void addObjectToScene(ros::Publisher *scenePub, string meshFullPackagePath, string modelName, geometry_msgs::Pose pose)
{
	// Define the attached object message
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// We will use this message to add or
	// subtract the object from the world
	// and to attach the object to the robot
	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = modelName;
	/* The header must contain a valid TF frame*/
	attached_object.object.header.frame_id = "world";
	/* The id of the object */
	attached_object.object.id = modelName;

	//Vector to scale 3D file units (to convert from mm to meters for example)
	Vector3d vectorScale(1, 1, 1);
	//Path where the .dae or .stl object is located
	shapes::Mesh* m = shapes::createMeshFromResource(meshFullPackagePath, vectorScale);
	ROS_INFO("Your mesh was loaded");

	shape_msgs::Mesh mesh;
	shapes::ShapeMsg mesh_msg;
	shapes::constructMsgFromShape(m, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

	attached_object.object.meshes.push_back(mesh);
	attached_object.object.mesh_poses.push_back(pose);

	// Note that attaching an object to the robot requires
	// the corresponding operation to be specified as an ADD operation
	attached_object.object.operation = attached_object.object.ADD;

	// Since we are attaching the object to the robot hand to simulate picking up the object,
	// we want the collision checker to ignore collisions between the object and the robot hand
	// attached_object.touch_links = vector<string>{ "object1", "object2", "object3" };

	// Add an object into the environment
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Add the object into the environment by adding it to
	// the set of collision objects in the "world" part of the
	// planning scene. Note that we are using only the "object"
	// field of the attached_object message here.
	ROS_INFO("Adding the object into the world at the location of the hand.");
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(attached_object.object);
	planning_scene.is_diff = true;
	scenePub->publish(planning_scene);
}
