//
// Created by karadalex on 31/3/20.
//

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scene_objects");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools.loadRemoteControl();

	static const std::string PLANNING_GROUP = "iiwa_arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	//Vector to scale 3D file units (to convert from mm to meters for example)
	Vector3d vectorScale(0.001, 0.001, 0.001);
	// Define a collision object ROS message.
	moveit_msgs::CollisionObject co;
//	co.header.frame_id = "world";
	// The id of the object is used to identify it.
	co.id = "mounting_dock";

	//Path where the .dae or .stl object is located
	shapes::Mesh* m = shapes::createMeshFromResource("package://kuka_barrett_gazebo/objects/mounting_dock/meshes/mounting_dock.stl", vectorScale);
	ROS_INFO("Your mesh was loaded");

	shape_msgs::Mesh mesh;
	shapes::ShapeMsg mesh_msg;
	shapes::constructMsgFromShape(m, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

	//Define a pose for your mesh (specified relative to frame_id)
	geometry_msgs::Pose obj_pose;
	obj_pose.position.x = 0.4;
	obj_pose.position.y = 0.7;
	obj_pose.position.z = 1.25;

	// Add the mesh to the Collision object message
	co.meshes.push_back(mesh);
	co.mesh_poses.push_back(obj_pose);
	co.operation = co.ADD;

	// Create vector of collision objects to add
	std::vector<moveit_msgs::CollisionObject> object;
	object.push_back(co);

	// Add the collision object into the world
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	planning_scene_interface.addCollisionObjects(object);

	visual_tools.trigger();

	// Wait for MoveGroup to recieve and process the collision object message
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");


	ros::shutdown();
	return 0;
}
