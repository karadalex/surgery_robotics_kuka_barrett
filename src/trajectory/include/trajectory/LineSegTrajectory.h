//
// Created by karadalex on 1/5/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_LINESEGTRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_LINESEGTRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include "kinematics/utils.h"

using namespace std;

class LineSegTrajectory {
public:
	vector<geometry_msgs::Pose> waypoints;
	vector<Eigen::Isometry3d> eigen_waypoints;
	Eigen::Vector3f start, end;
	float xf1, xf2, yf1, yf2, zf1, zf2;

	/**
	 *
	 * @param start
	 * @param end
	 */
	LineSegTrajectory(Eigen::Vector3f _start, Eigen::Vector3f _end);

	/**
	 *
	 * @param samples
	 * @param left_mat
	 * @param right_mat
	 * @return
	 */
	vector<geometry_msgs::Pose> getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat = Eigen::Matrix4d::Identity(), Eigen::Matrix4d right_mat = Eigen::Matrix4d::Identity());

private:
	float getWaypointXCoord(float t);
	float getWaypointYCoord(float t);
	float getWaypointZCoord(float t);
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_LINESEGTRAJECTORY_H
