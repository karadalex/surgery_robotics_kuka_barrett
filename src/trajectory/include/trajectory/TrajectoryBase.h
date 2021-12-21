//
// Created by karadalex on 21/12/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_TRAJECTORYBASE_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_TRAJECTORYBASE_H

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include "kinematics/utils.h"

using namespace std;


class TrajectoryBase {
public:
	vector<geometry_msgs::Pose> waypoints;
	vector<Eigen::Isometry3d> eigen_waypoints;

	/**
	 *
	 * @param samples
	 * @param left_mat
	 * @param right_mat
	 * @return
	 */
	vector<geometry_msgs::Pose> getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat = Eigen::Matrix4d::Identity(), Eigen::Matrix4d right_mat = Eigen::Matrix4d::Identity());

protected:
	virtual float getWaypointXCoord(float t) = 0;
	virtual float getWaypointYCoord(float t) = 0;
	virtual float getWaypointZCoord(float t) = 0;
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_TRAJECTORYBASE_H
