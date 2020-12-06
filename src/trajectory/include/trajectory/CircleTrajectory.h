//
// Created by karadalex on 11/8/20.
//

#ifndef SRC_CIRCLETRAJECTORY_H
#define SRC_CIRCLETRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;


class CircleTrajectory {
public:
	vector<geometry_msgs::Pose> waypoints;

	/**
	 *
	 * @param center
	 * @param radius
	 */
	CircleTrajectory(Eigen::Vector3f center, float radius);

	/**
	 *
	 * @param samples
	 * @param left_mat
	 * @param right_mat
	 * @return
	 */
	vector<geometry_msgs::Pose> getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat = Eigen::Matrix4d::Identity(), Eigen::Matrix4d right_mat = Eigen::Matrix4d::Identity());

private:
	float xf0, yf0, zf0, r0;

	float getWaypointXCoord(float t);
	float getWaypointYCoord(float t);
	float getWaypointZCoord(float t);
};


#endif //SRC_CIRCLETRAJECTORY_H
