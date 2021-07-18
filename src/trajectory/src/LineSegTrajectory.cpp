//
// Created by karadalex on 1/5/21.
//

#include "trajectory/LineSegTrajectory.h"

LineSegTrajectory::LineSegTrajectory(Eigen::Vector3f center, float radius) : r0(radius) {
	xf0 = center(0);
	yf0 = center(1);
	zf0 = center(2);
}

float LineSegTrajectory::getWaypointXCoord(float t) {
	float xf = r0 * cos(2*M_PI*t) + xf0;
	return xf;
}

float LineSegTrajectory::getWaypointYCoord(float t) {
	float yf = r0 * sin(2*M_PI*t) + yf0;
	return yf;
}

float LineSegTrajectory::getWaypointZCoord(float t) {
	return zf0;
}

vector<geometry_msgs::Pose> LineSegTrajectory::getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat, Eigen::Matrix4d right_mat) {

	return waypoints;
}