//
// Created by karadalex on 11/8/20.
//

#include "trajectory/CircleTrajectory.h"

CircleTrajectory::CircleTrajectory(Eigen::Vector3f center, float radius) : r0(radius) {
	xf0 = center(0);
	yf0 = center(1);
	zf0 = center(2);
}

float CircleTrajectory::getWaypointXCoord(float t) {
	float xf = r0 * cos(2*M_PI*t) + xf0;
	return xf;
}

float CircleTrajectory::getWaypointYCoord(float t) {
	float yf = r0 * sin(2*M_PI*t) + yf0;
	return yf;
}

float CircleTrajectory::getWaypointZCoord(float t) {
	return zf0;
}
