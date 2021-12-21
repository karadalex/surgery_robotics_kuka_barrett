//
// Created by karadalex on 1/5/21.
//

#include "trajectory/LineSegTrajectory.h"

LineSegTrajectory::LineSegTrajectory(Eigen::Vector3f _start, Eigen::Vector3f _end) : start(_start), end(_end) {
	xf1 = _start.x(); xf2 = _end.x();
	yf1 = _start.y(); yf2 = _end.y();
	zf1 = _start.z(); zf2 = _end.z();
}

float LineSegTrajectory::getWaypointXCoord(float t) {
	float xf = (1 - t)*xf1 + t*xf2;
	return xf;
}

float LineSegTrajectory::getWaypointYCoord(float t) {
	float yf = (1 - t)*yf1 + t*yf2;
	return yf;
}

float LineSegTrajectory::getWaypointZCoord(float t) {
	float zf = (1 - t)*zf1 + t*zf2;
	return zf;
}
