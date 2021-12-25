//
// Created by karadalex on 1/5/21.
//

#include "trajectory/BSplineTrajectory.h"


BSplineTrajectory::BSplineTrajectory(vector<Eigen::Vector3f> control_points) : cp(control_points) {}

float BSplineTrajectory::getWaypointXCoord(float t) {
  float m = 1 - t;
  float xf = pow(m, 3)*cp[0].x() + 3*pow(m, 2)*t*cp[1].x() + 3*m*pow(t, 2)*cp[2].x() + pow(t,3)*cp[3].x();
	return xf;
}

float BSplineTrajectory::getWaypointYCoord(float t) {
	float m = 1 - t;
  float yf = pow(m, 3)*cp[0].y() + 3*pow(m, 2)*t*cp[1].y() + 3*m*pow(t, 2)*cp[2].y() + pow(t,3)*cp[3].y();
  return yf;
}

float BSplineTrajectory::getWaypointZCoord(float t) {
	float m = 1 - t;
  float zf = pow(m, 3)*cp[0].z() + 3*pow(m, 2)*t*cp[1].z() + 3*m*pow(t, 2)*cp[2].z() + pow(t,3)*cp[3].z();
  return zf;
}
