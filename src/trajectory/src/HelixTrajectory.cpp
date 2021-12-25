//
// Created by karadalex on 25/12/21.
//

#include "trajectory/HelixTrajectory.h"


HelixTrajectory::HelixTrajectory(Eigen::Vector3f center, float radius, float cycles, float beta) : r0(radius), cycles(cycles), beta(beta) {
	xf0 = center(0);
	yf0 = center(1);
	zf0 = center(2);
}

float HelixTrajectory::getWaypointXCoord(float t) {
	float xf = r0 * cos(2*M_PI*t) + xf0;
	return xf;
}

float HelixTrajectory::getWaypointYCoord(float t) {
	float yf = r0 * sin(2*M_PI*t) + yf0;
	return yf;
}

float HelixTrajectory::getWaypointZCoord(float t) {
	return beta * t + zf0;
}

vector<geometry_msgs::Pose> HelixTrajectory::getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat, Eigen::Matrix4d right_mat) {
	float step = cycles / (float)samples;
	float xf, yf, zf, rho, theta, phi;

	for (int i = 0; i < samples-1; ++i) {
		// Get cartesian coordinates w.r.t. {F} reference frame
		xf = getWaypointXCoord(step*i);
		yf = getWaypointYCoord(step*i);
		zf = getWaypointZCoord(step*i);

		// Convert to spherical coordinates w.r.t. {F} reference frame
		rho = sqrt(xf*xf + yf*yf + zf*zf);
		theta = atan2(sqrt(xf*xf + yf*yf), zf);
		phi = atan2(yf, xf);

		// Generate pose in matrix form, this is the matrix F_T_B = p1
		// transformation to {B} w.r.t. {F}
		Eigen::Matrix4d p1;
		p1 << cos(theta)*cos(phi), -sin(phi), sin(theta)*cos(phi), rho*sin(theta)*cos(phi),
						cos(theta)*sin(phi),  cos(phi), sin(theta)*sin(phi), rho*sin(theta)*sin(phi),
						-sin(theta),          0,        cos(theta),          rho*cos(theta),
						0,                    0,        0,                    1;

		// Convert p1 w.r.t to Universal frame
		Eigen::Matrix4d p2 = left_mat * p1 * right_mat;

		// Generate pose in Quaternion Form
		geometry_msgs::Pose pose = matrixTransformToQuaternionPose(p2);

		// Add pose to waypoints list
		waypoints.push_back(pose);
		Eigen::Isometry3d eigenPose = Eigen::Isometry3d(p2);
		eigen_waypoints.push_back(eigenPose);
	}

	return waypoints;
}