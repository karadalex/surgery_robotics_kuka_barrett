//
// Created by karadalex on 21/12/21.
//

#include "trajectory/TrajectoryBase.h"

vector<geometry_msgs::Pose> TrajectoryBase::getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat, Eigen::Matrix4d right_mat) {
	float step = 1.0f / (float)samples;
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

		Eigen::Matrix4d rotx, roty, rotz;
		// Rotate by 180deg around x-axis (?)
		rotx << 1,  0,  0, 0,
						0, -1,  0, 0,
						0,  0, -1, 0,
						0,  0,  0, 1;
		// Rotate by 90deg around y-axis (?)
		roty <<  0, 0, 1, 0,
						0, 1, 0, 0,
						-1, 0, 0, 0,
						0, 0, 0, 1;
		// Rotate by 90deg around z-axis (?)
		rotz << -1,  0, 0, 0,
						0, -1, 0, 0,
						0,  0, 1, 0,
						0,  0, 0, 1;
		// p2 = p2 * rotx * roty;

		// Generate pose in Quaternion Form
		geometry_msgs::Pose pose = matrixTransformToQuaternionPose(p2);

		// Add pose to waypoints list
		waypoints.push_back(pose);
		Eigen::Isometry3d eigenPose = Eigen::Isometry3d(p2);
		eigen_waypoints.push_back(eigenPose);
	}

	return waypoints;
}