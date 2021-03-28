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

vector<geometry_msgs::Pose> CircleTrajectory::getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat, Eigen::Matrix4d right_mat) {
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

		Eigen::Matrix4d rotx, roty, rotz, p;
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
		// p = p2 * rotx * roty;
		p = p2;

		// Generate pose in Quaternion Form
		geometry_msgs::Pose pose;
		tf2::Quaternion quaternion;

		pose.position.x = p(0,3);
		pose.position.y = p(1,3);
		pose.position.z = p(2,3);

		float roll = atan2(p(1,0), p(0,0));
		float pitch = atan2(-p(2,0), sqrt(p(0,0)*p(0,0) + p(1,0)*p(1,0)));
		float yaw = atan2(p(2,1), p(2,2));

		// float roll, pitch, yaw;
		// roll = pitch = yaw = 0;

		quaternion.setRPY(roll, pitch, yaw);
		pose.orientation.w = quaternion.getW();
		pose.orientation.x = quaternion.getX();
		pose.orientation.y = quaternion.getY();
		pose.orientation.z = quaternion.getZ();

		// Add pose to waypoints list
		waypoints.push_back(pose);
		Eigen::Isometry3d eigenPose = Eigen::Isometry3d(p2);
		eigen_waypoints.push_back(eigenPose);
	}

	return waypoints;
}
