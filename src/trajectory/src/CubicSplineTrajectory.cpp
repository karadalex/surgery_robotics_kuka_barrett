//
// Created by karadalex on 1/5/21.
//

#include "trajectory/CubicSplineTrajectory.h"


CubicSplineSegmentTrajectory::CubicSplineSegmentTrajectory(vector<Eigen::Vector3f> positions, vector<Eigen::Vector3f> velocities) {
	// 2 positions and 2 velocities are needed for this segment to be constructed
	assert(positions.size() == 2);
	assert(velocities.size() == 2);

	for (int i = 0; i < 2; ++i) {
		Eigen::Vector3f p = positions.at(i);
		Eigen::Vector3f v = velocities.at(i);

		x.push_back(p.x()); xd.push_back(v.x());
		y.push_back(p.y()); yd.push_back(v.y());
		z.push_back(p.z()); zd.push_back(v.z());
	}

	float tau = 1.0;
	int i = 0;

	axi = (1.0/pow(tau, 3.0)) * (xd[i+1]*tau - 2*x[i+1] + xd[i]*tau + 2*x[i]);
	bxi = (1.0/pow(tau, 2.0))*(3*x[i+1] - 3*x[i] - xd[i+1]*tau - 2*xd[i]*tau);
	cxi = xd[i];
	dxi = x[i];

	ayi = (1.0/pow(tau, 3.0)) * (yd[i+1]*tau - 2*y[i+1] + yd[i]*tau + 2*y[i]);
	byi = (1.0/pow(tau, 2.0))*(3*y[i+1] - 3*y[i] - yd[i+1]*tau - 2*yd[i]*tau);
	cyi = yd[i];
	dyi = y[i];

	azi = (1.0/pow(tau, 3.0)) * (zd[i+1]*tau - 2*z[i+1] + zd[i]*tau + 2*z[i]);
	bzi = (1.0/pow(tau, 2.0))*(3*z[i+1] - 3*z[i] - zd[i+1]*tau - 2*zd[i]*tau);
	czi = zd[i];
	dzi = z[i];
}

float CubicSplineSegmentTrajectory::getWaypointXCoord(float t) {
	float xfi = axi*pow(t, 3.0) + bxi*pow(t, 2.0) + cxi*t + dxi;
	return xfi;
}

float CubicSplineSegmentTrajectory::getWaypointYCoord(float t) {
	float yfi = ayi*pow(t, 3.0) + byi*pow(t, 2.0) + cyi*t + dyi;
	return yfi;
}

float CubicSplineSegmentTrajectory::getWaypointZCoord(float t) {
	float zfi = azi*pow(t, 3.0) + bzi*pow(t, 2.0) + czi*t + dzi;
	return zfi;
}


CubicSplineTrajectory::CubicSplineTrajectory(vector<Eigen::Vector3f> positions, vector<Eigen::Vector3f> velocities) {
	// Make sure position points and velocities arrays have the same size
	assert(positions.size() == velocities.size());

	for (int i = 0; i < positions.size()-1; ++i) {
		vector<Eigen::Vector3f> pos_pair = {positions.at(i), positions.at(i+1)};
		vector<Eigen::Vector3f> vel_pair = {velocities.at(i), velocities.at(i+1)};
		CubicSplineSegmentTrajectory* spline = new CubicSplineSegmentTrajectory(pos_pair, vel_pair);
		cubicSplineSegmentTrajectories.push_back(spline);
	}
}

vector<geometry_msgs::Pose> CubicSplineTrajectory::getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat, Eigen::Matrix4d right_mat) {
	for (auto spline : cubicSplineSegmentTrajectories) {
		vector<geometry_msgs::Pose> spline_waypoints = spline->getCartesianWaypoints(samples, left_mat, right_mat);
		for (auto w: spline_waypoints) waypoints.push_back(w);

		vector<Eigen::Isometry3d> spline_eigen_waypoints = spline->eigen_waypoints;
		for (auto ew: spline_eigen_waypoints) eigen_waypoints.push_back(ew);
	}

	return waypoints;
}