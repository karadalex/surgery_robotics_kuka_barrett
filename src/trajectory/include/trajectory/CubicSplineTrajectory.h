//
// Created by karadalex on 1/5/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_CUBICSPLINETRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_CUBICSPLINETRAJECTORY_H


#include "trajectory/TrajectoryBase.h"
#include <cmath>

using namespace std;


class CubicSplineSegmentTrajectory: public TrajectoryBase {
public:
	/**
	 *
	 * @param center
	 * @param radius
	 */
	CubicSplineSegmentTrajectory(vector<Eigen::Vector3f> positions, vector<Eigen::Vector3f> velocities);

protected:
	float getWaypointXCoord(float t) override;
	float getWaypointYCoord(float t) override;
	float getWaypointZCoord(float t) override;

private:
	vector<float> x,y,z,xd,yd,zd;
	float axi, bxi, cxi, dxi;
	float ayi, byi, cyi, dyi;
	float azi, bzi, czi, dzi;
};


class CubicSplineTrajectory {
public:
	vector<geometry_msgs::Pose> waypoints;
	vector<Eigen::Isometry3d> eigen_waypoints;

	/**
	 * Setup individual CubicSplineSegmentTrajectory objects
	 * @param center
	 * @param radius
	 */
	CubicSplineTrajectory(vector<Eigen::Vector3f> positions, vector<Eigen::Vector3f> velocities);

	/**
	 *
	 * @param samples
	 * @param left_mat
	 * @param right_mat
	 * @return
	 */
	vector<geometry_msgs::Pose> getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat = Eigen::Matrix4d::Identity(), Eigen::Matrix4d right_mat = Eigen::Matrix4d::Identity());

private:
	vector<CubicSplineSegmentTrajectory*> cubicSplineSegmentTrajectories;
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_CUBICSPLINETRAJECTORY_H
