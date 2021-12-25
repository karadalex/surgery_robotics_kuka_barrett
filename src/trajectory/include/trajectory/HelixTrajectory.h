//
// Created by karadalex on 25/12/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_HELIXTRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_HELIXTRAJECTORY_H


#include "trajectory/TrajectoryBase.h"

using namespace std;


class HelixTrajectory: public TrajectoryBase {
public:
	vector<geometry_msgs::Pose> waypoints;
	vector<Eigen::Isometry3d> eigen_waypoints;

	/**
	 *
	 * @param center
	 * @param radius
	 * @param cycles
	 * @param beta
	 */
	HelixTrajectory(Eigen::Vector3f center, float radius, float cycles, float beta);

	/**
	 *
	 * @param samples
	 * @param left_mat
	 * @param right_mat
	 * @return
	 */
	vector<geometry_msgs::Pose> getCartesianWaypoints(int samples, Eigen::Matrix4d left_mat = Eigen::Matrix4d::Identity(), Eigen::Matrix4d right_mat = Eigen::Matrix4d::Identity());

protected:
	float getWaypointXCoord(float t) override;
	float getWaypointYCoord(float t) override;
	float getWaypointZCoord(float t) override;

private:
	float xf0, yf0, zf0, r0, beta, cycles;
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_HELIXTRAJECTORY_H
