//
// Created by karadalex on 11/8/20.
//

#ifndef SRC_CIRCLETRAJECTORY_H
#define SRC_CIRCLETRAJECTORY_H

#include "trajectory/TrajectoryBase.h"

using namespace std;


class CircleTrajectory: public TrajectoryBase {
public:
	vector<geometry_msgs::Pose> waypoints;
	vector<Eigen::Isometry3d> eigen_waypoints;

	/**
	 *
	 * @param center
	 * @param radius
	 */
	CircleTrajectory(Eigen::Vector3f center, float radius);

protected:
	float getWaypointXCoord(float t) override;
	float getWaypointYCoord(float t) override;
	float getWaypointZCoord(float t) override;

private:
	float xf0, yf0, zf0, r0;
};


#endif //SRC_CIRCLETRAJECTORY_H
