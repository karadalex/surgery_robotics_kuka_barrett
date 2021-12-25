//
// Created by karadalex on 1/5/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_BSPLINETRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_BSPLINETRAJECTORY_H

#include "trajectory/TrajectoryBase.h"
#include <cmath>

using namespace std;


class BSplineTrajectory: public TrajectoryBase {
public:
	vector<geometry_msgs::Pose> waypoints;
	vector<Eigen::Isometry3d> eigen_waypoints;

	/**
	 *
	 * @param control_points
	 */
	BSplineTrajectory(vector<Eigen::Vector3f> control_points);

protected:
	float getWaypointXCoord(float t) override;
	float getWaypointYCoord(float t) override;
	float getWaypointZCoord(float t) override;

private:
  vector<Eigen::Vector3f> cp;
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_BSPLINETRAJECTORY_H
