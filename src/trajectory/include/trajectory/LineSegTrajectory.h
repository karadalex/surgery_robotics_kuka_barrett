//
// Created by karadalex on 1/5/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_LINESEGTRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_LINESEGTRAJECTORY_H

#include "trajectory/TrajectoryBase.h"

using namespace std;

class LineSegTrajectory: public TrajectoryBase {
public:
	Eigen::Vector3f start, end;
	float xf1, xf2, yf1, yf2, zf1, zf2;

	/**
	 *
	 * @param start
	 * @param end
	 */
	LineSegTrajectory(Eigen::Vector3f _start, Eigen::Vector3f _end);

private:
	float getWaypointXCoord(float t) override;
	float getWaypointYCoord(float t) override;
	float getWaypointZCoord(float t) override;
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_LINESEGTRAJECTORY_H
