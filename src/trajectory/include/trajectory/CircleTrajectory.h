//
// Created by karadalex on 11/8/20.
//

#ifndef SRC_CIRCLETRAJECTORY_H
#define SRC_CIRCLETRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/Pose.h>

using namespace std;


class CircleTrajectory {
public:
	vector<geometry_msgs::Pose> waypoints;

	CircleTrajectory(Eigen::Vector3f center, float radius);

	vector<geometry_msgs::Pose> getCartesianWaypoints(int samples);

private:
	float xf0, yf0, zf0, r0;

	float getWaypointXCoord(float t);
	float getWaypointYCoord(float t);
	float getWaypointZCoord(float t);
};


#endif //SRC_CIRCLETRAJECTORY_H
