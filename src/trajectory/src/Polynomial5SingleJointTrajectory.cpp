//
// Created by karadalex on 25/12/21.
//

#include "trajectory/Polynomial5SingleJointTrajectory.h"


Polynomial5SingleJointTrajectory::Polynomial5SingleJointTrajectory(double q1, double q2, double qd1, double qd2,
																																	 double qdd1, double qdd2) : q1(q1), q2(q2), qd1(qd1),
																																															 qd2(qd2), qdd1(qdd1),
																																															 qdd2(qdd2) {
	a = b = c = d = e = f = 0;
}


Polynomial5SingleJointTrajectory::Polynomial5SingleJointTrajectory() {
	// Set positions, velocities, and accelerations in case that Class user does not initialize them
	q1 = q2 = qd1 = qd2 = qdd1 = qdd2 = 0;
	a = b = c = d = e = f = 0;
}


void Polynomial5SingleJointTrajectory::setPositions(double start, double end) {
	q1 = start;
	q2 = end;
}


void Polynomial5SingleJointTrajectory::setVelocities(double start, double end) {
	qd1 = start;
	qd2 = end;
}


void Polynomial5SingleJointTrajectory::setAccelerations(double start, double end) {
	qdd1 = start;
	qdd2 = end;
}


void Polynomial5SingleJointTrajectory::computeTrajectory(int samples, double t1, double t2) {
	a = q1;
	b = qd1;
	c = qdd1/2;
	d = (20*q2 - 20*q1 - (8*qd2 + 12*qd1)*t2 - (3*qdd1 - qdd2)*pow(t2, 2)) / (2*pow(t2, 3.0));
	e = (30*q1 - 30*q2 - (14*qd2 + 16*qd1)*t2 + (3*qdd1 - 2*qdd2)*pow(t2, 2)) / (2*pow(t2, 4));
	f = (12*q2 - 12*q1 - (6*qd2 + 6*qd1)*t2 - (qdd1 - qdd2)*pow(t2, 2)) / (2*pow(t2, 5));

	double step = (t2 - t1) / samples;

	// Clear waypoints, in case that these arrays have data from previous calls of this method
	position_waypoints.clear();
	velocity_waypoints.clear();
	acceleration_waypoints.clear();

	for (int i = 0; i < samples; ++i) {
		position_waypoints.push_back(getJointPosition(i*step));
		velocity_waypoints.push_back(getJointVelocity(i*step));
		acceleration_waypoints.push_back(getJointAcceleration(i*step));
	}
}


double Polynomial5SingleJointTrajectory::getJointPosition(double t) {
	double qi = a + b*t + c*pow(t, 2) + d*pow(t, 3) + e*pow(t, 4) + f*pow(t, 5);
	return qi;
}


double Polynomial5SingleJointTrajectory::getJointVelocity(double t) {
	double qdi = b + 2*c*t + 3*d*pow(t, 2) + 4*e*pow(t, 3) + 5*f*pow(t, 4);
	return qdi;
}


double Polynomial5SingleJointTrajectory::getJointAcceleration(double t) {
	double qddi = 2*c + 6*d*t + 12*e*pow(t, 2) + 20*f*pow(t, 3);
	return qddi;
}
