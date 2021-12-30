//
// Created by karadalex on 29/12/21.
//
// Implementing using integration methodology, which is slower than analytical solutions.
// TODO: Replace integrations (sum/while loops with analytical equations for position, velocity and acceleration)
//

#include "trajectory/SCurveSingleJointTrajectory.h"

SCurveSingleJointTrajectory::SCurveSingleJointTrajectory(double _q1, double _q2, double _t1, double _t2, double _tau1, double _tau2, double _qddc)
				: q1(_q1), q2(_q2), qddc(_qddc), t1(_t1), t2(_t2), tau1(_tau1), tau2(_tau2) {
	computeConstants();
}

void SCurveSingleJointTrajectory::setPositions(double _q1, double _q2) {
	q1 = _q1;
	q2 = _q2;
	computeConstants();
}

void SCurveSingleJointTrajectory::setVelocityProfile(double _t1, double _t2, double _tau1, double _tau2, double _qddc) {
	t1 = _t1;
	t2 = _t2;
	tau1 = _tau1;
	tau2 = _tau2;
	qddc = _qddc;
	computeConstants();
}


void SCurveSingleJointTrajectory::computeTrajectory(int _samples) {
	samples = _samples;
	double step = (t2 - t1) / samples;

	q1c = getJointPosition(t1);
	q2c = getJointPosition(t2);

	// Clear waypoints, in case that these arrays have data from previous calls of this method
	position_waypoints.clear();
	velocity_waypoints.clear();
	acceleration_waypoints.clear();

	double t;
	for (int i = 0; i < samples; ++i) {
		t = t1 + i*step;
		double qi = ((getJointPosition(t) - q1) / (q2c - q1c)) * (q2 - q1) + q1;
		position_waypoints.push_back(qi);
		velocity_waypoints.push_back(getJointVelocity(t));
		acceleration_waypoints.push_back(getJointAcceleration(t));
		time_waypoints.push_back(t);
	}
}

double SCurveSingleJointTrajectory::getJointPosition(double t) {
	double qi = q1;
	double step = (t2 - t1) / samples;
	double _t = t1;
	while (_t <= t) {
		qi += getJointVelocity(_t)*step;
		_t += step;
	}

	return qi;
}

double SCurveSingleJointTrajectory::getJointVelocity(double t) {
	double qdi = 0;
	double step = (t2 - t1) / samples;
	double _t = t1;
	while (_t <= t) {
		qdi += getJointAcceleration(_t)*step;
		_t += step;
	}

	return qdi;
}

double SCurveSingleJointTrajectory::getJointAcceleration(double t) {
	double qddi;
	if (t1 <= t && t <= ta) {
		qddi = ja*t;
	} else if (ta < t && t <= tb) {
		qddi = qddc;
	} else if (tb < t && t <= tc) {
		qddi = qddc - ja*(t - tb);
	} else if (tc < t && t <= td) {
		qddi = 0;
	} else if (td < t && t <= te) {
		qddi = -ja*(t - td);
	} else if (te < t && t <= tg) {
		qddi = -qddc;
	} else {
		qddi = -qddc + ja*(t - tg);
	}

	// Integration solution: slower, avoid
	// double qddi = 0;
	// double step = (t2 - t1) / samples;
	// double _t = t1;
	// while (_t <= t) {
	// 	qddi += getJointJerk(_t)*step;
	// 	_t += step;
	// }
	return qddi;
}

double SCurveSingleJointTrajectory::getJointJerk(double t) {
	double qdddi;

	if (t1 <= t && t <= ta) qdddi = ja;
	else if (ta < t && t <= tb) qdddi = 0;
	else if (tb < t && t <= tc) qdddi = -ja;
	else if (tc < t && t <= td) qdddi = 0;
	else if (td < t && t <= te) qdddi = -ja;
	else if (te < t && t <= tg) qdddi = 0;
	else qdddi = ja;

	return qdddi;
}


void SCurveSingleJointTrajectory::computeConstants() {
	ta = t1 + tau1;
	tb = t1 + tau1 + tau2;
	tc = t1 + 2*tau1 + tau2;
	td = t2 - 2*tau1 - tau2;
	te = t2 - tau1 - tau2;
	tg = t2 - tau1;

	if (q1 <= q2) qddc = abs(qddc);
	else qddc = -abs(qddc);

	ja = qddc / ta;
}
