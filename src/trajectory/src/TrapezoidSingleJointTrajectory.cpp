//
// Created by karadalex on 27/12/21.
//

#include "trajectory/TrapezoidSingleJointTrajectory.h"

TrapezoidSingleJointTrajectory::TrapezoidSingleJointTrajectory(double _q1, double _q2, double _t1, double _t2, double _tau, double _qdc)
	: q1(_q1), q2(_q2), qdc(_qdc), t1(_t1), t2(_t2), tau(_tau) {
	computeConstants();
}

void TrapezoidSingleJointTrajectory::setPositions(double _q1, double _q2) {
	q1 = _q1;
	q2 = _q2;
	computeConstants();
}

void TrapezoidSingleJointTrajectory::setVelocityProfile(double _t1, double _t2, double _tau, double _qdc) {
	t1 = _t1;
	t2 = _t2;
	tau = _tau;
	qdc = _qdc;
	computeConstants();
}


void TrapezoidSingleJointTrajectory::computeTrajectory(int _samples) {
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
		position_waypoints.push_back(getJointPosition(t));
		velocity_waypoints.push_back(getJointVelocity(t));
		acceleration_waypoints.push_back(getJointAcceleration(t));
		time_waypoints.push_back(t);
	}
}

double TrapezoidSingleJointTrajectory::getJointPosition(double t) {
	double qi;
	if (t1 <= t && t <= ta) {
		qi = q1 + 0.5*a*pow(t - t1, 2);
	} else if (ta < t && t <= td) {
		qi = getJointPosition(ta) + qdc*(t - ta);
	} else {
		//	td < t <= t2
		// qi = q2 - 0.5*a*pow(t - t2, 2);
		qi = getJointPosition(td);
		double step = (t2 - t1) / samples;
		double _t = td;
		while (_t <= t) {
			qi += getJointVelocity(_t)*step;
			_t += step;
		}
	}

	// Integration solution: slower, TODO: prefer above analytical solution
	// double qi = q1;
	// double step = (t2 - t1) / samples;
	// double _t = t1;
	// while (_t <= t) {
	// 	qi += getJointVelocity(_t)*step;
	// 	_t += step;
	// }
	return qi;
}

double TrapezoidSingleJointTrajectory::getJointVelocity(double t) {
	double qdi;
	if (t1 <= t && t <= ta) {
		qdi = a*t;
	} else if (ta < t && t <= td) {
		qdi = qdc;
	} else {
		//	td < t <= t2
		qdi = -a*(t - t2);
	}

	// Integration solution: slower, avoid
	// double qdi = 0;
	// double step = (t2 - t1) / samples;
	// double _t = t1;
	// while (_t <= t) {
	// 	qdi += getJointAcceleration(_t)*step;
	// 	_t += step;
	// }

	return qdi;
}

double TrapezoidSingleJointTrajectory::getJointAcceleration(double t) {
	if (t1 <= t && t < ta) {
		return a;
	} else if (ta <= t && t < td) {
		return 0;
	} else {
		//	td <= t <= t2
		return -a;
	}
}

void TrapezoidSingleJointTrajectory::computeConstants() {
	double vel_constraint = (q2 - q1) / t2;
	if (q1 <= q2) {
		if (qdc < vel_constraint || qdc > 2*vel_constraint) {
			qdc = vel_constraint;
		}
	} else {
		// q1 > q2
		if (qdc < 2*vel_constraint || qdc > vel_constraint) {
			qdc = vel_constraint;
		}
	}

	a = qdc / ta;
	ta = t1 + tau;
	td = t2 - tau;
}
