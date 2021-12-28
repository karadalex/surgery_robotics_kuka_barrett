//
// Created by karadalex on 27/12/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_TRAPEZOIDSINGLEJOINTTRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_TRAPEZOIDSINGLEJOINTTRAJECTORY_H


#include <vector>
#include <cmath>

using namespace std;

class TrapezoidSingleJointTrajectory {
public:
	vector<double> position_waypoints, velocity_waypoints, acceleration_waypoints, time_waypoints;

	/**
	 *
	 * @param _q1
	 * @param _q2
	 * @param _t1
	 * @param _t2
	 * @param _tau
	 * @param _qdc
	 */
	TrapezoidSingleJointTrajectory(double _q1, double _q2, double _t1, double _t2, double _tau, double _qdc);

	/**
	 *
	 * @param _q1
	 * @param _q2
	 */
	void setPositions(double _q1, double _q2);

	/**
	 *
	 * @param _qd1
	 * @param _qd2
	 * @param _qdc
	 */
	void setVelocityProfile(double _t1, double _t2, double _tau, double _qdc);

	/**
	 *
	 * @param samples
	 */
	void computeTrajectory(int samples);

private:
	double q1, q2;
	double qdc, a;
	double t1 = 0.0;
	double ta = 0.25;
	double td = 0.75;
	double t2 = 1.0;
	double tau = 0.25;

	/**
	 * Evaluate joint position at time t
	 * @param t Time
	 * @return
	 */
	double getJointPosition(double t);

	/**
	 * Evaluate joint velocity at time t
	 * @param t Time
	 * @return
	 */
	double getJointVelocity(double t);

	/**
	 * Evaluate joint acceleration at time t
	 * @param t Time
	 * @return
	 */
	double getJointAcceleration(double t);

	/**
	 *
	 */
	void computeConstants();
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_TRAPEZOIDSINGLEJOINTTRAJECTORY_H
