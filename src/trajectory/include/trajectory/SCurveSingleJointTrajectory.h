//
// Created by karadalex on 29/12/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_SCURVESINGLEJOINTTRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_SCURVESINGLEJOINTTRAJECTORY_H


#include <vector>
#include <cmath>

using namespace std;

class SCurveSingleJointTrajectory {
public:
	vector<double> position_waypoints, velocity_waypoints, acceleration_waypoints, jerk_waypoints, time_waypoints;

	/**
	 *
	 * @param _q1
	 * @param _q2
	 * @param _t1
	 * @param _t2
	 * @param _tau1
	 * @param _tau2
	 * @param _qddc
	 */
	SCurveSingleJointTrajectory(double _q1, double _q2, double _t1, double _t2, double _tau1, double _tau2, double _qddc);

	/**
	 *
	 * @param _q1
	 * @param _q2
	 */
	void setPositions(double _q1, double _q2);

	/**
	 *
	 * @param _t1
	 * @param _t2
	 * @param _tau1
	 * @param _tau2
	 * @param _qddc
	 */
	void setVelocityProfile(double _t1, double _t2, double _tau1, double _tau2, double _qddc);

	/**
	 *
	 * @param samples
	 */
	void computeTrajectory(int samples);

private:
	double q1, q2, ja;
	double qddc;
	double tau1 = 0.05;
	double tau2 = 0.1;
	double t1 = 0.0;
	double ta = 0.05;
	double tb = 0.15;
	double tc = 0.2;
	double td = 0.8;
	double te = 0.85;
	double tg = 0.95;
	double t2 = 1.0;
	int samples;

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
	 * @param t
	 * @return
	 */
	double getJointJerk(double t);

	/**
	 *
	 */
	void computeConstants();
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_SCURVESINGLEJOINTTRAJECTORY_H
