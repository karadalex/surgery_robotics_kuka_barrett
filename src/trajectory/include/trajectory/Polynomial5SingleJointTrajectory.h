//
// Created by karadalex on 25/12/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_POLYNOMIAL5SINGLEJOINTTRAJECTORY_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_POLYNOMIAL5SINGLEJOINTTRAJECTORY_H

#include <vector>
#include <cmath>

using namespace std;

class Polynomial5SingleJointTrajectory {
public:
	vector<double> position_waypoints, velocity_waypoints, acceleration_waypoints;

	Polynomial5SingleJointTrajectory();

	/**
	 * Initialize Polynomial5SingleJointTrajectory and set boundary conditions
	 * @param q1 Start joint position
	 * @param q2 End joint position
	 * @param qd1 Start joint velocity (position derivative)
	 * @param qd2 End joint velocity (position derivative)
	 * @param qdd1 Start joint acceleration (position second derivative)
	 * @param qdd2 End joint acceleration (position second derivative)
	 */
	Polynomial5SingleJointTrajectory(double q1, double q2, double qd1, double qd2, double qdd1, double qdd2);

	/**
	 * Set start and end joint positions (if not already set from the constructor)
	 * @param start
	 * @param end
	 */
	void setPositions(double start, double end);

	/**
	 * Set start and end joint velocities (if not already set from the constructor)
	 * @param start
	 * @param end
	 */
	void setVelocities(double start, double end);

	/**
	 * Set start and end joint accelerations (if not already set from the constructor)
	 * @param start
	 * @param end
	 */
	void setAccelerations(double start, double end);

	/**
	 * Compute joint polynomial trajectory
	 * @param samples Number of waypoints at which the trajectory will be evaluated
	 * @param t1 (optional) start time of the trajectory in seconds, defaults to 0.0
	 * @param t2 (optional) end time of the trajectory in seconds, defaults to 1.0
	 */
	void computeTrajectory(int samples, double t1 = 0.0, double t2 = 1.0);

private:
	double q1, q2, qd1, qd2, qdd1, qdd2;
	double a, b, c, d, e, f;

	/**
	 * Polynomial to evaluate joint position at time t
	 * @param t Time
	 * @return
	 */
	double getJointPosition(double t);

	/**
	 * Polynomial to evaluate joint velocity at time t
	 * @param t Time
	 * @return
	 */
	double getJointVelocity(double t);

	/**
	 * Polynomial to evaluate joint acceleration at time t
	 * @param t Time
	 * @return
	 */
	double getJointAcceleration(double t);
};


#endif //SURGERY_ROBOTICS_KUKA_BARRETT_POLYNOMIAL5SINGLEJOINTTRAJECTORY_H
