//
// Created by karadalex on 2/3/20.
//

#ifndef SRC_IIWA14INV_H
#define SRC_IIWA14INV_H


#include "Pose.h"
#include "kinematics/Transformation.h"
#include "custom_math/Scalar.h"
#include "Eigen/Dense"

class Iiwa14Inv {
public:
	Iiwa14Inv(Pose* targetPose);
	Pose* target;

	// Denavit-Hartenberg parameters and Forward Kinematics
	// vector<float> L; // L parameters are all 0, no need to use them
	vector<float> d, L, a;
	vector<Transformation*> fwdTransformations;
	Matrix4d M_0_7, M_U_0, M_7_TCP;
	void updateForwardKinematics(vecf th);

	// Inverse Kinematics
	vector<double> th1,th2,th3,th4,th5,th6,th7;
	// Solution set = 2(th6) x 2(th4) x 2(th2) -> 8rows, 6columns
	vector<vector<double>> solutionSet;
	void solveIK(Pose* targetPose);
	void buildSolutionSet();
	void printSolutionSet();
	void solveIKNumerically(Pose* targetPose);
	void validateSolution(vector<double> angles);

private:
	double ix,iy,iz,jx,jy,jz,kx,ky,kz,px,py,pz;

	void initializeForwardKinematicsTransformations();
};


#endif //SRC_IIWA14INV_H
