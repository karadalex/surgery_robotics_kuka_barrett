//
// Created by karadalex on 2/3/20.
//

#ifndef SRC_IIWA14INV_H
#define SRC_IIWA14INV_H


#include "Pose.h"
#include "kinematics/Transformation.h"
#include "custom_math/Scalar.h"

class Iiwa14Inv {
public:
	Iiwa14Inv(Pose* targetPose);
	Pose* target;

	// Denavit-Hartenberg parameters and Forward Kinematics
	// vector<float> L; // L parameters are all 0, no need to use them
	vector<float> d, a;
	vector<Transformation*> fwdTransformations;
	Transformation* M_U_0, *M_7_TCP;
	matrixf M_0_7;
	void updateForwardKinematics(float th[7]);

	// Inverse Kinematics
	vecf th1,th2,th3,th4,th5,th6,th7;
	void solveIK(Pose* targetPose);
	void buildSolutionSet();
	void printSolutionSet();
	void solveIKNumerically(Pose* targetPose);

	matrixf getSolutionSet();

private:
	float ky,s6,jy,iy,kz,kx,py,px,pz,p_1_5_len,p1x,c4,s4,c1,s1;
	vecf p_0_5, p_1_5;
	// Solution set = 2(th6) x 2(th4) x 2(th2) -> 8rows, 6columns
	matrixf solutionSet;

	void initializeForwardKinematicsTransformations();

	void th1_sol();
	void th2_sol();
	void th3_sol();
	void th4_sol();
	void th5_sol();
	void th6_sol();
	void th7_sol();

};


#endif //SRC_IIWA14INV_H
