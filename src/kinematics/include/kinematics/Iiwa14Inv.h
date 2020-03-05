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
	void updateForwardKinematics(float th[7]);

	// Inverse Kinematics
	vecf th1,th2,th3,th4,th5,th6;
	void solveIK(Pose* targetPose);
	void buildSolutionSet();
	void solveIKNumerically(Pose* targetPose);

private:
	float ky,s5,kx,c4,s4,iy,c6,s6,px,py,pz,c3,s3,s2,c2,g11,g12;

	void initializeForwardKinematicsTransformations();

	void th1_sol();
	void th2_sol();
	void th3_sol();
	void th4_sol();
	void th5_sol();
	void th6_sol();

};


#endif //SRC_IIWA14INV_H
