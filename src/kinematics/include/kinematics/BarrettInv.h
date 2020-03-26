//
// Created by karadalex on 20/3/20.
//

#ifndef SRC_BARRETTINV_H
#define SRC_BARRETTINV_H

#include "Pose.h"
#include "kinematics/Transformation.h"
#include "custom_math/Scalar.h"

class BarrettInv {
public:
	BarrettInv(vector<float> target);

	// DH Parameters (same for all fingers)
	vecf L = {0.05, 0.07, 0.058};

	// Inverse Kinematics solutions for each finger
	vector<vecf> finger1;
	vector<vecf> finger2;
	vector<vecf> finger3;

	// Solution set - All solutions in one structure
	vector<vecf> solutionSet;

private:
	float th1_sol(float* pos);
	float th2_sol(float* pos, float th1, float th3);
	vecf th3_sol(float* pos, float th1);

};


#endif //SRC_BARRETTINV_H
