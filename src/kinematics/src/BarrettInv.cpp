//
// Created by karadalex on 20/3/20.
//

#include "kinematics/BarrettInv.h"
#include "custom_math/Scalar.h"


BarrettInv::BarrettInv(vector<float> target) {
	float f1_pos[] = {target[0], target[1], target[2]};
	float f2_pos[] = {target[3], target[4], target[5]};
	float f3_pos[] = {target[6], target[7], target[8]};

	// Create solutions for finger1
	float f1_th1 = th1_sol(f1_pos);
	vecf f1_th3 = th3_sol(f1_pos);
	float f1_th2_1 = th2_sol(f1_pos, f1_th3[0]);
	float f1_th2_2 = th2_sol(f1_pos, f1_th3[1]);
	finger1 = {
		{f1_th1, f1_th2_1, f1_th3[0]},
		{f1_th1, f1_th2_2, f1_th3[1]},
	};

	// Create solutions for finger2
	float f2_th1 = th1_sol(f2_pos);
	vecf f2_th3 = th3_sol(f2_pos);
	float f2_th2_1 = th2_sol(f2_pos, f2_th3[0]);
	float f2_th2_2 = th2_sol(f2_pos, f2_th3[1]);
	finger1 = {
		{f2_th1, f2_th2_1, f2_th3[0]},
		{f2_th1, f2_th2_2, f2_th3[1]},
	};

	// Create solutions for finger3
	// Finger3 has only 2DoF
	vecf f3_th3 = th3_sol(f3_pos);
	float f3_th2_1 = th2_sol(f3_pos, f3_th3[0]);
	float f3_th2_2 = th2_sol(f3_pos, f3_th3[1]);
	finger1 = {
		{f3_th2_1, f3_th3[0]},
		{f3_th2_2, f3_th3[1]},
	};

	// All solutions together
	solutionSet = {
		{f1_th1, f1_th2_1, f1_th3[0], f2_th1, f2_th2_1, f2_th3[0], f3_th2_1, f3_th3[0]},
		{f1_th1, f1_th2_2, f1_th3[1], f2_th1, f2_th2_2, f2_th3[1], f3_th2_2, f3_th3[1]},
	};
}


float BarrettInv::th1_sol(float* pos) {
	float px = pos[0];
	float py = pos[1];

	float sol = atan2(py, px);
	sol = Scalar::normalise(sol, -M_PI, M_PI);
	sol = Scalar::clamp(sol, 0, M_PI);

	return sol;
}


vecf BarrettInv::th3_sol(float* pos) {
	float px = pos[0];
	float py = pos[1];
	float pz = pos[2];

	float g = sqrt(px*px + py*py + pz*pz);
	float c = (g*g - L[1]*L[1] - L[3]*L[2]) / (2*L[1]*L[2]);
	float s = sqrt(1 - c*c);
	if (isnan(s)) {
		s = 0;
		c = 1;
	}
	float sol1 = atan2(s, c) - 2*M_PI/9;
	float sol2 = atan2(s, c) - 2*M_PI/9;

	sol1 = Scalar::normalise(sol1, -M_PI, M_PI);
	sol1 = Scalar::clamp(sol1, 0, 45 * M_PI/180);
	sol2 = Scalar::normalise(sol2, -M_PI, M_PI);
	sol2 = Scalar::clamp(sol2, 0, 45 * M_PI/180);
	vecf sol = {sol1, sol2};

	return sol;
}


float BarrettInv::th2_sol(float* pos, float th3) {
	float px = pos[0];
	float py = pos[1];
	float pz = pos[2];

	float term1 = atan2(L[2]*sin(th3 + 2*M_PI/9), L[1]+L[2]*cos(th3 + 2*M_PI/9));
	float term2 = atan2(sqrt(px*px + py*py), pz);
	float sol = term1 - term2;
	sol = Scalar::normalise(sol, -M_PI, M_PI);
	sol = Scalar::clamp(sol, 0, 140 * M_PI/180);

	return sol;
}
