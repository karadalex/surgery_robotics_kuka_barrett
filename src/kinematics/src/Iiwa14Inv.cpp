//
// Created by karadalex on 2/3/20.
//

#include "kinematics/Iiwa14Inv.h"

Iiwa14Inv::Iiwa14Inv(Pose* targetPose) : target(targetPose) {

	initializeForwardKinematicsTransformations();
	M_U_0 = new Transformation(0, 1, 0, 0);
	M_7_TCP = new Transformation(0, 0, M_PI/2, 0);

	solveIK(targetPose);
}

void Iiwa14Inv::initializeForwardKinematicsTransformations() {
	// Angle th3 is skipped, because is used as a redundant degree of freedom!
	d = {0.36, 0, 0.42, 0.4, 0, 0.8};
	a = {0, -M_PI/2, M_PI, -M_PI/2, -M_PI/2, M_PI/2};

	for (int i = 0; i < 6; ++i) {
		Transformation* M_i = new Transformation(0, d[i], a[i], 0);
		fwdTransformations.push_back(M_i);
	}
}

void Iiwa14Inv::th1_sol() {
	float sol = atan2(py/p1x, px/p1x);
	// Check for singularity.
	// If th1 is not defined (px=py=0) then th1=0
	if (!isnan(sol)) th1.push_back(sol); else th1.push_back(0);
}

void Iiwa14Inv::th2_sol() {
	float phi = acosf((d[2]*d[2] + p_1_5_len*p_1_5_len - d[3]*d[3])/ (2*d[2]*p_1_5_len));
	// Check for singularity
	// If phi is not defined (fully extended) then phi = 0
	if (isnan(phi)) phi = 0;
	float sol1 = atan2(p1x, p_1_5[2]) + phi;
	float sol2 = atan2(p1x, p_1_5[2]) - phi;
	th2.push_back(sol1);
	th2.push_back(sol2);
}

void Iiwa14Inv::th3_sol() {
	th3.push_back(0);
}

void Iiwa14Inv::th4_sol() {
	c4 = (p_1_5_len*p_1_5_len - d[2]*d[2] - d[3]*d[3]) / (2 * d[2] * d[3]);
	// -1 <= c4 <= 1
	c4 = Scalar::clamp(c4, -1, 1);
	s4 = sqrt(1 - c4*c4);
	// Check for singularity.
	// If c4=0, then atan2 is not defined and th4=pi/2
	if (c4 != 0.0) {
		th4.push_back(atan2(s4, c4));
		th4.push_back(atan2(-s4, c4));
	} else {
		th4.push_back(0);
		th4.push_back(0);
	}
}

void Iiwa14Inv::th5_sol() {
	kz = M_0_7[2][2];
	kx = M_0_7[0][2];
	th5.push_back(atan2(-kz, kx));
}

void Iiwa14Inv::th6_sol() {
	ky = M_0_7[1][2];
	s6 = sqrt(1-ky*ky);
	if (isnan(s6)) s6 = 0;
	th6.push_back(atan2(s6, ky));
	th6.push_back(atan2(+s6, ky));
}

void Iiwa14Inv::th7_sol() {
	jy = M_0_7[1][1];
	iy = M_0_7[1][0];
	th7.push_back(atan2(-jy, iy));
}

void Iiwa14Inv::updateForwardKinematics(float *th) {
	for (int i = 0; i < 7; ++i) {
		fwdTransformations.at(i)->update(th[i]);
	}
}

void Iiwa14Inv::solveIK(Pose* targetPose) {

	M_0_7 = targetPose->pose;
	M_0_7 = Matrix::mul(M_0_7, M_7_TCP->inverse);
	M_0_7 = Matrix::mul(M_U_0->inverse, M_0_7);
	Matrix::printMatrix(M_0_7, "M_0_7");

	py = M_0_7[1][3];
	px = M_0_7[0][3];
	p_0_5.resize(4, 0);
	for (int i = 0; i < 4; ++i) {
		p_0_5[i] = M_0_7[i][3];
	}
	// p_1_5 is used in th2_sol and th4_sol methods
	p_1_5 = Matrix::mul(fwdTransformations.at(0)->inverse, p_0_5);
	p_1_5_len = p_1_5[0]*p_1_5[0] + p_1_5[1]*p_1_5[1] + p_1_5[2]*p_1_5[2];
	p1x = sqrt(py*py + px*px);

	// Calculate solutions
	th3_sol(); th6_sol(); th7_sol(); th5_sol(); th1_sol(); th4_sol(); th2_sol();

	buildSolutionSet();
	printSolutionSet();
}

void Iiwa14Inv::solveIKNumerically(Pose *targetPose) {
	// TODO
}

void Iiwa14Inv::buildSolutionSet() {
	for (int i = 0; i < 2; ++i)
		for (int j = 0; j < 2; ++j)
			for (int k = 0; k < 2; ++k)
				solutionSet.push_back({th1[0], th2[k], th3[0], th4[j], th5[0], th6[i], th7[0]});
}

void Iiwa14Inv::printSolutionSet() {
	// TODO: Keep only unique solutions
	cout << "th1 | th2 | th3 | th4 | th5 | th6 | th7" << endl;
	cout << "---------------------------------------" << endl;
	for (int i = 0; i < 8; ++i) {
		for (int j = 0; j < 7; ++j) cout << solutionSet[i][j] << " ";
		cout << endl;
	}
}
