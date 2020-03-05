//
// Created by karadalex on 2/3/20.
//

#include "kinematics/Iiwa14Inv.h"

Iiwa14Inv::Iiwa14Inv(Pose* targetPose) : target(targetPose) {

	initializeForwardKinematicsTransformations();
	M_U_0 = new Transformation(0, 1, 0, 0);
	M_7_TCP = new Transformation(0, 0.205, 0, 0);

	solveIK(targetPose);
}

void Iiwa14Inv::initializeForwardKinematicsTransformations() {
	d = {0.36, 0, 0.42, 0, 0.4, 0, 0};
	a = {0, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2};

	for (int i = 0; i < 7; ++i) {
		Transformation* M_i = new Transformation(0, d[i], a[i], 0);
		fwdTransformations.push_back(M_i);
	}
}

void Iiwa14Inv::th1_sol() {
	float g11g12px = sqrt(g11*g11 + g12*g12 - px*px);
	std::cout << "g11g12px = " << g11g12px << endl;
	float sol1 = atan2(g11,g12) - atan2(g11g12px, px);
	float sol2 = atan2(g11,g12) + atan2(g11g12px, px);
	th1.push_back(Scalar::round(sol1));
	th1.push_back(Scalar::round(sol2));
}

void Iiwa14Inv::th2_sol() {
	th2.push_back(atan2(-s2,c2));
	th2.push_back(atan2(s2,c2));
}

void Iiwa14Inv::th3_sol() {
	th3.push_back(atan2(-s3,c3));
	th3.push_back(atan2(s3,c3));
}

void Iiwa14Inv::th4_sol() {
	th4.push_back(atan2(-s4,c4));
	th4.push_back(atan2(s4,c4));
}

void Iiwa14Inv::th5_sol() {
	th5.push_back(atan2(-s5,ky));
	th5.push_back(atan2(s5,ky));
}

void Iiwa14Inv::th6_sol() {
	th6.push_back(atan2(-s6,c6));
	th6.push_back(atan2(s6,c6));
}

void Iiwa14Inv::updateForwardKinematics(float *th) {
	for (int i = 0; i < 7; ++i) {
		fwdTransformations.at(i)->update(th[i]);
	}
}

void Iiwa14Inv::solveIK(Pose* targetPose) {

	matrixf M_1_7 = targetPose->pose;
	M_1_7 = Matrix::mul(M_1_7, M_7_TCP->inverse);
	M_1_7 = Matrix::mul(M_U_0->inverse, M_1_7);
	M_1_7 = Matrix::mul(fwdTransformations.at(0)->inverse, M_1_7);
	Matrix::printMatrix(M_1_7, "M_1_7");

	ky = M_1_7[1][2];
	s5 = sqrt(1-ky*ky);

	kx = M_1_7[0][2];
	c4 = kx / sqrt(1-ky*ky);
	s4 = sqrt(1 - c4*c4);

	iy = M_1_7[1][0];
	c6 = iy / sqrt(1-ky*ky);
	s6 = sqrt(1 - c6*c6);

	px = M_1_7[0][3];
	py = M_1_7[1][3];
	pz = M_1_7[2][3];

	c3 = (px*px + py*py + pz*pz) / 0.336;
	s3 = sqrt(1 - c3*c3);
	std::cout << "c3 = " << c3 << endl;

	s2 = py / (0.4 * s3);
	c2 = sqrt(1 - s2*s2);

	g11 = 0.42 + 0.4*c3;
	g12 = 0.4*c2*s3;
	std::cout << "g11 = " << g11 << endl;

	// Calculate solutions
	th5_sol(); th4_sol(); th6_sol(); th3_sol(); th2_sol(); th1_sol();
}

void Iiwa14Inv::buildSolutionSet() {

}
