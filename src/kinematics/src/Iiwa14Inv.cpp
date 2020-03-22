//
// Created by karadalex on 2/3/20.
//

#include "kinematics/Iiwa14Inv.h"

Iiwa14Inv::Iiwa14Inv(Pose* targetPose) : target(targetPose) {

	initializeForwardKinematicsTransformations();
	M_U_0 = new Transformation(0, 1, 0, 0);
	M_7_TCP = (new Transformation(0, 0.0795, 0, 3*M_PI_2))->matrix;

	solveIK(targetPose);
}

void Iiwa14Inv::initializeForwardKinematicsTransformations() {
	// Angle th3 is skipped, because it is used as a redundant degree of freedom!
	d = {0.36, 0, 0, 0.4, 0, 0};
	L = {0, 0, 0.42, 0, 0, 0};
	a = {0, -M_PI/2, 0, -M_PI/2, M_PI/2, -M_PI/2};

	for (int i = 0; i < 6; ++i) {
		Transformation* M_i = new Transformation(L[i], d[i], a[i], 0);
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
	// float phi = acosf((L[2]*L[2] + p_1_7_len*p_1_7_len - D*D)/ (2*L[2]*p_1_7_len));
	float phi = atan2(D*s4, L[2]+D*c4);
	// Check for singularity
	// If phi is not defined (fully extended) then phi = 0
	if (isnan(phi)) phi = 0;
	float sol1 = atan2(p_2_5x, p_1_7[2]) + phi;
	float sol2 = atan2(p_2_5x, p_1_7[2]) - phi;
	th2.push_back(sol1);
	th2.push_back(sol2);
}

void Iiwa14Inv::th3_sol() {
	th3.push_back(0);
}

void Iiwa14Inv::th4_sol() {
	c4 = (p_1_7_len*p_1_7_len - L[2]*L[2] - D*D) / (2 * L[2] * D);
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
	if (s6 < 0) th5.push_back(atan2(-kz, -kx));
	else th5.push_back(atan2(kz, kx));
}

void Iiwa14Inv::th6_sol() {
	s6 = sqrt(kx*kx + kz*kz);
//	if (isnan(s6)) s6 = 0;
	th6.push_back(atan2(s6, -ky));
	th6.push_back(atan2(s6, ky));
}

void Iiwa14Inv::th7_sol() {
	if (s6 < 0) th7.push_back(atan2(jy, -iy));
	else th7.push_back(atan2(-jy, iy));
}

void Iiwa14Inv::updateForwardKinematics(vecf th) {
	for (int i = 0; i < 6; ++i) {
		fwdTransformations.at(i)->update(th[i]);
	}
}

void Iiwa14Inv::solveIK(Pose* targetPose) {

	M_0_7 = targetPose->pose;
	matrixf M_7_TCP_inv = Matrix::invTransf(M_7_TCP);
	M_0_7 = Matrix::mul(M_0_7, M_7_TCP_inv);
	M_0_7 = Matrix::mul(M_U_0->inverse, M_0_7);
	Matrix::printMatrix(M_0_7, "M_0_7");

	px = M_0_7[0][3];
	py = M_0_7[1][3];
	pz = M_0_7[2][3];
	vecf p_0_7 = {px, py, pz, 1};

	matrixf R_5_6 = fwdTransformations.at(4)->matrix;
	R_5_6[0][3] = R_5_6[1][3] = R_5_6[2][3] = 0;
	vecf p_6_7 = {0, 0, 0.08, 1};
	p_5_7 = Matrix::mul(R_5_6, p_6_7);

	// p_1_7 is used in th2_sol and th4_sol methods
	p_1_7 = Matrix::mul(fwdTransformations.at(0)->inverse, p_0_7);
	p_1_7_len = sqrt(p_1_7[0]*p_1_7[0] + p_1_7[1]*p_1_7[1] + p_1_7[2]*p_1_7[2]);
	p_2_5x = sqrt(p_1_7[0]*p_1_7[0] + p_1_7[1]*p_1_7[1]);
	p1x = sqrt(py*py + px*px);
	float p_5_7_len = sqrt(p_5_7[0]*p_5_7[0] + p_5_7[1]*p_5_7[1] + p_5_7[2]*p_5_7[2]);
	// D = d[3] + p_5_7_len;
	D = d[3] + 0.08;

	// Calculate solutions for position
	th1_sol(); th4_sol(); th2_sol();

	// Redundant degree of freedom
	th3_sol();

	// Calculate solutions for orientation
	vecf temp_th = {th1[0], th2[0], th4[0], 0, 0, 0};
	updateForwardKinematics(temp_th);
	matrixf M_0_3 = Matrix::eye();
	for (int i = 0; i < 3; ++i) {
		M_0_3 = Matrix::mul(M_0_3, fwdTransformations.at(i)->matrix);
	}
	matrixf M_3_7 = Matrix::mul(Matrix::invTransf(M_0_3), M_0_7);
//	matrixf M_3_7 = M_0_7;
	ix = M_3_7[0][0]; iy = M_3_7[1][0]; iz = M_3_7[2][0];
	jx = M_3_7[0][1]; jy = M_3_7[1][1]; jz = M_3_7[2][1];
	kx = M_3_7[0][2]; ky = M_3_7[1][2]; kz = M_3_7[2][2];
	th6_sol(); th7_sol(); th5_sol();

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

void Iiwa14Inv::validateSolution() {
	for (int j = 0; j < 8; ++j) {
		vecf th = solutionSet.at(j);
		updateForwardKinematics(th);
		matrixf pose = Matrix::eye();
		for (int i = 0; i < 6; ++i) { // only for 6 DoF
			pose = Matrix::mul(pose, fwdTransformations.at(i)->matrix);
		}
		// pose = Matrix::mul(pose, M_7_TCP);
		cout << "Solution " << j << endl;
		Matrix::printMatrix(pose, "forward Kinematics Validation Pose Sol");
	}
}
