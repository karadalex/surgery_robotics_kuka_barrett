//
// Created by karadalex on 2/3/20.
//

// #include "kinematics/Iiwa14Inv.h"

// Iiwa14Inv::Iiwa14Inv(Pose* targetPose) : target(targetPose) {

// 	initializeForwardKinematicsTransformations();
// 	M_U_0 = new Transformation(0, 1, 0, 0);
// 	M_7_TCP = (new Transformation(0, -0.119, 0, 3*M_PI_2))->matrix;

// 	solveIK(targetPose);
// }

// void Iiwa14Inv::initializeForwardKinematicsTransformations() {
// 	// Angle th3 is skipped, because it is used as a redundant degree of freedom!
// 	d = {0.36, 0, 0, 0.4, 0, 0};
// 	L = {0, 0, 0.42, 0, 0, 0};
// 	a = {0, -M_PI/2, 0, -M_PI/2, M_PI/2, -M_PI/2};

// 	for (int i = 0; i < 6; ++i) {
// 		Transformation* M_i = new Transformation(L[i], d[i], a[i], 0);
// 		fwdTransformations.push_back(M_i);
// 	}
// }

// void Iiwa14Inv::th1_sol() {
// 	float sol = atan2(py, px);
// 	// float sol = atan2(p_1_7[1], p_1_7[0]);
// 	// Check for singularity.
// 	// If th1 is not defined (px=py=0) then th1=0
// 	if (!isnan(sol)) th1.push_back(sol); else th1.push_back(0);
// }

// void Iiwa14Inv::th2_sol() {
// 	// float phi = acosf((L[2]*L[2] + p_1_7_len*p_1_7_len - D*D)/ (2*L[2]*p_1_7_len));
// 	float phi = atan2(D*s4, L[2]+D*c4);
// 	// Check for singularity
// 	// If phi is not defined (fully extended) then phi = 0
// 	if (isnan(phi)) phi = 0;
// 	float sol1 = atan2(p_2_5x, p_1_7[2]) + phi;
// 	float sol2 = atan2(p_2_5x, p_1_7[2]) - phi;
// 	th2.push_back(sol1);
// 	th2.push_back(sol2);
// }

// void Iiwa14Inv::th3_sol() {
// 	th3.push_back(0);
// }

// void Iiwa14Inv::th4_sol() {
// 	c4 = (p_1_7_len*p_1_7_len - L[2]*L[2] - D*D) / (2 * L[2] * D);
// 	// -1 <= c4 <= 1
// 	c4 = Scalar::clamp(c4, -1, 1);
// 	s4 = sqrt(1 - c4*c4);
// 	// Check for singularity.
// 	// If c4=0, then atan2 is not defined and th4=pi/2
// 	if (c4 != 0.0) {
// 		th4.push_back(atan2(s4, c4));
// 		th4.push_back(atan2(-s4, c4));
// 	} else {
// 		th4.push_back(0);
// 		th4.push_back(0);
// 	}
// }

// void Iiwa14Inv::th5_sol() {
// 	if (s6 < 0) th5.push_back(atan2(-kz, -kx));
// 	else th5.push_back(atan2(kz, kx));
// }

// void Iiwa14Inv::th6_sol() {
// 	s6 = sqrt(kx*kx + kz*kz);
// //	if (isnan(s6)) s6 = 0;
// 	th6.push_back(atan2(s6, -ky));
// 	th6.push_back(atan2(s6, ky));
// }

// void Iiwa14Inv::th7_sol() {
// 	if (s6 < 0) th7.push_back(atan2(jy, -iy));
// 	else th7.push_back(atan2(-jy, iy));
// }

// void Iiwa14Inv::updateForwardKinematics(vecf th) {
// 	for (int i = 0; i < 6; ++i) {
// 		fwdTransformations.at(i)->update(th[i]);
// 	}
// }

// void Iiwa14Inv::solveIK(Pose* targetPose) {

// 	M_0_7 = targetPose->pose;
// 	matrixf M_7_TCP_inv = Matrix::invTransf(M_7_TCP);
// 	M_0_7 = Matrix::mul(M_0_7, M_7_TCP_inv);
// 	M_0_7 = Matrix::mul(M_U_0->inverse, M_0_7);
// 	Matrix::printMatrix(M_0_7, "M_0_7");

// 	px = M_0_7[0][3];
// 	py = M_0_7[1][3];
// 	pz = M_0_7[2][3];
// 	p_0_7 = {px, py, pz, 1};
// 	p_1_7 = p_0_7; p_1_7[2] -= d[0];

// 	p_1_7_len = sqrt(p_1_7[0]*p_1_7[0] + p_1_7[1]*p_1_7[1] + p_1_7[2]*p_1_7[2]);
// 	p_2_5x = sqrt(p_1_7[0]*p_1_7[0] + p_1_7[1]*p_1_7[1]);
// 	p1x = sqrt(py*py + px*px);
// 	D = d[3];

// 	// Calculate solutions for position
// 	th1_sol(); th4_sol(); th2_sol();

// 	// Redundant degree of freedom
// 	th3_sol();

// 	// Calculate solutions for orientation
// 	vecf temp_th = {th1[0], th2[0], th4[0], 0, 0, 0};
// 	updateForwardKinematics(temp_th);
// 	matrixf M_0_3 = Matrix::eye();
// 	for (int i = 0; i < 3; ++i) {
// 		M_0_3 = Matrix::mul(M_0_3, fwdTransformations.at(i)->matrix);
// 	}
// 	matrixf M_3_7 = Matrix::mul(Matrix::invTransf(M_0_3), M_0_7);
// 	ix = M_3_7[0][0]; iy = M_3_7[1][0]; iz = M_3_7[2][0];
// 	jx = M_3_7[0][1]; jy = M_3_7[1][1]; jz = M_3_7[2][1];
// 	kx = M_3_7[0][2]; ky = M_3_7[1][2]; kz = M_3_7[2][2];
// 	th6_sol(); th7_sol(); th5_sol();

// 	buildSolutionSet();
// 	printSolutionSet();
// }

// void Iiwa14Inv::solveIKNumerically(Pose *targetPose) {
// 	// TODO
// }

// void Iiwa14Inv::buildSolutionSet() {
// 	for (int i = 0; i < 2; ++i)
// 		for (int j = 0; j < 2; ++j)
// 			for (int k = 0; k < 2; ++k)
// 				solutionSet.push_back({th1[0], th2[k], th3[0], th4[j], th5[0], th6[i], th7[0]});
// }

// void Iiwa14Inv::printSolutionSet() {
// 	// TODO: Keep only unique solutions
// 	cout << "th1 | th2 | th3 | th4 | th5 | th6 | th7" << endl;
// 	cout << "---------------------------------------" << endl;
// 	for (int i = 0; i < 8; ++i) {
// 		for (int j = 0; j < 7; ++j) cout << solutionSet[i][j] << " ";
// 		cout << endl;
// 	}
// }

// void Iiwa14Inv::validateSolution() {
// 	for (int j = 0; j < 8; ++j) {
// 		vecf th = solutionSet.at(j);
// 		updateForwardKinematics(th);
// 		matrixf pose = Matrix::eye();
// 		for (int i = 0; i < 6; ++i) { // only for 6 DoF
// 			pose = Matrix::mul(pose, fwdTransformations.at(i)->matrix);
// 		}
// 		// pose = Matrix::mul(pose, M_7_TCP);
// 		cout << "Solution " << j << endl;
// 		Matrix::printMatrix(pose, "forward Kinematics Validation Pose Sol");
// 	}
// }



#include "kinematics/Iiwa14Inv.h"

using namespace std;
using namespace Eigen;

Iiwa14Inv::Iiwa14Inv(Pose* targetPose) : target(targetPose) {
	d = {0.36, 0, 0.42, 0, 0.4, 0, 0};
	L = {0, 0, 0, 0, 0, 0, 0};
	a = {0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2};

	initializeForwardKinematicsTransformations();

	M_U_0 = (new Pose(0,0,1,0,0,0))->pose;
	// M_7_TCP = (new Transformation(0, 0.245, 0, 0))->matrix;
//	M_7_TCP = (new Transformation(0, 0.245, 0, 0))->matrix;
	 M_7_TCP = (new Pose(0,0,0.245,0,0,0))->pose;

	solveIK(targetPose);

	printSolutionSet();
}

void Iiwa14Inv::initializeForwardKinematicsTransformations() {
	for (int i = 0; i < 7; ++i) {
		Transformation* M_i = new Transformation(L[i], d[i], a[i], 0);
		fwdTransformations.push_back(M_i);
	}
}

void Iiwa14Inv::solveIK(Pose* targetPose) {
	Matrix4d M_U_TCP = targetPose->pose;
	cout << "M_U_TCP desired = \n" << M_U_TCP << endl;
	M_0_7 = M_U_0.inverse() * M_U_TCP * M_7_TCP.inverse();
	px = M_0_7(0,3);
	py = M_0_7(1,3);
	pz = M_0_7(2,3);

	// Solve for position: th1, th2, th4
	// Solve for th1
	double pxy = sqrt(px*px + py*py);
	th1.push_back(atan2(py/pxy, px/pxy));
	th1.push_back(atan2(py/-pxy, px/-pxy));

	// Substitute th1 in forward transformation matrix
	Matrix4d M_0_1 = fwdTransformations.at(0)->update(th1.at(0))->matrix;
	Vector4d p_0_5 = M_0_7.col(3);
	cout << "p_0_5 = " << p_0_5 << endl;
	Vector4d p_1_5 = M_0_1.inverse() * p_0_5;
	double p_1_5_len = sqrt(p_1_5(0)*p_1_5(0) + p_1_5(1)*p_1_5(1) + p_1_5(2)*p_1_5(2));

	// Solve for th4
	double c4 = (p_1_5_len*p_1_5_len - d[2]*d[2] - d[4]*d[4]) / (2*d[2]*d[4]);
	double s4 = sqrt(1 - c4*c4);
	cout << "s4 = " << s4 << endl;
	th4.push_back(atan2(s4, c4));
	th4.push_back(atan2(-s4, c4));

	// Solve for th2
	double phi = atan2(d[4]*s4, d[2]+d[4]*c4);
	th2.push_back(atan2(sqrt(px*px + py*py), p_1_5(2)) + phi);
	th2.push_back(atan2(sqrt(px*px + py*py), p_1_5(2)) - phi);

	// Redundant degree of freedom
	th3.push_back(0);

	// Solve for orientation
	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			// Calculate M_0_4
			// TODO: FInd a way to get result with fewer calculations
			M_0_1 = fwdTransformations.at(0)->update(th1.at(i))->matrix;
			Matrix4d M_1_2 = fwdTransformations.at(1)->update(th2.at(j))->matrix;
			Matrix4d M_2_3 = fwdTransformations.at(2)->matrix;
			Matrix4d M_3_4 = fwdTransformations.at(3)->update(th4.at(j))->matrix;
			Matrix4d M_0_4 = M_0_1 * M_1_2 * M_2_3 * M_3_4;

			Matrix3d R_0_4 = M_0_4.block(0,0,3,3);
			Matrix3d R_0_7 = M_0_7.block(0,0,3,3);
			Matrix3d R_4_7 = R_0_4.inverse() * R_0_7;

			iy = R_4_7(1,0);
			jy = R_4_7(1,1);
			kx = R_4_7(0,2);
			ky = R_4_7(1,2);
			kz = R_4_7(2,2);

			double s6 = sqrt(kx*kx + kz*kz);
			th6.push_back(atan2(s6, ky));
			th6.push_back(atan2(-s6, ky));

			th7.push_back(atan2(jy, iy));  // corresponds to th6[0] where s6>0
			th7.push_back(atan2(-jy, iy)); // corresponds to th6[1] where s6<0

			th5.push_back(atan2(-kz, kx));  // corresponds to th6[0] where s6>0
			th5.push_back(atan2(kz, -kx)); // corresponds to th6[1] where s6<0

		}
	}

	// Build solution set (8 solutions in total)
	buildSolutionSet();

}

void Iiwa14Inv::updateForwardKinematics(vecf th) {
	
}

void Iiwa14Inv::solveIKNumerically(Pose *targetPose) {
	// TODO
}

void Iiwa14Inv::buildSolutionSet() {
	for (int i = 0; i < 2; ++i)
		for (int j = 0; j < 2; ++j)
			for (int k = 0; k < 2; ++k)
				solutionSet.push_back({th1[i], th2[j], th3[0], th4[j], th5[k], th6[k], th7[k]});
}

void Iiwa14Inv::printSolutionSet() {
	// TODO: Keep only unique solutions
	cout << "th1 | th2 | th3 | th4 | th5 | th6 | th7" << endl;
	cout << "---------------------------------------" << endl;
	MatrixXd S(8,7);
	for (int i = 0; i < 8; ++i) {
		for (int j = 0; j < 7; ++j) {
			S(i,j) = solutionSet[i][j];
		}
	}
	cout << S << endl;
	
}

void Iiwa14Inv::validateSolution(vector<double> angles) {
	Matrix4d M_0_7_test = Matrix4d::Identity();
	for (int i = 0; i < 7; ++i) {
		Matrix4d M_i = (new Transformation(L[i], d[i], a[i], angles[i]))->matrix;
		M_0_7_test = M_0_7_test * M_i;
	}
	Matrix4d M_U_TCP_test = M_U_0 * M_0_7_test * M_7_TCP;
	cout << "M_U_TCP_test = \n" << M_U_TCP_test << endl;
}