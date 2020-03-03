//
// Created by karadalex on 2/3/20.
//

#include "kinematics/Iiwa14Inv.h"

Iiwa14Inv::Iiwa14Inv(Pose* targetPose) : target(targetPose) {
	ky = target->pose[1][2];
	s5 = sqrt(1-ky*ky);

	kx = target->pose[0][2];
	c4 = kx / sqrt(1-ky*ky);
	s4 = sqrt(1 - c4*c4);

	iy = target->pose[1][0];
	c6 = iy / sqrt(1-ky*ky);
	s6 = sqrt(1 - c6*c6);

	px = target->pose[0][3];
	py = target->pose[1][3];
	pz = target->pose[2][3];

	c3 = (px*px + py*py + pz*pz) / 0.336;
	s3 = sqrt(1 - c3*c3);

	s2 = py / (0.4 * s3);
	c2 = sqrt(1 - s2*s2);

	g11 = 0.42 + 0.4*c3;
	g12 = 0.4*c2*s3;

  // Calculate solutions
  th5_sol(); th4_sol(); th6_sol(); th3_sol(); th2_sol(); th1_sol();
}

void Iiwa14Inv::th1_sol() {
	float g11g12px = sqrt(g11*g11 + g12*g12 - px*px);
	float sol1 = atan2(g11,g12) - atan2(g11g12px, px);
	float sol2 = atan2(g11,g12) + atan2(g11g12px, px);
	th1.push_back(sol1);
	th1.push_back(sol2);
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
