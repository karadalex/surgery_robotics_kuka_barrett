//
// Created by karadalex on 2/3/20.
//

#include "kinematics/Pose.h"

using namespace std;


Pose::Pose(float x, float y, float z, float roll, float pitch, float yaw) {
	matrixf rot_roll = {
		{cos(roll), -sin(roll), 0, 0},
		{sin(roll),  cos(roll), 0, 0},
		{0,              0,     1, 0},
		{0, 0, 0, 0}
	};
	matrixf rot_pitch = {
		{cos(pitch), 0, sin(pitch), 0},
		{0,          1,      0,     0},
		{-sin(pitch),0, cos(pitch), 0},
		{0, 0, 0, 0}
	};
	matrixf rot_yaw = {
		{1,     0,         0,    0},
		{0, cos(yaw), -sin(yaw), 0},
		{0, sin(yaw), cos(yaw),  0},
		{0, 0, 0, 0}
	};

	matrixf rot_rpy = Matrix::mul(rot_roll, rot_pitch);
	rot_rpy = Matrix::mul(rot_rpy, rot_yaw);

	pose = rot_rpy;
	pose[0][3] = x;
	pose[1][3] = y;
	pose[2][3] = z;
	pose[3][3] = 1;

	cout << "Pose matrix = " << endl;
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j)
			cout << pose[i][j] <<" ";
		cout << endl;
	}
}