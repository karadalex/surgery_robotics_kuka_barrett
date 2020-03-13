//
// Created by karadalex on 2/3/20.
//

#include "kinematics/Pose.h"

using namespace std;


Pose::Pose(float x, float y, float z, float roll, float pitch, float yaw) {
	matrixf rot_roll = Transformation::Roll(roll);
	matrixf rot_pitch = Transformation::Pitch(pitch);
	matrixf rot_yaw = Transformation::Yaw(yaw);

	matrixf rot_rpy = Matrix::mul(rot_roll, rot_pitch);
	rot_rpy = Matrix::mul(rot_rpy, rot_yaw);

	pose = rot_rpy;
	pose[0][3] = x;
	pose[1][3] = y;
	pose[2][3] = z;
	pose[3][3] = 1;
}