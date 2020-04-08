//
// Created by karadalex on 2/3/20.
//

#include "kinematics/Pose.h"

using namespace std;


Pose::Pose(float x, float y, float z, float roll, float pitch, float yaw) {
	Matrix4d rot_roll = Transformation::Roll(roll);
	Matrix4d rot_pitch = Transformation::Pitch(pitch);
	Matrix4d rot_yaw = Transformation::Yaw(yaw);

	Matrix4d rot_rpy = rot_roll * rot_pitch * rot_yaw;

	pose = rot_rpy;
	pose(0,3) = x;
	pose(1,3) = y;
	pose(2, 3) = z;
	pose(3, 3) = 1;
}