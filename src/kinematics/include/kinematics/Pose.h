//
// Created by karadalex on 2/3/20.
//

#ifndef SRC_POSE_H
#define SRC_POSE_H

#include <cmath>
#include <vector>
#include "Eigen/Dense"
#include "custom_math/Matrix.h"
#include "kinematics/Transformation.h"


class Pose {
	public:
		Pose(float x, float y, float z, float roll, float pitch, float yaw);
		Matrix4d pose;
};


#endif //SRC_POSE_H
