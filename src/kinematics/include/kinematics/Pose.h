//
// Created by karadalex on 2/3/20.
//

#ifndef SRC_POSE_H
#define SRC_POSE_H

#include <cmath>
#include <vector>
#include "custom_math/Matrix.h"


class Pose {
	public:
		Pose(float x, float y, float z, float roll, float pitch, float yaw);
		float pose[4][4];
};


#endif //SRC_POSE_H
