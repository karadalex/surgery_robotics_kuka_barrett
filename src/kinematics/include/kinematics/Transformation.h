//
// Created by karadalex on 3/3/20.
//

#ifndef SRC_TRANSFORMATION_H
#define SRC_TRANSFORMATION_H

#include <cmath>
#include <vector>
#include "Eigen/Dense"
#include "custom_math/Matrix.h"


using namespace Eigen;

class Transformation {
public:
	Transformation(float L, float d, float a, float th);
	float L, d, a, th;

	Matrix4d matrix;
	Matrix4d inverse;

	static Matrix4d Roll(float roll);
	static Matrix4d Pitch(float pitch);
	static Matrix4d Yaw(float yaw);
	static Matrix4d Translation(vecf p);

	Transformation* update(float th);
};


#endif //SRC_TRANSFORMATION_H
