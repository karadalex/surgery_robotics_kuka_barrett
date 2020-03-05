//
// Created by karadalex on 3/3/20.
//

#ifndef SRC_TRANSFORMATION_H
#define SRC_TRANSFORMATION_H

#include <cmath>
#include <vector>
#include "custom_math/Matrix.h"


class Transformation {
public:
	Transformation(float L, float d, float a, float th);
	float L, d, a, th;

	matrixf matrix;
	matrixf inverse;

	Transformation* update(float th);
};


#endif //SRC_TRANSFORMATION_H
