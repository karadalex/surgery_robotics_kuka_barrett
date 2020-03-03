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
	matrixf matrix;
	matrixf inverse;
};


#endif //SRC_TRANSFORMATION_H
