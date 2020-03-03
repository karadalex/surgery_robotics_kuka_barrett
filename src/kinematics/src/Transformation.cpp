//
// Created by karadalex on 3/3/20.
//

#include "kinematics/Transformation.h"

Transformation::Transformation(float L, float d, float a, float th) {
	matrix = {
		{cos(th),            -sin(th),      0,            L},
		{sin(th)*cos(a), cos(th)*cos(a), -sin(a), -sin(a)*d},
		{sin(th)*sin(a), cos(th)*sin(a), cos(a),  cos(a)*d },
		{0,                      0,         0,            1}
	};

	inverse = Matrix::invTransf(matrix);
}
