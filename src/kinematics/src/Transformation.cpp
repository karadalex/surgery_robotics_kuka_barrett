//
// Created by karadalex on 3/3/20.
//

#include "kinematics/Transformation.h"

Transformation::Transformation(float L, float d, float a, float th) : L(L), d(d), a(a), th(th) {
	update(th);
}

Transformation* Transformation::update(float _th) {
	th = _th;
	matrix = {
		{cos(th),            -sin(th),      0,            L},
		{sin(th)*cos(a), cos(th)*cos(a), -sin(a), -sin(a)*d},
		{sin(th)*sin(a), cos(th)*sin(a), cos(a),  cos(a)*d },
		{0,                      0,         0,            1}
	};

	inverse = Matrix::invTransf(matrix);

	return this;
}

matrixf Transformation::Roll(float roll) {
	return {
		{cos(roll), -sin(roll), 0, 0},
		{sin(roll),  cos(roll), 0, 0},
		{0,              0,     1, 0},
		{0, 0, 0, 0}
	};
}

matrixf Transformation::Pitch(float pitch) {
	return {
		{cos(pitch), 0, sin(pitch), 0},
		{0,          1,      0,     0},
		{-sin(pitch),0, cos(pitch), 0},
		{0, 0, 0, 0}
	};
}

matrixf Transformation::Yaw(float yaw) {
	return {
		{1,     0,         0,    0},
		{0, cos(yaw), -sin(yaw), 0},
		{0, sin(yaw), cos(yaw),  0},
		{0, 0, 0, 0}
	};
}

matrixf Transformation::Translation(vecf p) {
	return {
		{1, 0, 0, p[0]},
		{0, 1, 0, p[1]},
		{0, 0, 1, p[2]},
		{0, 0, 0, 1}
	};
}
