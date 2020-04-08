//
// Created by karadalex on 3/3/20.
//

#include "kinematics/Transformation.h"


Transformation::Transformation(float L, float d, float a, float th) : L(L), d(d), a(a), th(th) {
	update(th);
}

Transformation* Transformation::update(float _th) {
	th = _th;
	matrix <<
		cos(th),            -sin(th),      0,            L,
		sin(th)*cos(a), cos(th)*cos(a), -sin(a), -sin(a)*d,
		sin(th)*sin(a), cos(th)*sin(a), cos(a),  cos(a)*d ,
		0,                      0,         0,            1
	;

	// inverse = Matrix::invTransf(matrix);
 	inverse = matrix.inverse();

	return this;
}

Matrix4d Transformation::Roll(float roll) {
	Matrix4d M;
	M <<
		cos(roll), -sin(roll), 0, 0,
		sin(roll),  cos(roll), 0, 0,
		0,              0,     1, 0,
		0, 0, 0, 1
	;
	return M;
}

Matrix4d Transformation::Pitch(float pitch) {
	Matrix4d M;
	M <<
		cos(pitch), 0, sin(pitch), 0,
		0,          1,      0,     0,
		-sin(pitch),0, cos(pitch), 0,
		0, 0, 0, 1
	;
	return M;
}

Matrix4d Transformation::Yaw(float yaw) {
	Matrix4d M;
	M <<
		1,     0,         0,    0,
		0, cos(yaw), -sin(yaw), 0,
		0, sin(yaw), cos(yaw),  0,
		0, 0, 0, 1
	;
	return M;
}

Matrix4d Transformation::Translation(vecf p) {
	Matrix4d M;
	M <<
		1, 0, 0, p[0],
		0, 1, 0, p[1],
		0, 0, 1, p[2],
		0, 0, 0, 1
	;
	return M;
}
