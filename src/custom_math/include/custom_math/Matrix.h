//
// Created by karadalex on 2/3/20.
//

#ifndef SRC_MATRIX_H
#define SRC_MATRIX_H

#include <iostream>
#include <string>
#include <vector>
#include "custom_math/Scalar.h"

using namespace std;

typedef vector<vector<float>> matrixf;
typedef vector<float> vecf;

class Matrix {
public:
	Matrix();
	static matrixf mul(matrixf a, matrixf b);
	static vecf mul(matrixf a, vecf vec);
	static matrixf invTransf(matrixf m);
	static matrixf pseudoInverse(matrixf m);
	static matrixf eye();
	static matrixf roundf(matrixf m);
	static void printMatrix(matrixf m, string name = "matrix");
};


#endif //SRC_MATRIX_H
