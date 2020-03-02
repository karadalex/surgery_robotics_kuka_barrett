//
// Created by karadalex on 2/3/20.
//

#ifndef SRC_MATRIX_H
#define SRC_MATRIX_H

#include <iostream>
#include <vector>

using namespace std;

typedef vector<vector<float>> matrixf;

class Matrix {
public:
	Matrix();
	static matrixf mul(matrixf a, matrixf b);
};


#endif //SRC_MATRIX_H
