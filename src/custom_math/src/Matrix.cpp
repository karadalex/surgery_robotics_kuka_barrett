//
// Created by karadalex on 2/3/20.
//

#include <vector>
#include "custom_math/Matrix.h"

using namespace std;

matrixf Matrix::mul(matrixf a, matrixf b) {
	int i, j, k;
	matrixf result;
	result.resize(4, vector<float>(4, 0));

	for(i=0; i<4; ++i)
		for(j=0; j<4; ++j)
			for(k=0; k<4; ++k) {
				result[i][j]+=a[i][k]*b[k][j];
			}

	return result;
}