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
			for(k=0; k<4; ++k)
				result[i][j]+=a[i][k]*b[k][j];

	return roundf(result);
}

matrixf Matrix::invTransf(matrixf m) {
	int i, j ,k;
	matrixf result;
	result.resize(4, vector<float>(4, 0));

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; ++j) {
			result[i][j] = m[j][i];
		}
	}
	for (i = 0; i < 3; i++) {
		result[i][3] = -m[i][3];
	}
	result[3][3] = 1;

	return roundf(result);
}

void Matrix::printMatrix(matrixf m, string name) {
	cout << name << " = " << endl;
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j)
			cout << m[i][j] <<" ";
		cout << endl;
	}

}

matrixf Matrix::eye() {
	matrixf eye;
	eye.resize(4, vector<float>(4, 0));
	for (int i = 0; i < 4; ++i) {
		eye[i][i] = 1;
	}
	return eye;
}

matrixf Matrix::roundf(matrixf m) {
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; ++j)
			m[i][j] = Scalar::round(m[i][j]);
	return m;
}
