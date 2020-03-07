//
// Created by karadalex on 5/3/20.
//

#ifndef SRC_SCALAR_H
#define SRC_SCALAR_H

#include <cmath>


class Scalar {
public:
	static float round(float scalar);
	static float clamp(float scalar, float low, float high);
	static float normalise(float value, float lowBoundary, float highBoundary);
};


#endif //SRC_SCALAR_H
