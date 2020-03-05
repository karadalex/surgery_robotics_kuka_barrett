//
// Created by karadalex on 5/3/20.
//

#include "custom_math/Scalar.h"

float Scalar::round(float scalar) {
	float precision = 10.0;
	float value = (int)(scalar * precision + 0.5);
	return (float)value / precision;
}
