//
// Created by karadalex on 5/3/20.
//

#include "custom_math/Scalar.h"

float Scalar::round(float scalar) {
	float precision = 1000;
	float value = (int)(scalar * precision + 0.5);
	return (float)value / precision;
}

float Scalar::clamp(float scalar, float low, float high) {
	if (scalar < low) {
		return low;
	} else if (scalar > high) {
		return high;
	} else {
		return scalar;
	}
}

float Scalar::normalise(float value, float lowBoundary, float highBoundary)
{
	float width = highBoundary - lowBoundary;
	float offsetValue = value - lowBoundary ;

	return offsetValue - (floorf(offsetValue / width) * width) + lowBoundary ;
}
