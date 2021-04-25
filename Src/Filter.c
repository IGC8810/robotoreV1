#include "Filter.h"

float ComplementaryFilter(float high_cut, float low_cut, float alpha, float complement_before) {
	float complement;

	complement = alpha * (complement_before + high_cut * DELTA_T) + (1.0f - alpha) * low_cut;

	return complement;
}

/*
 * LowPassFilter() {
 *
 * }
 */
