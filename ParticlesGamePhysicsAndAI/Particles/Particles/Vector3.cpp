/*
 *	Particle system demo - class for a vector in 3-dimensional space
 */
#include "Vector3.h"

Vector3 operator*(float s, const Vector3 &v) {
	return v*s;
}
