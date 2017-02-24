#ifndef VECTOR3_H
#define VECTOR3_H
/*
 *	Particle system demo - class for a vector in 3-dimensional space
 */

#include <cmath>

class Vector3 {
public:
	float	x, y, z;

	Vector3() : x(0), y(0), z(0) {}

	Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

	// Vector addition and subtraction
	Vector3 & operator+=(const Vector3 &o) {
		x += o.x; y += o.y; z += o.z;
		return *this;
	}

	Vector3 operator+(const Vector3 &o) const {
		return Vector3(x + o.x, y + o.y, z + o.z);
	}

	Vector3 & operator-=(const Vector3 &o) {
		x -= o.x; y -= o.y; z -= o.z;
		return *this;
	}

	Vector3 operator-(const Vector3 &o) const {
		return Vector3(x - o.x, y - o.y, z - o.z);
	}

	// Scalar multiplication
	Vector3 & operator*=(float s) {
		x *= s; y *= s; z *= s;
		return *this;
	}

	Vector3 operator*(float s) const {
		return Vector3(x * s, y * s, z * s);
	}

	Vector3 & operator/=(float s) {
		x /= s; y /= s; z /= s;
		return *this;
	}

	Vector3 operator/(float s) const {
		return Vector3(x / s, y / s, z / s);
	}

	// Cross product
	Vector3 &operator*(const Vector3 &o) {
		*this = *this * o;
        return *this;
	}

	Vector3 operator*(const Vector3 &o) const {
		return Vector3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
	}

	// Dot product
	float dot(const Vector3 &o) const {
		return x*o.x + y*o.y + z*o.z;
	}

	// Magnitude of the vector
	float length() const {
		return sqrt(x*x + y*y + z*z);
	}
};

// Scalar multiplication
extern Vector3 operator*(float s, const Vector3 &v);

#endif
