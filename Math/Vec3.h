#pragma once
#include <cmath>

class Vec3 {
public:
	double x, y, z;

	Vec3() {
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}

	Vec3(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	double magSqrd() {
		return x * x + y * y + z * z;
	}
	
	double mag() {
		return sqrt(x * x + y * y + z * z);
	}

	Vec3 normalize() {
		double magnitude = mag();
		return Vec3(x / magnitude, y / magnitude, z / magnitude);
	}

	Vec3 cross(const Vec3 v) {
		return Vec3(y * v.z - z * v.y,
					z * v.x - x * v.z,
					x * v.y - y * v.x);
	}

	double dot(const Vec3 v) {
		return x * v.x + y * v.y + z * v.z;
	}

	bool operator==(const Vec3 v) {
		return x == v.x && y == v.y && z == v.z; //need to be carful using == with floats
	}

	Vec3 operator+(const Vec3 v) {
		return Vec3(x + v.x, y + v.y, z + v.z);
	}

	Vec3 operator-(const Vec3 v) {
		return Vec3(x - v.x, y - v.y, z - v.z);
	}

	void operator+=(const Vec3 v) {
		x += v.x;
		y += v.y;
		z += v.z;
	}

	void operator-=(const Vec3 v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}

	void operator*=(const double d) {
		x *= d;
		y *= d;
		z *= d;
	}

	void operator/=(const double d) {
		x /= d;
		y /= d;
		z /= d;
	}

};

Vec3 operator*(const double d, const Vec3 v);
Vec3 operator*(const Vec3 v, const double d);
Vec3 operator/(const Vec3 v, const double d);


