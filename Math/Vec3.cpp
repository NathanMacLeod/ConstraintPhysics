#include "Vec3.h"

Vec3 operator*(const double d, const Vec3 v) {
	return Vec3(v.x * d, v.y * d, v.z * d);
}

Vec3 operator*(const Vec3 v, const double d) {
	return Vec3(v.x * d, v.y * d, v.z * d);
}

Vec3 operator/(const Vec3 v, const double d) {
	return Vec3(v.x / d, v.y / d, v.z / d);
}