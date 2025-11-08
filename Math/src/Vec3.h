#pragma once
#include <cmath>
#include <cassert>

#include "NVec.h"

namespace mthz {

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

		NVec<3> toNVec() {
			NVec<3> out = {x, y, z};
			return out;
		}

		inline double magSqrd() const {
			return x * x + y * y + z * z;
		}

		inline double mag() const {
			return sqrt(x * x + y * y + z * z);
		}

		inline Vec3 normalize() const {
			double magnitude = mag();
			return (magnitude == 0)? Vec3(0, 0, 0) : Vec3(x / magnitude, y / magnitude, z / magnitude);
		}

		inline Vec3 cross(const Vec3 v) const {
			return Vec3(y * v.z - z * v.y,
				z * v.x - x * v.z,
				x * v.y - y * v.x);
		}

		inline double dot(const Vec3 v) const {
			return x * v.x + y * v.y + z * v.z;
		}

		void getPerpendicularBasis(Vec3* u, Vec3* w) const {
			mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0); //two guesses for non parallel vectors, at least 1 not parallel
			*u = (abs(this->dot(axis1)) < abs(this->dot(axis2))) ? this->cross(axis1).normalize() : this->cross(axis2).normalize();
			*w = this->cross(*u).normalize();
		}

		inline double& operator[](int index) {
			assert(index >= 0 && index < 3);
			if (index == 0)      return x;
			else if (index == 1) return y;
			else                 return z;
			
		}

		inline double operator[](int index) const {
			assert(index >= 0 && index < 3);
			if (index == 0)      return x;
			else if (index == 1) return y;
			else                 return z;
		}

		inline bool operator==(const Vec3 v) const {
			return x == v.x && y == v.y && z == v.z; //need to be carful using == with floats
		}

		inline bool operator!=(const Vec3 v) const {
			return x != v.x || y != v.y || z != v.z; //need to be carful using == with floats
		}

		inline Vec3 operator+(const Vec3 v) const {
			return Vec3(x + v.x, y + v.y, z + v.z);
		}

		inline Vec3 operator-(const Vec3 v) const {
			return Vec3(x - v.x, y - v.y, z - v.z);
		}

		inline Vec3 operator-() const {
			return Vec3(-x, -y, -z);
		}

		inline void operator+=(const Vec3 v) {
			x += v.x;
			y += v.y;
			z += v.z;
		}

		inline void operator-=(const Vec3 v) {
			x -= v.x;
			y -= v.y;
			z -= v.z;
		}

		inline void operator*=(const double d) {
			x *= d;
			y *= d;
			z *= d;
		}

		inline void operator/=(const double d) {
			x /= d;
			y /= d;
			z /= d;
		}

	};

	Vec3 operator*(const double d, const Vec3 v);
	Vec3 operator*(const Vec3 v, const double d);
	Vec3 operator/(const Vec3 v, const double d);

}

