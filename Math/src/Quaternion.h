#pragma once
#include "Vec3.h"
#include "Mat3.h"

namespace mthz {

	class Quaternion {
	public:
		double r, i, j, k;

		Quaternion() {
			r = 1;
			i = 0;
			j = 0;
			k = 0;
		}

		Quaternion(double r, double i, double j, double k) {
			this->r = r;
			this->i = i;
			this->j = j;
			this->k = k;
		}

		Quaternion(double theta, Vec3 rot_axis) {
			rot_axis = rot_axis.normalize(); //needs to be unit vector
			r = cos(theta / 2.0); //quaternion multiplication doubles the rotation angle
			double sin_theta = sin(theta / 2.0);
			i = rot_axis.x * sin_theta;
			j = rot_axis.y * sin_theta;
			k = rot_axis.z * sin_theta;
		}

		Vec3 applyRotation(Vec3 v) const {
			return Vec3(v.x * (r * r + i * i - j * j - k * k) + v.y * (2 * i * j - 2 * r * k) + v.z * (2 * i * k + 2 * r * j),
				v.x * (2 * i * j + 2 * r * k) + v.y * (r * r - i * i + j * j - k * k) + v.z * (2 * j * k - 2 * r * i),
				v.x * (2 * i * k - 2 * r * j) + v.y * (2 * j * k + 2 * r * i) + v.z * (r * r - i * i - j * j + k * k));
		}

		Mat3 getRotMatrix() const {
			Mat3 mat;
			mat.v[0][0] = r * r + i * i - j * j - k * k; mat.v[0][1] = 2 * i * j - 2 * r * k;         mat.v[0][2] = 2 * i * k + 2 * r * j;
			mat.v[1][0] = 2 * i * j + 2 * r * k;         mat.v[1][1] = r * r - i * i + j * j - k * k; mat.v[1][2] = 2 * j * k - 2 * r * i;
			mat.v[2][0] = 2 * i * k - 2 * r * j;         mat.v[2][1] = 2 * j * k + 2 * r * i;         mat.v[2][2] = r * r - i * i - j * j + k * k;
			return mat;
		}

		Quaternion conjugate() const {
			return Quaternion(r, -i, -j, -k);
		}

		Quaternion operator*(const Quaternion q) {
			return Quaternion(r * q.r - i * q.i - j * q.j - k * q.k,
				r * q.i + i * q.r + j * q.k - k * q.j,
				r * q.j - i * q.k + j * q.r + k * q.i,
				r * q.k + i * q.j - j * q.i + k * q.r);
		}

		double magSqrd() const {
			return r * r + i * i + j * j + k * k;
		}

		double mag() const {
			return sqrt(r * r + i * i + j * j + k * k);
		}

		Quaternion normalize() const {
			double magnitude = mag();
			return Quaternion(r / magnitude, i / magnitude, j / magnitude, k / magnitude);
		}

		void operator*=(const Quaternion q) {
			double new_r = r * q.r - i * q.i - j * q.j - k * q.k;
			double new_i = r * q.i + i * q.r + j * q.k - k * q.j;
			double new_j = r * q.j - i * q.k + j * q.r + k * q.i;
			double new_k = r * q.k + i * q.j - j * q.i + k * q.r;
			r = new_r;
			i = new_i;
			j = new_j;
			k = new_k;
		}
	};

}