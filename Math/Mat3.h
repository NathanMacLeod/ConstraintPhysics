#pragma once

#include "Vec3.h"

class Mat3 {
public:
	double v[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };

	Vec3 operator*(Vec3 w) {
		return Vec3(v[0][0] * w.x + v[0][1] * w.y + v[0][2] * w.z,
					v[1][0] * w.x + v[1][1] * w.y + v[1][2] * w.z,
					v[2][0] * w.x + v[2][1] * w.y + v[2][2] * w.z);
	}

	Mat3 operator*(const Mat3 m) {
		Mat3 out;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					out.v[i][j] += v[i][k] * m.v[k][j];
				}
			}
		}
		return out;
	}

	void operator*=(const Mat3 m) {
		double out[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					out[i][j] += v[i][k] * m.v[k][j];
				}
			}
		}

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				v[i][j] = out[i][j];
			}
		}
	}

	void operator*=(const double d) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				v[i][j] *= d;
			}
		}
	}
};

Mat3 operator*(const double d, const Mat3 m);
Mat3 operator*(const Mat3 m, const double d);
Mat3 operator/(const Mat3 m, const double d);