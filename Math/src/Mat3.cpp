#include "Mat3.h";

namespace mthz {

	Mat3 operator*(const double d, const Mat3 m) {
		Mat3 out;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				out.v[i][j] = d * m.v[i][j];
			}
		}
		return out;
	}

	Mat3 operator*(const Mat3 m, const double d) {
		Mat3 out;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				out.v[i][j] = d * m.v[i][j];
			}
		}
		return out;
	}

	Mat3 operator/(const Mat3 m, const double d) {
		Mat3 out;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				out.v[i][j] = m.v[i][j] / d;
			}
		}
		return out;
	}

}