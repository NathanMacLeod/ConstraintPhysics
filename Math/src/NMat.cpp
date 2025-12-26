#include "NMat.h"

namespace mthz {

	template<>
	void rowMajorOrderInverse<1>(double* target, double* source) {
		*target = 1.0 / *source;
	}

	template<>
	void rowMajorOrderInverse<2>(double* target, double* source) {
		double a = source[2*0 + 0]; double b = source[2*0 + 1]; 
		double c = source[2*1 + 0]; double d = source[2*1 + 1];
		double determinant = a * d - b * c;
		assert(std::abs(determinant) > 0.000000001);
		double di = 1.0 / determinant;

		target[2*0 + 0] =  d * di; target[2*0 + 1] = -b * di;
		target[2*1 + 0] = -c * di; target[2*1 + 1] =  a * di;
	}

	template<>
	void rowMajorOrderInverse<3>(double* target, double* source) {
		double a = source[3*0 + 0], b = source[3*0 + 1], c = source[3*0 + 2], 
			   d = source[3*1 + 0], e = source[3*1 + 1], f = source[3*1 + 2], 
			   g = source[3*2 + 0], h = source[3*2 + 1], i = source[3*2 + 2];
		double determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
		assert(std::abs(determinant) > 0.000000001);
		double di = 1.0 / determinant;

		target[3*0 + 0] = (e * i - f * h) * di; target[3*0 + 1] = (c * h - b * i) * di; target[3*0 + 2] = (b * f - c * e) * di;
		target[3*1 + 0] = (f * g - d * i) * di; target[3*1 + 1] = (a * i - c * g) * di; target[3*1 + 2] = (c * d - a * f) * di;
		target[3*2 + 0] = (d * h - e * g) * di; target[3*2 + 1] = (b * g - a * h) * di; target[3*2 + 2] = (a * e - b * d) * di;
	}

	void rowMajorOrderInverse(int n, double* target, double* source) {
		std::vector<double> copy(n * n);
		//initialize copy to source, target to identity matrix
		memcpy(copy.data(), source, n * n * sizeof(double));
		memset(target, 0x00, n * n * sizeof(double));
		for (int i = 0; i < n; i++) target[i * (n + 1)] = 1.0;

		for (int piv = 0; piv < n; piv++) {
			for (int target_row = 0; target_row < n; target_row++) {
				if (piv == target_row) continue;

				double ratio = copy[n * target_row + piv] / copy[n * piv + piv];
				for (int target_col = 0; target_col < n; target_col++) {
					copy[n * target_row + target_col] -= ratio * copy[n * piv + target_col];
					target[n * target_row + target_col] -= ratio * target[n * piv + target_col];
				}

			}
		}
		for (int piv = 0; piv < n; piv++) {
			for (int col = 0; col < n; col++) {
				target[n * piv + col] /= copy[n * piv + piv];
			}
		}
	}

	NMat<3, 3> crossMat(mthz::Vec3 v) {
		NMat<3, 3> out;

		                   out.v[0][1] = -v.z; out.v[0][2] = v.y;
		out.v[1][0] = v.z;                     out.v[1][2] = -v.x;
		out.v[2][0] = -v.y; out.v[2][1] = v.x;

		return out;
	}

};