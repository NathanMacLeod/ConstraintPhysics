#pragma once
#include "NVec.h"
#include "Vec3.h"
#include <cstring>
#include <vector>

namespace mthz {

	template<int n>
	void rowMajorOrderInverse(double* target, double* source);

	template <int n_row, int n_col>
	struct NMat {
		double v[n_row][n_col] = { 0 };

		NMat<n_row, n_col> inverse() const {
			assert(n_row == n_col);
			NMat<n_row, n_col> out;
			rowMajorOrderInverse<n_row>((double*)out.v, (double*)v);
			return out;
		}

		NMat<n_col, n_row> transpose() const {
			NMat<n_col, n_row> out;

			for (int i = 0; i < n_row; i++) {
				for (int j = 0; j < n_col; j++) {
					out.v[j][i] = v[i][j];
				}
			}

			return out;
		}

		template<int r, int c>
		void copyInto(const NMat<r, c>& m, int row, int col) {
			for (int i = 0; i < r; i++) {
				for (int j = 0; j < c; j++) {
					v[i + row][j + col] = m.v[i][j];
				}
			}
		}

		NMat<n_row, n_col> operator+(const NMat<n_row, n_col>& r) const {
			NMat<n_row, n_col> out;
			for (int i = 0; i < n_row; i++) {
				for (int j = 0; j < n_col; j++) {
					out.v[i][j] = this->v[i][j] + r.v[i][j];
				}
			}
			return out;
		}
		NVec<n_row> operator*(const NVec<n_col>& n_vec) const {
			NVec<n_row> out;
			for (int i = 0; i < n_row; i++) {
				out.v[i] = 0;
				for (int j = 0; j < n_col; j++) {
					out.v[i] += n_vec.v[j] * this->v[i][j];
				}
			}
			return out;
		}

		NMat<n_row, n_col> operator-(const NMat<n_row, n_col>& r) const {
			NMat<n_row, n_col> out;
			for (int i = 0; i < n_row; i++) {
				for (int j = 0; j < n_col; j++) {
					out.v[i][j] = this->v[i][j] - r.v[i][j];
				}
			}
			return out;
		}

		NMat<n_row, n_col> operator-() const {
			NMat<n_row, n_col> out;
			for (int i = 0; i < n_row; i++) {
				for (int j = 0; j < n_col; j++) {
					out.v[i][j] = -this->v[i][j];
				}
			}
			return out;
		}

		NMat<n_row, n_col> operator*(const double d) const{
		NMat<n_row, n_col> out;
		for (int i = 0; i < n_row; i++) {
			for (int j = 0; j < n_col; j++) {
				out.v[i][j] = v[i][j] * d;
			}
		}
		return out;
	}

		template<int x>
		NMat<n_row, x> operator*(const NMat<n_col, x>& r) const {
			NMat<n_row, x> out;
			for (int i = 0; i < n_row; i++) {
				for (int j = 0; j < x; j++) {
					out.v[i][j] = 0;
					for (int k = 0; k < n_col; k++) {
						out.v[i][j] += v[i][k] * r.v[k][j];
					}
				}
			}
			return out;
		}
	};

	template<int n>
	NMat<n, n> idenMat() {
		NMat<n, n> out;
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				out.v[i][j] = 0;
			}
			out.v[i][i] = 1;
		}
		return out;
	}

	void rowMajorOrderInverse(int n, double* target, double* source);

	template<int n>
	void rowMajorOrderInverse(double* target, double* source) {
		double copy[n*n];
		//initialize copy to source, target to identity matrix
		memcpy(copy, source, n * n * sizeof(double));
		memset(target, 0x00, n * n * sizeof(double));
		for (int i = 0; i < n; i++) target[i * (n + 1)] = 1.0;

		for (int piv = 0; piv < n; piv++) {
			for (int target_row = 0; target_row < n; target_row++) {
				if (piv == target_row) continue;

				double ratio = copy[n*target_row + piv] / copy[n*piv + piv];
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

	template<int n_row, int n_col>
	NMat<n_row, n_col> operator*(double d, const NMat<n_row, n_col> mat) {
		return mat * d;
	}

	template<>
	void rowMajorOrderInverse<1>(double* target, double* source);

	template<>
	void rowMajorOrderInverse<2>(double* target, double* source);

	template<>
	void rowMajorOrderInverse<3>(double* target, double* source);

	NMat<3, 3> crossMat(mthz::Vec3 v);

};