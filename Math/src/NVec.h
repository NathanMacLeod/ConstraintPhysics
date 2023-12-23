#pragma once
#include <cmath>

#define CUTOFF 0.00000001

namespace mthz {

	template <int n>
	struct NVec {
		double v[n] = { 0 };

		bool isZero() const {
			for (int i = 0; i < n; i++) {
				if (abs(v[i]) > CUTOFF) {
					return false;
				}
			}
			return true;
		}

		NVec<n> operator+(const NVec<n>& r) const {
			NVec<n> out;
			for (int i = 0; i < n; i++) {
				out.v[i] = this->v[i] + r.v[i];
			}
			return out;
		}

		NVec<n> operator-(const NVec<n>& r) const {
			NVec<n> out;
			for (int i = 0; i < n; i++) {
				out.v[i] = this->v[i] - r.v[i];
			}
			return out;
		}

		NVec<n> operator-() const {
			NVec<n> out;
			for (int i = 0; i < n; i++) {
				out.v[i] = -this->v[i];
			}
			return out;
		};

		double mag() const {
			double sum = 0;
			for (int i = 0; i < n; i++) {
				sum += v[i] * v[i];
			}
			return sqrt(sum);
		}

		void operator+=(const NVec<n>& r) {
			for (int i = 0; i < n; i++) {
				v[i] += r.v[i];
			}
		}
	};

	template<int n>
	NVec<n> operator*(NVec<n> v, double d) {
		NVec<n> out;
		for (int i = 0; i < n; i++) out.v[i] = v.v[i] * d;
		return out;
	}

	template<int n>
	NVec<n> operator*(double d, NVec<n> v) {
		NVec<n> out;
		for (int i = 0; i < n; i++) out.v[i] = v.v[i] * d;
		return out;
	}

}