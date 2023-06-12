#pragma once
#include "../../Math/src/Vec3.h"
#include <vector>

namespace phyz{
	struct AABB {
		mthz::Vec3 min;
		mthz::Vec3 max;

		static bool intersects(const AABB& a, const AABB& b) {
			return ! (a.max.x < b.min.x || a.min.x > b.max.x
				   || a.max.y < b.min.y || a.min.y > b.max.y
				   || a.max.z < b.min.z || a.min.z > b.max.z);
		}

		static AABB combine(const AABB& a, const AABB& b) {
			AABB out;
			out.min.x = std::min<double>(a.min.x, b.min.x);
			out.min.y = std::min<double>(a.min.y, b.min.y);
			out.min.z = std::min<double>(a.min.z, b.min.z);

			out.max.x = std::max<double>(a.max.x, b.max.x);
			out.max.y = std::max<double>(a.max.y, b.max.y);
			out.max.z = std::max<double>(a.max.z, b.max.z);
			return out;
		}

		static AABB combine(const std::vector<AABB>& v) {
			if (v.size() == 1) {
				return v[0];
			}

			double inf = std::numeric_limits<double>::infinity();
			AABB out = {
				mthz::Vec3(inf, inf, inf),
				mthz::Vec3(-inf, -inf, -inf)
			};

			for (const AABB& aabb : v) {
				out.min.x = std::min<double>(out.min.x, aabb.min.x);
				out.min.y = std::min<double>(out.min.y, aabb.min.y);
				out.min.z = std::min<double>(out.min.z, aabb.min.z);

				out.max.x = std::max<double>(out.max.x, aabb.max.x);
				out.max.y = std::max<double>(out.max.y, aabb.max.y);
				out.max.z = std::max<double>(out.max.z, aabb.max.z);
			}

			return out;
		}

		static double volume(const AABB& a) {
			return	  (a.max.x - a.min.x)
					* (a.max.y - a.min.y)
					* (a.max.z - a.min.z);
		}

		static AABB getScaled(const AABB& a, double scale_factor) {
			mthz::Vec3 center = (a.max + a.min) / 2.0;

			return AABB{
				scale_factor * a.min - (scale_factor - 1) * center,
				scale_factor * a.max - (scale_factor - 1) * center
			};
		}

		static bool isAABBContained(const AABB& contained_aabb, const AABB& containing_aabb) {
			return	   contained_aabb.min.x >= containing_aabb.min.x
					&& contained_aabb.min.y >= containing_aabb.min.y
					&& contained_aabb.min.z >= containing_aabb.min.z
					&& contained_aabb.max.x <= containing_aabb.max.x
					&& contained_aabb.max.y <= containing_aabb.max.y
					&& contained_aabb.max.z <= containing_aabb.max.z;
		}
	};
}