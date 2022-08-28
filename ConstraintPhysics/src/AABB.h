#pragma once
#include "../../Math/src/Vec3.h"
#include <vector>

namespace phyz{
	struct AABB {
		mthz::Vec3 min;
		mthz::Vec3 max;

		static bool intersects(const AABB& a, const AABB& b) {
			return !(a.max.x < b.min.x || a.min.x > b.max.x
				   || a.max.y < b.min.y || a.min.y > b.max.y
				   || a.max.z < b.min.z || a.min.z > b.max.z);
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
	};
}