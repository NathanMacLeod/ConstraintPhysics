#pragma once
#include "../../Math/src/Vec3.h"
#include <vector>

namespace phyz{
	struct AABB {
		mthz::Vec3 min;
		mthz::Vec3 max;

		static bool intersects(AABB a, AABB b) {
			return ! (a.max.x < b.min.x || a.min.x > b.max.x
				   || a.max.y < b.min.y || a.min.y > b.max.y
				   || a.max.z < b.min.z || a.min.z > b.max.z);
		}

		static AABB combine(AABB a, AABB b) {
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

		static double volume(AABB a) {
			return	  (a.max.x - a.min.x)
					* (a.max.y - a.min.y)
					* (a.max.z - a.min.z);
		}

		static double surfaceArea(AABB a) {
			double dx = a.max.x - a.min.x;
			double dy = a.max.y - a.min.y;
			double dz = a.max.z - a.min.z;

			return 2 * (dx * dy) + (dy * dz) + (dz * dx);
		}

		static AABB getScaled(AABB a, double scale_factor) {
			mthz::Vec3 center = (a.max + a.min) / 2.0;

			return AABB{
				scale_factor * a.min - (scale_factor - 1) * center,
				scale_factor * a.max - (scale_factor - 1) * center
			};
		}

		static bool isAABBContained(AABB contained_aabb, AABB containing_aabb) {
			return	   contained_aabb.min.x >= containing_aabb.min.x
					&& contained_aabb.min.y >= containing_aabb.min.y
					&& contained_aabb.min.z >= containing_aabb.min.z
					&& contained_aabb.max.x <= containing_aabb.max.x
					&& contained_aabb.max.y <= containing_aabb.max.y
					&& contained_aabb.max.z <= containing_aabb.max.z;
		}

		//slab method
		static bool rayIntersectsAABB(AABB aabb, mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
			double t_min = -std::numeric_limits<double>::infinity();
			double t_max = std::numeric_limits<double>::infinity();

			double eps = 0.00000000001;
			for (int i = 0; i < 3; i++) {
				if (abs(ray_dir[i]) < eps) {
					//case where ray_dir is parralel to the dimension being clipped
					if (ray_origin[i] < aabb.min[i] || ray_origin[i] > aabb.max[i]) return false;
					//otherwise no limitations on bounds so dont need to restrict
				}
				else {
					double t0 = (aabb.min[i] - ray_origin[i]) / ray_dir[i];
					double t1 = (aabb.max[i] - ray_origin[i]) / ray_dir[i];
					if (t0 > t1) {
						double tmp = t0;
						t0 = t1;
						t1 = tmp;
					}

					t_min = std::max<double>(t0, t_min);
					t_max = std::min<double>(t1, t_max);

					//t_max < t_min implies no portion of the ray lies within the AABB
					//t_max < 0 implies instersection occurs behind the origin
					if (t_max < 0 || t_max < t_min) return false;
				}
			}

			return true;
		}
	};
}