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

			for (AABB aabb : v) {
				out.min.x = std::min<double>(out.min.x, aabb.min.x);
				out.min.y = std::min<double>(out.min.y, aabb.min.y);
				out.min.z = std::min<double>(out.min.z, aabb.min.z);

				out.max.x = std::max<double>(out.max.x, aabb.max.x);
				out.max.y = std::max<double>(out.max.y, aabb.max.y);
				out.max.z = std::max<double>(out.max.z, aabb.max.z);
			}

			return out;
		}

		static AABB encapsulatePointCloud(const std::vector<mthz::Vec3>& points) {
			double inf = std::numeric_limits<double>::infinity();
			AABB out = {
				mthz::Vec3(inf, inf, inf),
				mthz::Vec3(-inf, -inf, -inf)
			};

			for (mthz::Vec3 p : points) {
				out.min.x = std::min<double>(out.min.x, p.x);
				out.min.y = std::min<double>(out.min.y, p.y);
				out.min.z = std::min<double>(out.min.z, p.z);

				out.max.x = std::max<double>(out.max.x, p.x);
				out.max.y = std::max<double>(out.max.y, p.y);
				out.max.z = std::max<double>(out.max.z, p.z);
			}

			return out;
		}

		static AABB conformNewBasis(AABB a, mthz::Vec3 u, mthz::Vec3 v, mthz::Vec3 w, mthz::Vec3 xyz_origin) {
			std::vector<mthz::Vec3> xyz_relative_vertices(8);

			xyz_relative_vertices[0] = mthz::Vec3(a.min.x, a.min.y, a.min.z) - xyz_origin;
			xyz_relative_vertices[1] = mthz::Vec3(a.min.x, a.min.y, a.max.z) - xyz_origin;
			xyz_relative_vertices[2] = mthz::Vec3(a.min.x, a.max.y, a.min.z) - xyz_origin;
			xyz_relative_vertices[3] = mthz::Vec3(a.min.x, a.max.y, a.max.z) - xyz_origin;
			xyz_relative_vertices[4] = mthz::Vec3(a.max.x, a.min.y, a.min.z) - xyz_origin;
			xyz_relative_vertices[5] = mthz::Vec3(a.max.x, a.min.y, a.max.z) - xyz_origin;
			xyz_relative_vertices[6] = mthz::Vec3(a.max.x, a.max.y, a.min.z) - xyz_origin;
			xyz_relative_vertices[7] = mthz::Vec3(a.max.x, a.max.y, a.max.z) - xyz_origin;

			double inf = std::numeric_limits<double>::infinity();
			AABB out = {
				mthz::Vec3(inf, inf, inf),
				mthz::Vec3(-inf, -inf, -inf)
			};

			for (int i = 0; i < 8; i++) {
				mthz::Vec3 uvw_coord = mthz::Vec3(xyz_relative_vertices[i].dot(u), xyz_relative_vertices[i].dot(v), xyz_relative_vertices[i].dot(w));

				//out's X, Y, Z represent U, V, W, basis respectively.
				out.min.x = std::min<double>(out.min.x, uvw_coord.x);
				out.min.y = std::min<double>(out.min.y, uvw_coord.y);
				out.min.z = std::min<double>(out.min.z, uvw_coord.z);

				out.max.x = std::max<double>(out.max.x, uvw_coord.x);
				out.max.y = std::max<double>(out.max.y, uvw_coord.y);
				out.max.z = std::max<double>(out.max.z, uvw_coord.z);
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

		static double longestDimension(AABB a) {
			return std::max<double>(a.max.x - a.min.x, std::max<double>(a.max.y - a.min.y, a.max.z - a.min.z));
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