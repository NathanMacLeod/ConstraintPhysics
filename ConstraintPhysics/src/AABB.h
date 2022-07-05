#pragma once
#include "../../Math/src/Vec3.h"

struct AABB {
	mthz::Vec3 min;
	mthz::Vec3 max;

	static bool intersects(const AABB& a, const AABB& b) {
		return !( a.max.x < b.min.x || a.min.x > b.max.x
			   || a.max.y < b.min.y || a.min.y > b.max.y
			   || a.max.z < b.min.z || a.min.z > b.max.z );
	}
};