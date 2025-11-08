#pragma once
#include "../../Math/src/Vec3.h"
#include "ConvexPrimitive.h"
#include "AABB.h"

#include <vector>
#include <string>

namespace phyz {

	//Cut feature for now but keeping around in case I want to continue it later

	struct Mesh {
		std::vector<mthz::Vec3> vertices;
		std::vector<std::vector<uint32_t>> face_indices;

		AABB getAABB();
	};

	Mesh readOBJ(const std::string& file_path, double scale=1.0);
	//ConvexPoly getConvexHull(const Mesh& m);
}