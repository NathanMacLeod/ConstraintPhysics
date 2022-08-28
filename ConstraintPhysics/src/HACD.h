#pragma once
#include "../../Math/src/Vec3.h"
#include "ConvexPoly.h"
#include "AABB.h"

#include <vector>
#include <string>

namespace phyz {

	struct Mesh {
		std::vector<mthz::Vec3> vertices;
		std::vector<std::vector<unsigned int>> face_indices;

		AABB getAABB();
	};

	Mesh readOBJ(const std::string& file_path, double scale=1.0);
	ConvexPoly getConvexHull(const Mesh& m);
}