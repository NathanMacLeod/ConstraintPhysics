#include "Mesh.h"

float frand() {
	return (float)std::rand() / RAND_MAX;
}

Mesh fromPhyzMesh(const phyz::Mesh& m) {
	std::vector<Vertex> vertices;
	vertices.reserve(m.vertices.size());
	for (mthz::Vec3 v : m.vertices) {
		vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, frand(), frand(), frand() });
	}

	std::vector<unsigned int> indices;
	for (const std::vector<unsigned int>& face_indices : m.face_indices) {
		//naive- only works if all faces are convex
		for (int i = 2; i < face_indices.size(); i++) {
			indices.push_back(face_indices[0]);
			indices.push_back(face_indices[i - 1]);
			indices.push_back(face_indices[i]);
		}
	}

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(vertices.data(), vertices.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices.data(), indices.size()) };
}

Mesh fromGeometry(const phyz::Geometry g) {
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	int vertex_offset = 0;

	for (const phyz::ConvexPoly& c : g.getPolyhedra()) {
		for (mthz::Vec3 v : c.getPoints()) {
			vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, frand(), frand(), frand() });
		}

		for (const phyz::Surface& s : c.getSurfaces()) {
			//naive- only works if all faces are convex
			for (int i = 2; i < s.n_points(); i++) {
				indices.push_back(s.point_indexes[0] + vertex_offset);
				indices.push_back(s.point_indexes[i - 1] + vertex_offset);
				indices.push_back(s.point_indexes[i] + vertex_offset);
			}
		}

		vertex_offset += c.getPoints().size();
	}

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(vertices.data(), vertices.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices.data(), indices.size()) };
}
