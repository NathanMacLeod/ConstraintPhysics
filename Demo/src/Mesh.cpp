#include "Mesh.h"

color auto_generate = { -1.0f, -1.0f, -1.0f };

float frand() {
	return (float)std::rand() / RAND_MAX;
}

Mesh fromPhyzMesh(const phyz::Mesh& m) {
	std::vector<Vertex> vertices;
	vertices.reserve(m.vertices.size());
	for (mthz::Vec3 v : m.vertices) {
		vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, frand(), frand(), frand(), 0.5f, 0.5f, 0.5f, 5.0f });
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
	layout.Push<float>(1);//ambient_constant
	layout.Push<float>(1);//diffuse_constant
	layout.Push<float>(1);//specular_constant
	layout.Push<float>(1);//specular_pow
	return { new rndr::VertexArray(vertices.data(), vertices.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices.data(), indices.size()) };
}

Mesh fromGeometry(const phyz::Geometry g, color model_color) {
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	int vertex_offset = 0;

	for (const phyz::ConvexPoly& c : g.getPolyhedra()) {
		for (mthz::Vec3 v : c.getPoints()) {
			color col = (model_color == auto_generate) ? color{ 0, 0, 1 } : model_color;
			vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, col.r, col.g, col.b, 0.2f, 0.7f, 0.7f, 5.0f });
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
	layout.Push<float>(1);//ambient_constant
	layout.Push<float>(1);//diffuse_constant
	layout.Push<float>(1);//specular_constant
	layout.Push<float>(1);//specular_pow
	return { new rndr::VertexArray(vertices.data(), vertices.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices.data(), indices.size()) };
}
