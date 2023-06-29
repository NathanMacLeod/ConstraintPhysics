#include "Mesh.h"

color auto_generate = { -1.0f, -1.0f, -1.0f };

float frand() {
	return (float)std::rand() / RAND_MAX;
}

rndr::VertexArrayLayout Vertex::generateLayout() {
	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	layout.Push<float>(1);//ambient_constant
	layout.Push<float>(1);//diffuse_constant
	layout.Push<float>(1);//specular_constant
	layout.Push<float>(1);//specular_pow
	return layout;
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

	return Mesh{ vertices, indices };
}

Mesh fromGeometry(const phyz::Geometry g, color model_color) {
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	int vertex_offset = 0;

	for (const phyz::ConvexPrimitive& prim : g.getPolyhedra()) {
		switch (prim.getType()) {
		case phyz::POLYHEDRON:
		{
			const phyz::Polyhedron& c = (const phyz::Polyhedron&)*prim.getGeometry();

			for (mthz::Vec3 v : c.getPoints()) {
				color col = (model_color == auto_generate) ? color{ frand(), frand(), frand() } : model_color;
				vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, col.r, col.g, col.b, 1.0f, 1.0f, 0.7f, 5.0f });
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
			break;
		}

		case phyz::SPHERE:
		{
			const phyz::Sphere& s = (const phyz::Sphere&)*prim.getGeometry();
			int n_rows = 15;
			int n_cols = 20;

			mthz::Vec3 bottom_pole = s.getCenter() - mthz::Vec3(0, s.getRadius(), 0);
			mthz::Vec3 top_pole = s.getCenter() + mthz::Vec3(0, s.getRadius(), 0);
			color bot_col = (model_color == auto_generate) ? color{ frand(), frand(), frand() } : model_color;
			color top_col = (model_color == auto_generate) ? color{ frand(), frand(), frand() } : model_color;
			vertices.push_back(Vertex{ (float)bottom_pole.x, (float)bottom_pole.y, (float)bottom_pole.z, bot_col.r, bot_col.g, bot_col.b, 1.0f, 1.0f, 0.7f, 5.0f });
			vertices.push_back(Vertex{ (float)top_pole.x, (float)top_pole.y, (float)top_pole.z, top_col.r, top_col.g, top_col.b, 1.0f, 1.0f, 0.7f, 5.0f });

			//create other vertices
			for (int row = 1; row < n_rows; row++) {
				for (int col = 0; col < n_cols; col++) {
					//polar coordinates
					double theta = -2 * PI * col / n_cols;
					double phi = PI - PI * row / n_rows;

					mthz::Vec3 v = s.getCenter() + s.getRadius() * mthz::Vec3(cos(theta) * sin(phi), cos(phi), sin(theta) * sin(phi));
					color v_col = (model_color == auto_generate) ? color{ frand(), frand(), frand() } : model_color;
					vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, v_col.r, v_col.g, v_col.b, 1.0f, 1.0f, 0.7f, 5.0f });
				}
			}

			int bottom_pole_index = 0;
			int top_pole_index = 1;
			int nonpole_offset = 2;
			//create triangles
			for (int col = 0; col < n_cols; col++) {
				int i1 = col;
				int i2 = (col + 1) % n_cols;
				indices.push_back(bottom_pole_index + vertex_offset);
				indices.push_back(i2 + nonpole_offset + vertex_offset);
				indices.push_back(i1 + nonpole_offset + vertex_offset);
			}
			for (int row = 1; row < n_rows - 1; row++) {
				for (int col = 0; col < n_cols; col++) {
					int i1 = col;
					int i2 = (col + 1) % n_cols;
					int row_offset = (row - 1) * n_cols;

					indices.push_back(i1 + row_offset + nonpole_offset + vertex_offset);
					indices.push_back(i2 + row_offset + nonpole_offset + vertex_offset);
					indices.push_back(i2 + n_cols + row_offset + nonpole_offset + vertex_offset);

					indices.push_back(i2 + n_cols + row_offset + nonpole_offset + vertex_offset);
					indices.push_back(i1 + n_cols + row_offset + nonpole_offset + vertex_offset);
					indices.push_back(i1 + row_offset + nonpole_offset + vertex_offset);
				}
			}
			for (int col = 0; col < n_cols; col++) {
				int i1 = col;
				int i2 = (col + 1) % n_cols;
				int row_offset = (n_rows - 2) * n_cols;
				indices.push_back(i1 + row_offset + nonpole_offset + vertex_offset);
				indices.push_back(i2 + row_offset + nonpole_offset + vertex_offset);
				indices.push_back(top_pole_index + vertex_offset);
			}

			vertex_offset += 2 + (n_rows - 1) * n_cols;
			break;
		}
		}
	}

	return Mesh{ vertices, indices };
}

Mesh getTransformed(const Mesh& m, mthz::Vec3 model_position, mthz::Quaternion model_orientation, mthz::Vec3 camera_position, mthz::Quaternion camera_orientation, bool recolor, color new_color) {
	Mesh out = m;
	writeTransformedTo(m, &out, model_position, model_orientation, camera_position, camera_orientation, recolor, new_color);
	return out;
}

void writeTransformedTo(const Mesh& m, Mesh* out, mthz::Vec3 model_position, mthz::Quaternion model_orientation, mthz::Vec3 camera_position, mthz::Quaternion camera_orientation, bool recolor, color new_color) {
	assert(m.vertices.size() == out->vertices.size());

	mthz::Mat3 camera_rot = camera_orientation.conjugate().getRotMatrix();
	mthz::Mat3 model_rot = model_orientation.getRotMatrix();

	for (int i = 0; i < m.vertices.size(); i++) {
		mthz::Vec3 pos = mthz::Vec3(m.vertices[i].x, m.vertices[i].y, m.vertices[i].z);
		mthz::Vec3 transformed_pos = camera_rot * (model_rot * pos + model_position - camera_position);
		out->vertices[i].x = transformed_pos.x; out->vertices[i].y = transformed_pos.y; out->vertices[i].z = transformed_pos.z;

		if (recolor) {
			out->vertices[i].r = new_color.r; out->vertices[i].g = new_color.g; out->vertices[i].b = new_color.b;
		}
	}
}
