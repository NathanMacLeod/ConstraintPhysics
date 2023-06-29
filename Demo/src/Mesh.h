#pragma once
#include "renderer/Renderer.h"
#include "../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../ConstraintPhysics/src/HACD.h"

struct Vertex {
	float x;
	float y;
	float z;
	float r;
	float g;
	float b;
	float ambient_k;
	float diffuse_k;
	float specular_k;
	float specular_p;

	static rndr::VertexArrayLayout generateLayout();
};

struct Mesh {
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
};

struct PhysBod {
	Mesh mesh;
	phyz::RigidBody* r;
};


static struct color {
	float r;
	float g;
	float b;

	bool operator==(const color& c) {
		return r == c.r && g == c.g && b == c.b;
 	}
};
extern color auto_generate;

float frand();

Mesh fromPhyzMesh(const phyz::Mesh& m);
Mesh fromGeometry(const phyz::Geometry g, color c=auto_generate);
Mesh getTransformed(const Mesh& m, mthz::Vec3 model_position, mthz::Quaternion model_orientation, mthz::Vec3 camera_position, mthz::Quaternion camera_orientation, bool recolor = false, color new_color=auto_generate);
void writeTransformedTo(const Mesh& m, Mesh* out, mthz::Vec3 model_position, mthz::Quaternion model_orientation, mthz::Vec3 camera_position, mthz::Quaternion camera_orientation, bool recolor = false, color new_color = auto_generate);