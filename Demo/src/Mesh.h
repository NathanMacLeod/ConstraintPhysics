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
};

static struct Mesh {
	rndr::VertexArray* va;
	rndr::IndexBuffer* ib;
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