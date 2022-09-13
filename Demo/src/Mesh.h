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
	std::vector<Mesh> meshes;
	phyz::RigidBody* r;
};


float frand();

Mesh fromPhyzMesh(const phyz::Mesh& m);
Mesh fromGeometry(const phyz::Geometry g);