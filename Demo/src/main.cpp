
#define GLEW_STATIC
#include "renderer/Renderer.h"
#include "renderer/Mat4.h"
#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../ConstraintPhysics/src/Rigidbody.h"
#include "../../ConstraintPhysics/src/ConvexPoly.h"
#include <cmath>

struct Vertex {
	float x;
	float y;
	float z;
	float r;
	float g;
	float b;
};

static struct Mesh {
	~Mesh() {
		delete va;
		delete ib;
	}
	rndr::VertexArray* va;
	rndr::IndexBuffer* ib;
};

static Mesh rect(float x, float y, float z, float w, float h, float d) {
	std::vector<Vertex> verticies = {
		{ x - w/2, y - h/2, z + d/2, 1.0f, 0.0f, 0.0f}, //0
		{ x + w/2, y - h/2, z + d/2, 1.0f, 1.0f, 0.0f}, //1
		{ x + w/2, y + h/2, z + d/2, 1.0f, 0.0f, 1.0f}, //2
		{ x - w/2, y + h/2, z + d/2, 1.0f, 1.0f, 1.0f}, //3
		{ x - w/2, y - h/2, z - d/2, 0.0f, 0.0f, 0.0f}, //4
		{ x + w/2, y - h/2, z - d/2, 0.0f, 0.0f, 1.0f}, //5
		{ x + w/2, y + h/2, z - d/2, 1.0f, 0.0f, 1.0f}, //6
		{ x - w/2, y + h/2, z - d/2, 0.0f, 1.0f, 1.0f}, //7
	};

	unsigned int indices[] = {
		0, 1, 2,
		2, 3, 0,

		1, 5, 6,
		6, 2, 1,

		5, 4, 7,
		7, 6, 5,

		4, 0, 3,
		3, 7, 4,

		2, 6, 7,
		7, 3, 2,

		0, 4, 5,
		5, 1, 0,
	};

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(verticies.data(), verticies.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices, 36) };
}

int main() {

	phyz::PhysicsEngine p;

	rndr::init(960, 960, "Hello World");

	Mesh m1 = rect(0, 0, 0, 1, 1, 1);
	std::vector<phyz::ConvexPoly> geom1;
	geom1.push_back(phyz::ConvexPoly::genRect(0, 2, 0, 1, 1, 1));
	phyz::RigidBody* r1 = p.createRigidBody(geom1);
	r1->ang_vel = mthz::Vec3(0, 1, 15);

	Mesh m2 = rect(0, 0, 0, 6, 0.1, 6);
	std::vector<phyz::ConvexPoly> geom2;
	geom2.push_back(phyz::ConvexPoly::genRect(-3, -1, -3, 6, 0.1, 6));
	phyz::RigidBody* r2 = p.createRigidBody(geom2, true);

	rndr::Shader shader("resources/shaders/Basic.shader");

	shader.bind();
	
	float t = 0;
	float fElapsedTime;

	mthz::Vec3 pos(0, 0, 10);
	mthz::Quaternion orient;
	double mv_speed = 2;
	double rot_speed = 1;

	double phyz_time = 0;
	double timestep = 1 / 60.0;
	p.step_time = timestep;
	p.gravity = mthz::Vec3(0, -1.0, 0);

	while (rndr::render_loop(&fElapsedTime)) {

		if (rndr::getKey(GLFW_KEY_W)) {
			pos += orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
		}
		else if (rndr::getKey(GLFW_KEY_S)) {
			pos += orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
		}
		if (rndr::getKey(GLFW_KEY_A)) {
			pos += orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
		}
		else if (rndr::getKey(GLFW_KEY_D)) {
			pos += orient.applyRotation(mthz::Vec3(1, 0, 0) * fElapsedTime * mv_speed);
		}

		if (rndr::getKey(GLFW_KEY_UP)) {
			orient = orient * mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
		}
		else if (rndr::getKey(GLFW_KEY_DOWN)) {
			orient = orient * mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
		}
		if (rndr::getKey(GLFW_KEY_LEFT)) {
			orient = mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
		}
		else if (rndr::getKey(GLFW_KEY_RIGHT)) {
			orient = mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
		}

		t += fElapsedTime;

		phyz_time += fElapsedTime;
		while (phyz_time > timestep) {
			phyz_time -= timestep;
			p.timeStep();
		}


		rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

		shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0)* rndr::Mat4::cam_view(pos, orient) * rndr::Mat4::model(r1->com, r1->orientation));
		rndr::draw(*m1.va, *m1.ib, shader);

		shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0) * rndr::Mat4::cam_view(pos, orient) * rndr::Mat4::model(r2->com, r2->orientation));
		rndr::draw(*m2.va, *m2.ib, shader);

		if (r1->com.y < -3) {
			r1->com = mthz::Vec3(0, 3, 0);
		}
	}

	rndr::terminate();
	return 0;
}