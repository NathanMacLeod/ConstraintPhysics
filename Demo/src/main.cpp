
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

static Mesh tetra(float r) {
	float rt3 = sqrt(3.0);

	std::vector<Vertex> verticies = {
		{  3*r/4, -r/3, -rt3*r/4, 1.0f, 0.0f, 0.0f}, //0
		{  0,     -r/3,  rt3*r/2, 1.0f, 1.0f, 0.0f}, //1
		{ -3*r/4, -r/3, -rt3*r/4, 1.0f, 0.0f, 1.0f}, //2
		{  0,      r,    0,       1.0f, 1.0f, 1.0f}, //3
	};

	unsigned int indices[] = {
		0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		2, 1, 3,
	};

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(verticies.data(), verticies.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices, 12) };
}

static Mesh ramp(float l, float h, float w) {

	std::vector<Vertex> verticies = {
		{ 0, 0, 0, 1.0f, 0.0f, 0.0f },
		{ l, 0, 0, 1.0f, 1.0f, 0.0f },
		{ 0, h, 0, 1.0f, 0.0f, 1.0f },
		{ 0, 0, w, 1.0f, 1.0f, 0.0f },
		{ l, 0, w, 1.0f, 0.0f, 1.0f },
		{ 0, h, w, 1.0f, 0.0f, 0.0f },
	};

	unsigned int indices[] = {
		0, 2, 1,
		3, 4, 5,
		0, 1, 4,
		4, 3, 0,
		1, 2, 5,
		5, 4, 1,
		2, 0, 3,
		3, 5, 2,
	};

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(verticies.data(), verticies.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices, 24) };
}

struct PhysBod {
	std::vector<Mesh> meshes;
	phyz::RigidBody* r;
	phyz::RigidBody::PKey draw_p;
};

int main() {

	phyz::PhysicsEngine p;

	rndr::init(960, 960, "Hello World");

	std::vector<PhysBod> bodies;

	{
		double s = 400;
		std::vector<Mesh> m2 = { rect(0, 0, 0, s, 2, s) };
		std::vector<phyz::ConvexPoly> geom2;
		geom2.push_back(phyz::ConvexPoly::genRect(-s / 2, -3, -s / 2, s, 2, s));
		phyz::RigidBody* r2 = p.createRigidBody(geom2, true);
		phyz::RigidBody::PKey draw_p = r2->track_point(mthz::Vec3(0, -2, 0));
		r2->com.y = -5;
		r2->updateGeometry();
		bodies.push_back({ m2, r2, draw_p });
	}

	//{
	//	double s = 10;
	//	mthz::Vec3 pos(-1, -4, -5);
	//	mthz::Vec3 dim(14, 3, 10);
	//	std::vector<Mesh> m2 = { ramp(dim.x, dim.y, dim.z) };
	//	std::vector<phyz::ConvexPoly> geom2;
	//	geom2.push_back(phyz::ConvexPoly::genRamp(pos.x, pos.y, pos.z, dim.x, dim.y, dim.z));
	//	phyz::RigidBody* r2 = p.createRigidBody(geom2, true);
	//	phyz::RigidBody::PKey draw_p = r2->track_point(pos);
	//	r2->updateGeometry();
	//	bodies.push_back({ m2, r2, draw_p });
	//}

	/*Mesh m1 = rect(0, 0, 0, 1, 1, 1);
	std::vector<phyz::ConvexPoly> geom1;
	geom1.push_back(phyz::ConvexPoly::genRect(0, 2, 0, 1, 1, 1));
	phyz::RigidBody* r1 = p.createRigidBody(geom1);
	r1->ang_vel = mthz::Vec3(0, 0, 0);*/
	//r1->orientation = mthz::Quaternion(3.1415926563 / 4, mthz::Vec3(0, 0, 1)) * mthz::Quaternion(3.1415926563 / 4, mthz::Vec3(0, 1, 0));

	rndr::Shader shader("resources/shaders/Basic.shader");

	shader.bind();
	
	float t = 0;
	float fElapsedTime;

	mthz::Vec3 pos(0, 0, 10);
	mthz::Quaternion orient;
	double mv_speed = 2;
	double rot_speed = 1;

	double phyz_time = 0;
	double timestep = 1 / 90.0;
	p.setStep_time(timestep);
	p.setGravity(mthz::Vec3(0, -9.8, 0));

	while (rndr::render_loop(&fElapsedTime)) {

		if (rndr::getKeyDown(GLFW_KEY_W)) {
			pos += orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
		}
		else if (rndr::getKeyDown(GLFW_KEY_S)) {
			pos += orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
		}
		if (rndr::getKeyDown(GLFW_KEY_A)) {
			pos += orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
		}
		else if (rndr::getKeyDown(GLFW_KEY_D)) {
			pos += orient.applyRotation(mthz::Vec3(1, 0, 0) * fElapsedTime * mv_speed);
		}

		if (rndr::getKeyDown(GLFW_KEY_UP)) {
			orient = orient * mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
		}
		else if (rndr::getKeyDown(GLFW_KEY_DOWN)) {
			orient = orient * mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
		}
		if (rndr::getKeyDown(GLFW_KEY_LEFT)) {
			orient = mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
		}
		else if (rndr::getKeyDown(GLFW_KEY_RIGHT)) {
			orient = mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
		}

		t += fElapsedTime;

		if (rndr::getKeyPressed(GLFW_KEY_B)) {
			int a = 1 + 2;
		}

		//square
		if (rndr::getKeyPressed(GLFW_KEY_K)) {
			std::vector<Mesh> m1 = { rect(0, 0, 0, 1, 1, 1) };
			std::vector<phyz::ConvexPoly> geom1;
			double h = 10;
			geom1.push_back(phyz::ConvexPoly::genRect(-0.5, h - 0.5, -0.5, 1, 1, 1));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->track_point(mthz::Vec3(0, h, 0));
			//r1->ang_vel = mthz::Vec3(0, 100, 1);
			//r1->vel = mthz::Vec3(1, 0, 0);
			//r1->com.x = 2.3;
			//r1->orientation = mthz::Quaternion(3.1415926535 / 4, mthz::Vec3(0, 0, 1)) * mthz::Quaternion(3.1415926535 / 4, mthz::Vec3(0, 1, 0));
			bodies.push_back({ m1, r1, draw_p });
		}
		//tetrahedron
		if (rndr::getKeyPressed(GLFW_KEY_I)) {
			std::vector<Mesh> m1 = { tetra(1) };
			std::vector<phyz::ConvexPoly> geom1;
			geom1.push_back(phyz::ConvexPoly::genTetra(0, 10, 0, 1));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->track_point(mthz::Vec3(0, 10, 0));
			r1->ang_vel = mthz::Vec3(0, 0, 0);
			r1->vel = mthz::Vec3(0, 0, 0);
			//r1->com.x = 2.3;
			r1->orientation = mthz::Quaternion(3.1415926535 / 2, mthz::Vec3(0, 0, 1));
			bodies.push_back({ m1, r1, draw_p });
		}
		//top
		if (rndr::getKeyPressed(GLFW_KEY_J)) {
			double h1 = 1.8, w1 = 0.05, h2 = 0.2, w2 = 2, s = 0.13;
			std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2, w2)};
			std::vector<phyz::ConvexPoly> geom1;
			geom1.push_back(phyz::ConvexPoly::genRect(-w1/2, -h1/2, -w1/2, w1, h1, w1));
			geom1.push_back(phyz::ConvexPoly::genRect(-w2/2, -h2/2 -h1*s/2, -w2/2, w2, h2, w2));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->track_point(mthz::Vec3(0, 0, 0));
			r1->com = mthz::Vec3(0, 10, 0);
			r1->ang_vel = mthz::Vec3(0, 40, 2.5);
			//r1->vel = mthz::Vec3(1, 0, 0);
			bodies.push_back({ m1, r1, draw_p });
		}

		//barbell
		if (rndr::getKeyPressed(GLFW_KEY_L)) {
			double h1 = 1.8, w1 = 0.2, h2 = 0.5, w2 = 1, s = 0.5;
			std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2, w2), rect(0, h1 * s / 2, 0, w2, h2, w2) };
			std::vector<phyz::ConvexPoly> geom1;
			geom1.push_back(phyz::ConvexPoly::genRect(-w1 / 2, -h1 / 2, -w1 / 2, w1, h1, w1));
			geom1.push_back(phyz::ConvexPoly::genRect(-w2 / 2, -h2 / 2 - h1 * s / 2, -w2 / 2, w2, h2, w2));
			geom1.push_back(phyz::ConvexPoly::genRect(-w2/2, -h2 / 2 + h1 * s / 2, -w2/2, w2, h2, w2));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->track_point(mthz::Vec3(0, 0, 0));
			r1->com = mthz::Vec3(0, 10, 0);
			//r1->orientation = mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(0, 0, 1));
			r1->ang_vel = mthz::Vec3(0, 6, 1.4);
			//r1->vel = mthz::Vec3(10, 0, 0);
			bodies.push_back({ m1, r1, draw_p });
		}

		//unbalanced_barbell
		if (rndr::getKeyPressed(GLFW_KEY_U)) {
			double h1 = 2.8, w1 = 0.2, h2 = 0.5, w2 = 1, s = 0.5;
			std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2/10, w2), rect(0, h1 * s / 2, 0, w2 * 3/4.0, h2, w2 * 3/4.0) };
			std::vector<phyz::ConvexPoly> geom1;
			geom1.push_back(phyz::ConvexPoly::genRect(-w1 / 2, -h1 / 2, -w1 / 2, w1, h1, w1));
			geom1.push_back(phyz::ConvexPoly::genRect(-w2 / 2, -h2 / 20 - h1 * s / 2, -w2 / 2, w2, h2/10, w2));
			geom1.push_back(phyz::ConvexPoly::genRect(-w2 * 3 / 8.0, -h2 / 2 + h1 * s / 2, -w2  * 3 / 8.0, w2 * 3/4.0, h2, w2 * 3/4.0));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->track_point(mthz::Vec3(0, 0, 0));
			r1->com = mthz::Vec3(0, 10, 0);
			r1->orientation = mthz::Quaternion(0.67895851999693979, 0.0070540392161153693, 0.022756044727275559, 0.73378997750219643);
			r1->ang_vel = mthz::Vec3(0, 0.5, 10);
			//r1->vel = mthz::Vec3(10, 0, 0);
			bodies.push_back({ m1, r1, draw_p });
		}

		phyz_time += fElapsedTime;
		//phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
		if (phyz_time > 1 / 30.0) {
			printf("n objects: %d\n", p.getNumBodies());
		}
		while (phyz_time > timestep) {
			phyz_time -= timestep;
			p.timeStep();
		}


		rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

		for (const PhysBod& b : bodies) {
			for (const Mesh& m : b.meshes) {
				shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0) * rndr::Mat4::cam_view(pos, orient) * rndr::Mat4::model(b.r->getTrackedP(b.draw_p), b.r->orientation));
				shader.setUniform1i("u_Asleep", b.r->getAsleep());
				rndr::draw(*m.va, *m.ib, shader);
			}
		}
	}

	rndr::terminate();
	return 0;
}