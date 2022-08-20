
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

static Mesh tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4) {

	std::vector<Vertex> verticies = {
		Vertex{ (float)p1.x, (float)p1.y, (float)p1.z, 1.0f, 0.0f, 0.0f}, //0
		Vertex{ (float)p2.x, (float)p2.y, (float)p2.z, 1.0f, 1.0f, 0.0f}, //1
		Vertex{ (float)p3.x, (float)p3.y, (float)p3.z, 1.0f, 0.0f, 1.0f}, //2
		Vertex{ (float)p4.x, (float)p4.y, (float)p4.z, 1.0f, 1.0f, 1.0f}, //3
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
};

phyz::RigidBody* box(std::vector<PhysBod>* bodies, phyz::PhysicsEngine* p, mthz::Vec3 pos, mthz::Vec3 dim) {
	std::vector<Mesh> m1 = { rect(0, 0, 0, dim.x, dim.y, dim.z) };
	phyz::Geometry geom1;
	geom1.pushConvexPoly(phyz::ConvexPoly::box(-dim.x / 2.0, -dim.y / 2.0, -dim.z / 2.0, dim.x, dim.y, dim.z));
	phyz::RigidBody* r1 = p->createRigidBody(geom1);
	phyz::RigidBody::PKey draw_p1 = r1->trackPoint(mthz::Vec3(0, 0, 0));
	bodies->push_back({ m1, r1 });
	r1->setCOMtoPosition(pos);
	return r1;
}

int main() {

	phyz::PhysicsEngine p;

	rndr::init(960, 960, "Hello World");

	std::vector<PhysBod> bodies;

	{
		double s = 400;
		std::vector<Mesh> m2 = { rect(0, 0, 0, s, 2, s) };
		phyz::Geometry geom2;
		geom2.pushConvexPoly(phyz::ConvexPoly::box(-s / 2, -1, -s / 2, s, 2, s));
		phyz::RigidBody* r2 = p.createRigidBody(geom2, true);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		r2->setCOMtoPosition(mthz::Vec3(0, -5, 0));
		bodies.push_back({ m2, r2 });
	}

	//{
	//	double s = 10;
	//	mthz::Vec3 pos(-1, -4, -5);
	//	mthz::Vec3 dim(14, 3, 10);
	//	std::vector<Mesh> m2 = { ramp(dim.x, dim.y, dim.z) };
	//	phyz::Geometry geom2;
	//	geom2.pushConvexPoly(phyz::ConvexPoly::genRamp(pos.x, pos.y, pos.z, dim.x, dim.y, dim.z));
	//	phyz::RigidBody* r2 = p.createRigidBody(geom2, true);
	//	phyz::RigidBody::PKey draw_p = r2->track_point(pos);
	//	r2->updateGeometry();
	//	bodies.push_back({ m2, r2, draw_p });
	//}

	/*Mesh m1 = rect(0, 0, 0, 1, 1, 1);
	phyz::Geometry geom1;
	geom1.pushConvexPoly(phyz::ConvexPoly::box(0, 2, 0, 1, 1, 1));
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
			p.setGravity(p.getGravity() + mthz::Vec3(3, 0, 0));
		}
		if (rndr::getKeyPressed(GLFW_KEY_V)) {
			p.setGravity(p.getGravity() + mthz::Vec3(-3, 0, 0));
		}

		//square
		if (rndr::getKeyPressed(GLFW_KEY_K)) {
			std::vector<Mesh> m1 = { rect(0, 0, 0, 1, 1, 1) };
			phyz::Geometry geom1;
			double h = 10;
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-0.5, -0.5, -0.5, 1, 1, 1));
			phyz::RigidBody* r1 = p.createRigidBody(geom1, false, mthz::Vec3(0, h, 0));
			phyz::RigidBody::PKey draw_p = r1->trackPoint(mthz::Vec3(0, h, 0));
			//r1->ang_vel = mthz::Vec3(0, 100, 1);
			//r1->vel = mthz::Vec3(1, 0, 0);
			//r1->com.x = 2.3;
			//r1->orientation = mthz::Quaternion(3.1415926535 / 4, mthz::Vec3(0, 0, 1)) * mthz::Quaternion(3.1415926535 / 4, mthz::Vec3(0, 1, 0));
			bodies.push_back({ m1, r1 });
		}
		//tetrahedron
		if (rndr::getKeyPressed(GLFW_KEY_I)) {
			mthz::Vec3 p1(0, 0, 0), p2(1, 0, 0), p3(0, 1, 0), p4(0, 0, 1);
			std::vector<Mesh> m1 = { tetra(p1, p2, p3, p4) };
			phyz::Geometry geom1;
			geom1.pushConvexPoly(phyz::ConvexPoly::tetra(p1, p2, p3, p4));
			phyz::RigidBody* r1 = p.createRigidBody(geom1, false, mthz::Vec3(0, 10, 0));
			//r1->setAngVel(mthz::Vec3(0, 0, 0));
			//r1->setVel(mthz::Vec3(0, 0, 0));
			//r1->setOrientation(mthz::Quaternion(3.1415926535 / 2, mthz::Vec3(0, 0, 1)));
			bodies.push_back({ m1, r1 });
		}
		//top
		if (rndr::getKeyPressed(GLFW_KEY_J)) {
			double h1 = 1.8, w1 = 0.05, h2 = 0.2, w2 = 2, s = 0.13;
			std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2, w2)};
			phyz::Geometry geom1;
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w1/2, -h1/2, -w1/2, w1, h1, w1));
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w2/2, -h2/2 -h1*s/2, -w2/2, w2, h2, w2));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->trackPoint(mthz::Vec3(0, 0, 0));
			r1->setCOMtoPosition(mthz::Vec3(0, 10, 0));
			r1->setAngVel(mthz::Vec3(0, 40, 2.5));
			bodies.push_back({ m1, r1 });
		}

		//barbell
		if (rndr::getKeyPressed(GLFW_KEY_L)) {
			double h1 = 1.8, w1 = 0.2, h2 = 0.5, w2 = 1, s = 0.5;
			std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2, w2), rect(0, h1 * s / 2, 0, w2, h2, w2) };
			phyz::Geometry geom1;
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w1 / 2, -h1 / 2, -w1 / 2, w1, h1, w1));
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w2 / 2, -h2 / 2 - h1 * s / 2, -w2 / 2, w2, h2, w2));
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w2/2, -h2 / 2 + h1 * s / 2, -w2/2, w2, h2, w2));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->trackPoint(mthz::Vec3(0, 0, 0));
			r1->setCOMtoPosition(mthz::Vec3(0, 10, 0));
			//r1->orientation = mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(0, 0, 1));
			r1->setAngVel(mthz::Vec3(0, 6, 1.4));
			//r1->vel = mthz::Vec3(10, 0, 0);
			bodies.push_back({ m1, r1 });
		}

		//unbalanced_barbell
		if (rndr::getKeyPressed(GLFW_KEY_U)) {
			double h1 = 2.8, w1 = 0.2, h2 = 0.5, w2 = 1, s = 0.5;
			std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2/10, w2), rect(0, h1 * s / 2, 0, w2 * 3/4.0, h2, w2 * 3/4.0) };
			phyz::Geometry geom1;
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w1 / 2, -h1 / 2, -w1 / 2, w1, h1, w1));
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w2 / 2, -h2 / 20 - h1 * s / 2, -w2 / 2, w2, h2/10, w2));
			geom1.pushConvexPoly(phyz::ConvexPoly::box(-w2 * 3 / 8.0, -h2 / 2 + h1 * s / 2, -w2  * 3 / 8.0, w2 * 3/4.0, h2, w2 * 3/4.0));
			phyz::RigidBody* r1 = p.createRigidBody(geom1);
			phyz::RigidBody::PKey draw_p = r1->trackPoint(mthz::Vec3(0, 0, 0));
			r1->setCOMtoPosition(mthz::Vec3(0, 10, 0));
			r1->setOrientation(mthz::Quaternion(0.67895851999693979, 0.0070540392161153693, 0.022756044727275559, 0.73378997750219643));
			r1->setAngVel(mthz::Vec3(0, 0.5, 10));
			//r1->vel = mthz::Vec3(10, 0, 0);
			bodies.push_back({ m1, r1 });
		}

		//chain
		if (rndr::getKeyPressed(GLFW_KEY_O)) {
			double end_y = 10;
			mthz::Vec3 link_dimensions(0.25, 0.85, 0.25);
			int n_links = 50;
			
			phyz::RigidBody* prev_link = nullptr;
			for (int i = 0; i < n_links; i++) {
				std::vector<Mesh> m1 = { rect(0, 0, 0, link_dimensions.x, link_dimensions.y, link_dimensions.z) };
				phyz::Geometry geom1;
				geom1.pushConvexPoly(phyz::ConvexPoly::box(-link_dimensions.x / 2.0, -link_dimensions.y / 2.0, -link_dimensions.z / 2.0, link_dimensions.x, link_dimensions.y, link_dimensions.z));
				phyz::RigidBody* r1 = p.createRigidBody(geom1);
				phyz::RigidBody::PKey draw_p1 = r1->trackPoint(mthz::Vec3(0, 0, 0));
				bodies.push_back({ m1, r1 });
				double y = end_y + link_dimensions.y * (n_links - i);
				r1->setCOMtoPosition(mthz::Vec3(0, y, 0));

				if (prev_link != nullptr) {
					p.addBallSocketConstraint(r1, prev_link, mthz::Vec3(0, y + link_dimensions.y/2.0, 0));
				}
				prev_link = r1;
			}
		}

		//simple hinge
		if (rndr::getKeyPressed(GLFW_KEY_P)) {
			double end_y = 1;
			mthz::Vec3 box1_dimensions(1, 0.5, 1);
			mthz::Vec3 box2_dimensions(0.5, 0.5, 0.5);

			phyz::RigidBody* r1 = box(&bodies, &p, mthz::Vec3(0, end_y, 0), box1_dimensions);
			phyz::RigidBody* r2 = box(&bodies, &p, mthz::Vec3(0, end_y + box1_dimensions.y / 2.0 + box2_dimensions.y / 2.0, 0), box2_dimensions);
			
			p.addHingeConstraint(r1, r2, mthz::Vec3(0, end_y + box1_dimensions.y / 2.0, 0), mthz::Vec3(0, 1, 0));
			
		}

		//car
		if (rndr::getKeyPressed(GLFW_KEY_C)) {

			mthz::Vec3 pos(0, 10, 0);
			mthz::Vec3 chasis_dim(3, 0.25, 1.3);
			mthz::Vec3 wheel_dim(1.25, 1.25, 0.15);
			double wheel_spacing = 0.66;

			phyz::RigidBody* chasis = box(&bodies, &p, pos, chasis_dim);
			phyz::RigidBody* w1 = box(&bodies, &p, pos + mthz::Vec3(chasis_dim.x * wheel_spacing / 2.0, 0, chasis_dim.z / 2.0), wheel_dim);
			phyz::RigidBody* w2 = box(&bodies, &p, pos + mthz::Vec3(-chasis_dim.x * wheel_spacing / 2.0, 0, chasis_dim.z / 2.0), wheel_dim);
			phyz::RigidBody* w3 = box(&bodies, &p, pos + mthz::Vec3(chasis_dim.x * wheel_spacing / 2.0, 0, -chasis_dim.z / 2.0), wheel_dim);
			phyz::RigidBody* w4 = box(&bodies, &p, pos + mthz::Vec3(-chasis_dim.x * wheel_spacing / 2.0, 0, -chasis_dim.z / 2.0), wheel_dim);

			p.addHingeConstraint(chasis, w1, w1->getCOM(), mthz::Vec3(0, 0, 1));
			p.addHingeConstraint(chasis, w2, w2->getCOM(), mthz::Vec3(0, 0, 1));
			p.addHingeConstraint(chasis, w3, w3->getCOM(), mthz::Vec3(0, 0, -1));
			p.addHingeConstraint(chasis, w4, w4->getCOM(), mthz::Vec3(0, 0, -1));
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
				shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0) * rndr::Mat4::cam_view(pos, orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
				shader.setUniform1i("u_Asleep", b.r->getAsleep());
				rndr::draw(*m.va, *m.ib, shader);
			}
		}
	}

	rndr::terminate();
	return 0;
}