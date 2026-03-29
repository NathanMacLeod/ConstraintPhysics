#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

class TestGeomAgainstSingleSquareMesh : public Test {
public:
	TestGeomAgainstSingleSquareMesh(const phyz::ConvexUnionGeometry& geom, const std::string& test_name)
		: geom(geom), mesh_roughness(mesh_roughness), test_name(test_name)
	{}

	std::string getTestName() const override { return test_name; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(30 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		// init mesh
		double size = 5.0;
		phyz::MeshInput mesh = phyz::generateGridMeshInput(1, 1, size, mthz::Vec3(-size/2.0, 0, -size/2.0));
		bodies->push_back(PhysBod{ fromStaticMeshInput(mesh, color{0.3f, 0.3f, 0.3f}), p->createRigidBody(mesh) });

		// create body from geom
		const double y_spawn_offset = 3;
		mthz::Vec3 pos = mthz::Vec3(0, y_spawn_offset, 0);
		phyz::ConvexUnionGeometry translated_geom = geom.getTranslated(pos);
		body_r = p->createRigidBody(translated_geom);
		bodies->push_back(PhysBod(fromGeometry(translated_geom), body_r));

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
		}

		test_current_tick_count++;

		// just check that if anything fell off, it fell off the edge.
		//for (phyz::RigidBody* r : rigid_bodies) {
		//	mthz::Vec3 com = r->getCOM();
		//	if (com.y < -10) {
		//		double radius2 = com.x * com.x + com.z * com.z;
		//		if (radius2 < RADIUS * RADIUS) {
		//			return TestOutcome{ TestOutcomeState::FAILED, "Body fell through the mesh" };
		//		}
		//	}
		//}


		if (test_current_tick_count >= test_total_tick_duration) {
			return TestOutcome{ TestOutcomeState::PASSED };
		}
		return TestOutcome{ TestOutcomeState::STILL_RUNNING };

	}
	void teardownTest() override {
		delete p;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
	phyz::RigidBody* body_r;

	static constexpr double RADIUS = 50;
	phyz::ConvexUnionGeometry geom;
	double mesh_roughness;
	std::string test_name;
};

class TestGeomAgainstBumpyMesh : public Test {
public:
	TestGeomAgainstBumpyMesh(const phyz::ConvexUnionGeometry& geom, double mesh_roughness, const std::string& test_name)
		: geom(geom), mesh_roughness(mesh_roughness), test_name(test_name)
	{}

	std::string getTestName() const override { return test_name; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(30 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		// init mesh
		double seg_size = 0.5;
		int seg_count = RADIUS / seg_size;
		phyz::MeshInput mesh = phyz::generateRadialMeshInput(13, seg_count, seg_size);
		srand(0);
		for (int i = 0; i < mesh.points.size(); i++) {
			mesh.points[i].y += (2.0 * frand() - 1.0) * mesh_roughness;
		}
		bodies->push_back(PhysBod{ fromStaticMeshInput(mesh, color{0.3f, 0.3f, 0.3f}), p->createRigidBody(mesh) });
		
		// create bodies from geom
		srand(0);
		const double y_spawn_offset = 2;
		for (int i = 0; i < 30; i++) {
			mthz::Vec3 delta = 0.5 * mthz::Vec3(2.0 * frand() - 1, 2.0 * frand() - 1, 2.0 * frand() - 1);
			mthz::Vec3 pos = delta + mthz::Vec3(0, (i + 1) * y_spawn_offset, 0);
			phyz::ConvexUnionGeometry translated_geom = geom.getTranslated(pos);
			phyz::RigidBody* r = p->createRigidBody(translated_geom);
			rigid_bodies.push_back(r);
			bodies->push_back(PhysBod(fromGeometry(translated_geom), r));
		}

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
		}

		test_current_tick_count++;

		// just check that if anything fell off, it fell off the edge.
		for (phyz::RigidBody* r : rigid_bodies) {
			mthz::Vec3 com = r->getCOM();
			if (com.y < -10) {
				double radius2 = com.x * com.x + com.z * com.z;
				if (radius2 < RADIUS * RADIUS) { 
					return TestOutcome{ TestOutcomeState::FAILED, "Body fell through the mesh" };
				}
			}
		}


		if (test_current_tick_count >= test_total_tick_duration) {
			return TestOutcome{ TestOutcomeState::PASSED };
		}
		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
		
	}
	void teardownTest() override {
		delete p;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
	std::vector<phyz::RigidBody*> rigid_bodies;

	static constexpr double RADIUS = 50;
	phyz::ConvexUnionGeometry geom;
	double mesh_roughness;
	std::string test_name;
};

class TriangleMeshTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "Static Triangle Mesh"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;

		double flat = 0;
		double minor_rough = 0.1;
		double medium_rough = 0.5;
		double extreme_rough = 2.0;

		// sphere against mesh
		phyz::ConvexUnionGeometry sphere = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.5);
		out.push_back(std::make_unique<TestGeomAgainstSingleSquareMesh>(sphere, "Spheres vs Square Mesh"));
		out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, flat, "Spheres vs Flat Mesh"));
		out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, minor_rough, "Spheres vs Minor Rough"));
		out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, medium_rough, "Spheres vs Medium Rough"));
		out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, extreme_rough, "Spheres vs Extreme Rough"));

		// simple poly against mesh
		phyz::ConvexUnionGeometry polyhedron = phyz::ConvexUnionGeometry::regDodecahedron(mthz::Vec3(), 1.0);
		out.push_back(std::make_unique<TestGeomAgainstSingleSquareMesh>(polyhedron, "Polyhedron vs Square Mesh"));


		//composite bovine 
		MeshColliderOutput cow_mc = readMeshAndColliders("resources/mesh/cow_col.obj", 0.15);
		phyz::ConvexUnionGeometry cow_geom = {
			cow_mc.colliders["back_left"], cow_mc.colliders["back_right"], cow_mc.colliders["body"], cow_mc.colliders["ear_left"], cow_mc.colliders["ear_right"],
			cow_mc.colliders["front_left"], cow_mc.colliders["front_right"], cow_mc.colliders["head"], cow_mc.colliders["horn_base"], cow_mc.colliders["horn_tip_left"],
			cow_mc.colliders["horn_tip_right"], cow_mc.colliders["neck"], cow_mc.colliders["tail"]
		};
		out.push_back(std::make_unique<TestGeomAgainstSingleSquareMesh>(cow_geom, "Bovine vs Square Mesh"));
		return out;
	}
};