#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

class TestGeomAgainstSimpleMesh : public Test {
public:
	TestGeomAgainstSimpleMesh(const phyz::ConvexUnionGeometry& geom, const phyz::MeshInput& mesh, const std::string& test_name)
		: geom(geom), mesh_roughness(mesh_roughness), test_name(test_name), mesh(mesh)
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

	phyz::MeshInput mesh;
	phyz::ConvexUnionGeometry geom;
	double mesh_roughness;
	std::string test_name;
};

class TestVertexVsVertexCollision : public Test {
public:
	TestVertexVsVertexCollision() {}

	std::string getTestName() const override { return "Vertex against Vertex of nearly flat mesh"; }
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
		phyz::MeshInput corner_up_mesh = phyz::generateRadialMeshInput(3, 1, 3.0, mthz::Vec3(0, 0, 0));
		corner_up_mesh.points[0].y += 0.1;
		bodies->push_back(PhysBod{ fromStaticMeshInput(corner_up_mesh, color{0.3f, 0.3f, 0.3f}), p->createRigidBody(corner_up_mesh) });

		// create body from geom
		const double y_spawn_offset = 30;
		mthz::Vec3 pos = mthz::Vec3(0, y_spawn_offset, 0);
		phyz::ConvexUnionGeometry box = phyz::ConvexUnionGeometry::box(mthz::Vec3(-0.5, -0.5, -0.5), 1.0, 1.0, 1.0)
										.getRotated(mthz::Quaternion(PI / 4.0, mthz::Vec3(0, 0, 1)))
										.getRotated(mthz::Quaternion(asin(1.0 / sqrt(3)), mthz::Vec3(1, 0, 0))); //rotate to point corner downwards
		phyz::ConvexUnionGeometry translated_geom = box.getTranslated(pos);
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
};

class TestGeomAgainstBumpyMesh : public Test {
public:
	TestGeomAgainstBumpyMesh(const phyz::ConvexUnionGeometry& geom, uint32_t geom_create_count, double mesh_roughness, const std::string& test_name)
		: geom(geom), mesh_roughness(mesh_roughness), test_name(test_name), geom_create_count(geom_create_count)
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
		for (uint32_t i = 0; i < geom_create_count; i++) {
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
	uint32_t geom_create_count;
	double mesh_roughness;
	std::string test_name;
};

class TriangleMeshTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "Static Triangle Mesh"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;

		phyz::ConvexUnionGeometry box = phyz::ConvexUnionGeometry::box(mthz::Vec3(-0.5, -0.5, -0.5), 1.0, 1.0, 1.0);
		phyz::ConvexUnionGeometry box_edge = box.getRotated(mthz::Quaternion(PI / 4.0, mthz::Vec3(0, 0, 1)));
		phyz::ConvexUnionGeometry box_corner = box_edge.getRotated(mthz::Quaternion(asin(1.0/sqrt(3)), mthz::Vec3(1, 0, 0)));
		phyz::ConvexUnionGeometry sphere = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.5);
		phyz::ConvexUnionGeometry polyhedron = phyz::ConvexUnionGeometry::regDodecahedron(mthz::Vec3(), 1.0);
		phyz::ConvexUnionGeometry capsule = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(), 1.0, 1.0);
		phyz::ConvexUnionGeometry capsule_flat = capsule.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)));
		phyz::ConvexUnionGeometry cylinder = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -0.5, 0), 0.5, 1.0);
		phyz::ConvexUnionGeometry cylinder_drum = cylinder.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)));
		phyz::ConvexUnionGeometry cylinder_edge = cylinder.getRotated(mthz::Quaternion(PI / 4.0, mthz::Vec3(0, 0, 1)));
		//composite bovine 
		/*MeshColliderOutput cow_mc = readMeshAndColliders("resources/mesh/cow_col_fixed.obj", 0.15);
		phyz::ConvexUnionGeometry cow_geom = {
			cow_mc.colliders["back_left"], cow_mc.colliders["back_right"], cow_mc.colliders["body"], cow_mc.colliders["ear_left"], cow_mc.colliders["ear_right"],
			cow_mc.colliders["front_left"], cow_mc.colliders["front_right"], cow_mc.colliders["head"], cow_mc.colliders["horn_base"], cow_mc.colliders["horn_tip_left"],
			cow_mc.colliders["horn_tip_right"], cow_mc.colliders["neck"], cow_mc.colliders["tail"]
		};*/

		{ // one object against simple mesh_types
			double size = 5.0;
			phyz::MeshInput flat_square_mesh = phyz::generateGridMeshInput(1, 1, size, mthz::Vec3(-size / 2.0, 0, -size / 2.0));
			phyz::MeshInput edge_up_mesh = phyz::generateGridMeshInput(1, 1, size, mthz::Vec3(-size / 2.0, 0, -size / 2.0));
			edge_up_mesh.points[0].y += 1; edge_up_mesh.points[3].y += 1;
			phyz::MeshInput corner_up_mesh = phyz::generateRadialMeshInput(3, 1, size, mthz::Vec3(0, 0, 0));
			corner_up_mesh.points[0].y += 1;

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box, flat_square_mesh, "Box vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box, edge_up_mesh, "Box vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box, corner_up_mesh, "Box vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box_edge, flat_square_mesh, "Box Edge vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box_edge, edge_up_mesh, "Box Edge vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box_edge, corner_up_mesh, "Box Edge vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box_corner, flat_square_mesh, "Box Corner vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box_corner, edge_up_mesh, "Box Corner vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(box_corner, corner_up_mesh, "Box Corner vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(sphere, flat_square_mesh, "Sphere vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(sphere, edge_up_mesh, "Sphere vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(sphere, corner_up_mesh, "Sphere vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(capsule, flat_square_mesh, "Capsule vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(capsule, edge_up_mesh, "Capsule vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(capsule, corner_up_mesh, "Capsule vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(capsule_flat, flat_square_mesh, "Capsule Drum vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(capsule_flat, edge_up_mesh, "Capsule Drum vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(capsule_flat, corner_up_mesh, "Capsule Drum vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder, flat_square_mesh, "Cylinder Flat vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder, edge_up_mesh, "Cylinder Flat vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder, corner_up_mesh, "Cylinder Flat vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder_drum, flat_square_mesh, "Cylinder Drum vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder_drum, edge_up_mesh, "Cylinder Drum vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder_drum, corner_up_mesh, "Cylinder Drum vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder_edge, flat_square_mesh, "Cylinder Edge vs Mesh Flat"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder_edge, edge_up_mesh, "Cylinder Edge vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(cylinder_edge, corner_up_mesh, "Cylinder Edge vs Mesh Corner"));

			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(polyhedron, flat_square_mesh, "Polyhedron vs Square Mesh"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(polyhedron, edge_up_mesh, "Polyhedron vs Mesh Edge"));
			out.push_back(std::make_unique<TestGeomAgainstSimpleMesh>(polyhedron, corner_up_mesh, "Polyhedron vs Mesh Corner"));
		}

		{ // spawning many geom objects against bumpy terrain
			double flat = 0;
			double minor_rough = 0.1;
			double medium_rough = 0.5;
			double extreme_rough = 2.0;

			// sphere against mesh
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, 30, flat, "Spheres vs Flat Mesh"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, 30, minor_rough, "Spheres vs Minor Rough"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, 30, medium_rough, "Spheres vs Medium Rough"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, 30, extreme_rough, "Spheres vs Extreme Rough"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, 1, medium_rough, "Single Sphere vs Extreme Rough"));

			// simple poly against mesh
			
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(polyhedron, 30, flat, "Polyhedrons vs Flat Mesh"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(polyhedron, 30, minor_rough, "Polyhedrons vs Minor Rough"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(polyhedron, 30, medium_rough, "Polyhedrons vs Medium Rough"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(polyhedron, 30, extreme_rough, "Polyhedrons vs Extreme Rough"));
			out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(sphere, 1, medium_rough, "Single Polyhedron vs Extreme Rough"));
		}

		{ // square corner against a vertex on a nearly flat mesh. 
			out.push_back(std::make_unique<TestVertexVsVertexCollision>());
		}
		
		//out.push_back(std::make_unique<TestGeomAgainstSingleSquareMesh>(cow_geom, "Bovine vs Square Mesh"));
		//out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(cow_geom, flat, "Bovine vs Flat Mesh"));
		//out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(cow_geom, minor_rough, "Bovine vs Minor Rough"));
		//out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(cow_geom, medium_rough, "Bovine vs Medium Rough"));
		//out.push_back(std::make_unique<TestGeomAgainstBumpyMesh>(cow_geom, extreme_rough, "Bovine vs Extreme Rough"));
		return out;
	}
};