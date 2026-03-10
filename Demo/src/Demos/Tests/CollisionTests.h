#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

struct coltest_pair {
	phyz::RigidBody* dynamic_body;
	phyz::RigidBody* static_body;
};
struct coltest_orientation {
	mthz::Quaternion dynamic_body_orientation;
	mthz::Quaternion static_body_orientation;
	std::string id;
};
std::map<std::string, coltest_pair> setupMultipleOrientations(phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, const phyz::ConvexUnionGeometry& dynamic_body_geom, const phyz::ConvexUnionGeometry& static_body_geom, const std::vector<coltest_orientation>& orientations_to_test, double dynamic_body_verticle_offset=5, double spacing=5);

#ifndef MUL_ORIENTATION_IMPL
#define MUL_ORIENTATION_IMPL
std::map<std::string, coltest_pair> setupMultipleOrientations(phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, const phyz::ConvexUnionGeometry& dynamic_body_geom, const phyz::ConvexUnionGeometry& static_body_geom, const std::vector<coltest_orientation>& orientations_to_test, double dynamic_body_verticle_offset, double spacing) {
	std::map<std::string, coltest_pair> out;
	int grid_size = std::ceil(sqrt(orientations_to_test.size()));
	mthz::Vec3 start_corner = mthz::Vec3(-spacing * grid_size / 2.0, 0, -spacing * grid_size / 2.0);

	int i = 0;
	for (coltest_orientation conf : orientations_to_test) {
		int row = i / grid_size;
		int col = i % grid_size;
		i++;

		mthz::Vec3 pos = start_corner + spacing * mthz::Vec3(col, 0, row);
		phyz::ConvexUnionGeometry transf_static = static_body_geom.getRotated(conf.static_body_orientation).getTranslated(pos);
		phyz::ConvexUnionGeometry transf_dynamic = dynamic_body_geom.getRotated(conf.dynamic_body_orientation).getTranslated(pos + mthz::Vec3(0, dynamic_body_verticle_offset, 0));

		coltest_pair pa;
		pa.dynamic_body = p->createRigidBody(transf_dynamic); 
		pa.static_body = p->createRigidBody(transf_static, phyz::RigidBody::FIXED);
		bodies->push_back(PhysBod{ fromGeometry(transf_dynamic), pa.dynamic_body });
		bodies->push_back(PhysBod{ fromGeometry(transf_static), pa.static_body });
		out[conf.id] = pa;
	}

	return out;
}
#endif

class TestBasicBoxAgainstGround : public Test {
public:
	std::string getTestName() const override { return "Box vs Ground"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		collision_occured = false;
		no_incorrect_manifolds = true;
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(5 * tick_frequency);
		test_current_tick_count = 0;
		

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);
		
		// create ground plane
		double ground_width = 100.0;
		double ground_thickness = 0.5;
		phyz::ConvexUnionGeometry ground_geom = phyz::ConvexUnionGeometry::box(mthz::Vec3(-ground_width / 2.0, -ground_thickness, -ground_width / 2.0), ground_width, ground_thickness, ground_width);
		phyz::RigidBody* ground_r = p->createRigidBody(ground_geom, phyz::RigidBody::MovementType::FIXED);
		bodies->push_back(PhysBod{ fromGeometry(ground_geom), ground_r });

		//create box
		box_size = 1.0;
		phyz::ConvexUnionGeometry box_geom = phyz::ConvexUnionGeometry::box(mthz::Vec3(0, 5.0, 0), box_size, box_size, box_size);
		box_r = p->createRigidBody(box_geom);
		bodies->push_back(PhysBod{ fromGeometry(box_geom), box_r });

		p->registerCollisionAction(phyz::CollisionTarget::with(ground_r), phyz::CollisionTarget::with(box_r), [this](phyz::RigidBody*, phyz::RigidBody*, const std::vector<phyz::Manifold>& manifolds) {
			this->collision_occured = true;
			if (manifolds.size() > 1) { this->no_incorrect_manifolds = false; return; }
			mthz::Vec3 norm = manifolds.front().normal;
			if (abs(norm.dot(mthz::Vec3(0, 1, 0))) < 0.9999) { this->no_incorrect_manifolds = false; return; }
		});

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
		}

		test_current_tick_count++;

		if (test_current_tick_count >= test_total_tick_duration) {
			if (box_r->getVel().mag() > 0.00001) {
				return TestOutcome{ TestOutcomeState::FAILED, "Box had excessive linear velocity at the end of the simulation" };
			}
			else if (box_r->getAngVel().mag() > 0.00001) {
				return TestOutcome{ TestOutcomeState::FAILED, "Box had excessive angular velocity at the end of the simulation" };
			}
			else if (abs(box_r->getCOM().y - box_size / 2.0) > 0.1) { 
				return TestOutcome{ TestOutcomeState::FAILED, "Center of the box was not resting at the expected height" };
			}
			else if (!collision_occured) {
				return TestOutcome{ TestOutcomeState::FAILED, "Failed to detect collision between the ground and the box" };
			}
			else if (!no_incorrect_manifolds) {
				return TestOutcome{ TestOutcomeState::FAILED, "Generated collision manifolds were incorrect" };
			}
			else {
				return TestOutcome{ TestOutcomeState::PASSED };
			}
		}
		else { return TestOutcome{ TestOutcomeState::STILL_RUNNING }; }
	}
	void teardownTest() override { 
		delete p;
		p = nullptr;
		box_r = nullptr;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	phyz::RigidBody* box_r;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
	double box_size;
	bool collision_occured;
	bool no_incorrect_manifolds;
};

class TestCapsuleAgainstSphere : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Sphere"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(5 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		double radius = 0.75;
		phyz::ConvexUnionGeometry sphere_geom = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), radius);

		//create capsule
		double capsule_radius = 0.75;
		double capsule_drum_height = 1.5;
		phyz::ConvexUnionGeometry capsule_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(0, - capsule_drum_height/2.0, 0), capsule_radius, capsule_drum_height);
		
		setupMultipleOrientations(p, bodies, sphere_geom, capsule_geom, {
			coltest_orientation{ mthz::Quaternion(), mthz::Quaternion(), "sphere against top cap"},
			coltest_orientation{ mthz::Quaternion(), mthz::Quaternion(PI/2.0, mthz::Vec3(0, 0, 1.0)), "sphere against barrel"},
			coltest_orientation{ mthz::Quaternion(), mthz::Quaternion(PI, mthz::Vec3(0, 0, 1.0)), "sphere against bot cap"},
		});

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
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
};

class TestCapsuleAgainstPolyhedron : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Polyhedron"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(5 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		double box_size = 1.5;
		phyz::ConvexUnionGeometry cube_geom = phyz::ConvexUnionGeometry::box(mthz::Vec3(-box_size/2.0, -box_size/2.0, -box_size/2.0), box_size, box_size, box_size);
		mthz::Quaternion edge_down_orientation = mthz::Quaternion(PI / 4.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion corner_down_orientation = mthz::Quaternion(atan(sqrt(0.5)), mthz::Vec3(1.0, 0, 0)) * mthz::Quaternion(PI / 4.0, mthz::Vec3(0, 0, 1.0));
		//mthz::Quaternion corner_down_orientation = mthz::Quaternion(PI / 4.0, mthz::Vec3(1.0, 0, 0)) * mthz::Quaternion(PI / 4.0, mthz::Vec3(0, 0, 1.0));


		//create capsule
		double capsule_radius = 0.75;
		double capsule_drum_height = 1.5;
		phyz::ConvexUnionGeometry capsule_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(0, -capsule_drum_height / 2.0, 0), capsule_radius, capsule_drum_height);
		mthz::Quaternion sideways = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion upside_down = mthz::Quaternion(PI, mthz::Vec3(0, 0, 1.0));

		setupMultipleOrientations(p, bodies, cube_geom, capsule_geom, {
			coltest_orientation{ mthz::Quaternion(), mthz::Quaternion(), "face against top cap"},
			coltest_orientation{ mthz::Quaternion(), sideways, "face against barrel"},
			coltest_orientation{ mthz::Quaternion(), upside_down, "face against bot cap"},
			coltest_orientation{ edge_down_orientation, upside_down, "edge against top cap"},
			coltest_orientation{ edge_down_orientation, sideways, "edge against barrel"},
			coltest_orientation{ edge_down_orientation, upside_down, "edge against bot cap"},
			coltest_orientation{ corner_down_orientation, upside_down, "vertex against top cap"},
			coltest_orientation{ corner_down_orientation, sideways, "vertex against barrel"},
			coltest_orientation{ corner_down_orientation, upside_down, "vertex against bot cap"},
		});

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
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
};

class TestCapsuleAgainstCapsule : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Capsule"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(5 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		//create capsule
		double capsule_radius = 0.75;
		double capsule_drum_height = 1.5;
		phyz::ConvexUnionGeometry capsule_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(0, -capsule_drum_height / 2.0, 0), capsule_radius, capsule_drum_height);
		mthz::Quaternion sideways = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion sideways_turned = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 1.0, 0)) * mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion upside_down = mthz::Quaternion(PI, mthz::Vec3(0, 0, 1.0));

		setupMultipleOrientations(p, bodies, capsule_geom, capsule_geom, {
			coltest_orientation{ mthz::Quaternion(), mthz::Quaternion(), "bot cap against top cap"},
			coltest_orientation{ mthz::Quaternion(), sideways, "bot cap against barrel"},
			coltest_orientation{ mthz::Quaternion(), upside_down, "bot cap against bot cap"},
			coltest_orientation{ sideways_turned, mthz::Quaternion(), "barrel against top cap"},
			coltest_orientation{ sideways_turned, sideways, "barrel against barrel"},
			coltest_orientation{ sideways, sideways, "barrel against barrel parallel"},
			coltest_orientation{ sideways_turned, upside_down, "barrel against bot cap"},
			coltest_orientation{ upside_down, mthz::Quaternion(), "bot cap against top cap"},
			coltest_orientation{ upside_down, sideways, "bot cap against barrel"},
			coltest_orientation{ upside_down, upside_down, "bot cap against bot cap"},
		});

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
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
};

class TestCapsuleAgainstCylinder : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Cylinder"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(5 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		//create cylinder
		double cylinder_radius = 0.5;
		double cylinder_height = 1;
		phyz::ConvexUnionGeometry cylinder_geom = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -cylinder_height / 2.0, 0), cylinder_radius, cylinder_height);
		mthz::Quaternion barrel_down = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion upside_down = mthz::Quaternion(PI, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion bot_edge_down = mthz::Quaternion(PI / 4.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion top_edge_down = mthz::Quaternion(PI * 5.0 / 4.0, mthz::Vec3(0, 0, 1.0));

		//create capsule
		double capsule_radius = 0.75;
		double capsule_drum_height = 1.5;
		phyz::ConvexUnionGeometry capsule_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(0, -capsule_drum_height / 2.0, 0), capsule_radius, capsule_drum_height);
		mthz::Quaternion sideways = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion sideways_turned = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 1.0, 0)) * mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));

		setupMultipleOrientations(p, bodies, cylinder_geom, capsule_geom, {
			coltest_orientation{ mthz::Quaternion(), mthz::Quaternion(), "bot face against top cap"},
			coltest_orientation{ mthz::Quaternion(), sideways, "bot face against barrel"},
			coltest_orientation{ mthz::Quaternion(), upside_down, "bot face against bot cap"},
			coltest_orientation{ barrel_down, mthz::Quaternion(), "barrel against top cap"},
			coltest_orientation{ barrel_down, sideways_turned, "barrel against barrel"},
			coltest_orientation{ barrel_down, sideways, "barrel against barrel parallel"},
			coltest_orientation{ barrel_down, upside_down, "barrel against bot cap"},
			coltest_orientation{ bot_edge_down, mthz::Quaternion(), "bot edge against top cap"},
			coltest_orientation{ bot_edge_down, sideways, "bot edge against barrel"},
			coltest_orientation{ bot_edge_down, upside_down, "bot edge against bot cap"},
			coltest_orientation{ top_edge_down, mthz::Quaternion(), "top edge against top cap"},
			coltest_orientation{ top_edge_down, sideways, "top edge against barrel"},
			coltest_orientation{ top_edge_down, upside_down, "top edge against bot cap"},
		}, 5.0);

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
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
};

class CollisionTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "Collision"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestBasicBoxAgainstGround>());
		out.push_back(std::make_unique<TestCapsuleAgainstSphere>());
		out.push_back(std::make_unique<TestCapsuleAgainstPolyhedron>());
		out.push_back(std::make_unique<TestCapsuleAgainstCapsule>());
		out.push_back(std::make_unique<TestCapsuleAgainstCylinder>());
		return out;
	}
};