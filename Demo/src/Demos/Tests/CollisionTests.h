#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

struct coltest_orientation {
	mthz::Quaternion dynamic_body_orientation;
	mthz::Quaternion static_body_orientation;
	std::string id;
};
struct coltest_result {
	bool collision_occured;
	std::string id;
	TestOutcome outcome;
};

std::vector<std::shared_ptr<coltest_result>> setupMultipleOrientations(phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, const phyz::ConvexUnionGeometry& dynamic_body_geom, const phyz::ConvexUnionGeometry& static_body_geom, const std::vector<coltest_orientation>& orientations_to_test, double dynamic_body_verticle_offset=5, double spacing=5);

#ifndef MUL_ORIENTATION_IMPL
#define MUL_ORIENTATION_IMPL
std::vector<std::shared_ptr<coltest_result>> setupMultipleOrientations(phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, const phyz::ConvexUnionGeometry& dynamic_body_geom, const phyz::ConvexUnionGeometry& static_body_geom, const std::vector<coltest_orientation>& orientations_to_test, double dynamic_body_verticle_offset, double spacing) {
	std::vector<std::shared_ptr<coltest_result>> out;
	int grid_size = static_cast<int>(std::ceil(sqrt(orientations_to_test.size())));
	mthz::Vec3 start_corner = mthz::Vec3(-spacing * grid_size / 2.0, 0, -spacing * grid_size / 2.0);

	int i = 0;
	for (coltest_orientation conf : orientations_to_test) {
		int row = i / grid_size;
		int col = i % grid_size;
		i++;

		mthz::Vec3 pos = start_corner + spacing * mthz::Vec3(col, 0, row);
		phyz::ConvexUnionGeometry transf_static = static_body_geom.getRotated(conf.static_body_orientation).getTranslated(pos);
		phyz::ConvexUnionGeometry transf_dynamic = dynamic_body_geom.getRotated(conf.dynamic_body_orientation).getTranslated(pos + mthz::Vec3(0, dynamic_body_verticle_offset, 0));

		phyz::RigidBody* dynamic_body = p->createRigidBody(transf_dynamic); 
		phyz::RigidBody* static_body = p->createRigidBody(transf_static, phyz::RigidBody::FIXED);
		bodies->push_back(PhysBod{ fromGeometry(transf_dynamic), dynamic_body });
		bodies->push_back(PhysBod{ fromGeometry(transf_static), static_body });
		
		std::shared_ptr<coltest_result> result = std::make_shared<coltest_result>();
		result->collision_occured = false;
		result->id = conf.id;

		double expected_y_coord_of_contact_point = static_body->getAABB().max.y;

		p->registerCollisionAction(phyz::CollisionTarget::with(dynamic_body), phyz::CollisionTarget::with(static_body), [result, expected_y_coord_of_contact_point](phyz::RigidBody* b1, phyz::RigidBody* b2, const std::vector<phyz::Manifold>& manifolds) {
			if (result->collision_occured) return;
			result->collision_occured = true;

			for (const phyz::Manifold& m : manifolds) {
				// normal is expected to be vertical
				if (abs(m.normal.dot(mthz::Vec3(0, 1, 0))) < 0.99) {
					result->outcome = TestOutcome{ TestOutcomeState::FAILED, std::format("for pair {} manifold had a non-vertical normal", result->id) };
					return;
				}

				for (phyz::ContactP p : m.points) {
					if (abs(p.pos.y - expected_y_coord_of_contact_point) > 0.1) {
						result->outcome = TestOutcome{ TestOutcomeState::FAILED, std::format("for pair {} a contact point did not have the expected y coordinate", result->id) };
						return;
					}
				}
			}

			result->outcome = TestOutcome{ TestOutcomeState::PASSED };
		});

		out.push_back(result);
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


// todo: these shape v shape tests are super repetetive. seems refactorable to reduce copy paste.
class TestCapsuleAgainstSphere : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Sphere"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		running_without_graphics = false; // will be set to true later if we are not using graphics
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
		
		outcomes = setupMultipleOrientations(p, bodies, sphere_geom, capsule_geom, {
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

		test_current_tick_count++;

		bool all_collisions_occured = true;
		for (const std::shared_ptr<coltest_result>& r : outcomes) {
			if (!r->collision_occured) { all_collisions_occured = false; }
			// return any failed collisions
			if (r->collision_occured && r->outcome.state == TestOutcomeState::FAILED) { return r->outcome; }
		}

		// if all collisions occured the test is passed based on the validations. but it's nice to run longer for visual validation.
		if (running_without_graphics && all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }

		if (test_current_tick_count >= test_total_tick_duration) {
			if (all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }
			
			// missing collisions
			for (const std::shared_ptr<coltest_result>& r : outcomes) {
				if (!r->collision_occured) { return TestOutcome{ TestOutcomeState::FAILED, std::format("pair {} did not collide", r->id)}; }
			}
		}

		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
	};
	TestOutcome runWithoutGraphics() override {
		running_without_graphics = true;
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
	bool running_without_graphics;
	std::vector<std::shared_ptr<coltest_result>> outcomes;
};

class TestCapsuleAgainstPolyhedron : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Polyhedron"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		running_without_graphics = false; // will be set to true later if we are not using graphics
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

		outcomes = setupMultipleOrientations(p, bodies, cube_geom, capsule_geom, {
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

		test_current_tick_count++;

		bool all_collisions_occured = true;
		for (const std::shared_ptr<coltest_result>& r : outcomes) {
			if (!r->collision_occured) { all_collisions_occured = false; }
			// return any failed collisions
			if (r->collision_occured && r->outcome.state == TestOutcomeState::FAILED) { return r->outcome; }
		}

		// if all collisions occured the test is passed based on the validations. but it's nice to run longer for visual validation.
		if (running_without_graphics && all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }

		if (test_current_tick_count >= test_total_tick_duration) {
			if (all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }

			// missing collisions
			for (const std::shared_ptr<coltest_result>& r : outcomes) {
				if (!r->collision_occured) { return TestOutcome{ TestOutcomeState::FAILED, std::format("pair {} did not collide", r->id) }; }
			}
		}

		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
	};
	TestOutcome runWithoutGraphics() override {
		running_without_graphics = true;
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
	bool running_without_graphics;
	std::vector<std::shared_ptr<coltest_result>> outcomes;
};

class TestCapsuleAgainstCapsule : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Capsule"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		running_without_graphics = false; // will be set to true later if we are not using graphics
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

		outcomes = setupMultipleOrientations(p, bodies, capsule_geom, capsule_geom, {
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

		test_current_tick_count++;

		bool all_collisions_occured = true;
		for (const std::shared_ptr<coltest_result>& r : outcomes) {
			if (!r->collision_occured) { all_collisions_occured = false; }
			// return any failed collisions
			if (r->collision_occured && r->outcome.state == TestOutcomeState::FAILED) { return r->outcome; }
		}

		// if all collisions occured the test is passed based on the validations. but it's nice to run longer for visual validation.
		if (running_without_graphics && all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }

		if (test_current_tick_count >= test_total_tick_duration) {
			if (all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }

			// missing collisions
			for (const std::shared_ptr<coltest_result>& r : outcomes) {
				if (!r->collision_occured) { return TestOutcome{ TestOutcomeState::FAILED, std::format("pair {} did not collide", r->id) }; }
			}
		}

		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
	};
	TestOutcome runWithoutGraphics() override {
		running_without_graphics = true;
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
	bool running_without_graphics;
	std::vector<std::shared_ptr<coltest_result>> outcomes;
};

class TestCapsuleAgainstCylinder : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Cylinder"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		running_without_graphics = false; // will be set to true later if we are not using graphics
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

		outcomes = setupMultipleOrientations(p, bodies, cylinder_geom, capsule_geom, {
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

		test_current_tick_count++;

		bool all_collisions_occured = true;
		for (const std::shared_ptr<coltest_result>& r : outcomes) {
			if (!r->collision_occured) { all_collisions_occured = false; }
			// return any failed collisions
			if (r->collision_occured && r->outcome.state == TestOutcomeState::FAILED) { return r->outcome; }
		}

		// if all collisions occured the test is passed based on the validations. but it's nice to run longer for visual validation.
		if (running_without_graphics && all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }

		if (test_current_tick_count >= test_total_tick_duration) {
			if (all_collisions_occured) { return TestOutcome{ TestOutcomeState::PASSED }; }

			// missing collisions
			for (const std::shared_ptr<coltest_result>& r : outcomes) {
				if (!r->collision_occured) { return TestOutcome{ TestOutcomeState::FAILED, std::format("pair {} did not collide", r->id) }; }
			}
		}

		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
	};
	TestOutcome runWithoutGraphics() override {
		running_without_graphics = true;
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
	bool running_without_graphics;
	std::vector<std::shared_ptr<coltest_result>> outcomes;
};

class TestCapsuleAgainstMeshGeometry : public Test {
public:
	std::string getTestName() const override { return "Capsule vs Mesh"; }
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

		//create mesh
		int grid_count = 30;
		double grid_size = 0.5;
		phyz::MeshInput grid = phyz::generateGridMeshInput(grid_count, grid_count, grid_size, mthz::Vec3(-grid_count * grid_size / 2.0, 0, -grid_count * grid_size / 2.0));
		for (mthz::Vec3& v : grid.points) {
			v.y += 0.1 * 2 * (0.5 - frand());
		}
		
		bodies->push_back(PhysBod{ fromStaticMeshInput(grid, color{ 0.5, 0.5, 0.5 }), p->createRigidBody(grid)});

		//create capsule
		double capsule_radius = 0.75;
		double capsule_drum_height = 1.5;
		phyz::ConvexUnionGeometry capsule_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(0, -capsule_drum_height / 2.0, 0), capsule_radius, capsule_drum_height).getTranslated(mthz::Vec3(0, 5, 0));
		mthz::Quaternion sideways = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));
		mthz::Quaternion sideways_turned = mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 1.0, 0)) * mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1.0));

		capsule_r = p->createRigidBody(capsule_geom);
		bodies->push_back(PhysBod{ fromGeometry(capsule_geom), capsule_r });

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
		}

		test_current_tick_count++;

		// keeping it very simple for this one
		if (capsule_r->getCOM().y < -1) {
			return TestOutcome{ TestOutcomeState::FAILED, "" };
		}

		if (test_current_tick_count >= test_total_tick_duration) {
			return TestOutcome{ TestOutcomeState::PASSED };
		}

		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
		capsule_r = nullptr;
		p = nullptr;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	phyz::RigidBody* capsule_r;
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
		out.push_back(std::make_unique<TestCapsuleAgainstMeshGeometry>());
		return out;
	}
};