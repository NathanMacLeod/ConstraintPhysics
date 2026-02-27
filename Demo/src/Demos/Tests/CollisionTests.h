#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

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

class CollisionTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "Collision"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestBasicBoxAgainstGround>());
		return out;
	}
};