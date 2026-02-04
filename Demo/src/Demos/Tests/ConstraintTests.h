#pragma once
#include "Test.h"
#include <cassert>
#include "../Common.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

class TestDistanceConstraint : public Test {
public:
	std::string getTestName() const override { return "distance constraint"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		tick_frequency = 60.0;
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


		//create boxes
		mthz::Vec3 boxes_pos = mthz::Vec3(0.0, 0.0, 0.0);
		starting_distance = 0.25;
		box_size = 1.0;
		phyz::ConvexUnionGeometry box1_geom = phyz::ConvexUnionGeometry::box(boxes_pos, box_size, box_size, box_size);
		box1_r = p->createRigidBody(box1_geom);
		bodies->push_back(PhysBod{ fromGeometry(box1_geom), box1_r });
		phyz::ConvexUnionGeometry box2_geom = phyz::ConvexUnionGeometry::box(boxes_pos + mthz::Vec3(box_size + starting_distance, 0, 0), box_size, box_size, box_size);
		box2_r = p->createRigidBody(box2_geom);
		bodies->push_back(PhysBod{ fromGeometry(box2_geom), box2_r });

		//add distance constraint
		dist_constraint = p->addDistanceConstraint(box1_r, box2_r, boxes_pos + mthz::Vec3(box_size, box_size / 2.0, box_size / 2.0), boxes_pos + mthz::Vec3(box_size + starting_distance, box_size / 2.0, box_size / 2.0));
		p->reallowCollision(box1_r, box2_r);

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		test_current_tick_count++;

		double t = test_current_tick_count / tick_frequency;
		double expected_distance;

		CurrentState state = get_current_state({
			StateWithDuration{"sit_still", 1.0},
			StateWithDuration{"expand", 1.0},
			StateWithDuration{"sit_expanded", 1.0},
			StateWithDuration{"shrink", 1.0},
			StateWithDuration{"sit_shrunken", 1.0},
		}, t);

		// switch between moving, fixing, and shrinking the distance depending on the time interval
		if (state.current.name == "sit_still") {
			expected_distance = starting_distance;
		}
		else if (state.current.name == "expand") {
			p->setDistanceConstraintToMoveAtTargetVelocity(dist_constraint, 5.0);
			expected_distance = starting_distance + 5.0 * state.time_into_current_state;
		}
		else if (state.current.name == "sit_expanded") {
			p->setDistanceConstraintTargetDistance(dist_constraint, starting_distance + 5.0);
			expected_distance = starting_distance + 5.0;
		}
		else if (state.current.name == "shrink") {
			p->setDistanceConstraintToMoveAtTargetVelocity(dist_constraint, -5.0);
			expected_distance = starting_distance + 5.0 - 5.0 * state.time_into_current_state;
		}
		else {
			p->setDistanceConstraintTargetDistance(dist_constraint, starting_distance);
			expected_distance = starting_distance;
		}

		if (!state.final_state_has_elapsed) {
			p->timeStep();
		}

		double allowed_error = 0.1;

		if (state.final_state_has_elapsed) { return TestOutcome{ TestOutcomeState::PASSED }; }
		else if (abs(expected_distance - p->getDistanceConstraintCurrentDistance(dist_constraint)) > allowed_error) {
			return TestOutcome{ TestOutcomeState::FAILED, "Distance constraint did not maintain the expected distances"};
		}
		else {
			return TestOutcome{ TestOutcomeState::STILL_RUNNING };
		}
	}
	void teardownTest() override {
		delete p;
		p = nullptr;
		box1_r = nullptr;
		box2_r = nullptr;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	phyz::RigidBody* box1_r;
	phyz::RigidBody* box2_r;
	phyz::ConstraintID dist_constraint;
	uint32_t test_current_tick_count;
	double box_size;
	bool collision_occured;
	bool no_incorrect_manifolds;
	double tick_frequency;
	double starting_distance;
};

class ConstraintTestsGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "Constraints"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestDistanceConstraint>());
		return out;
	}
};