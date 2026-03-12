#pragma once
#include "Test.h"
#include <cassert>
#include "../Common.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

class TestDistanceConstraint : public Test {
public:
	std::string getTestName() const override { return "Distance Constraint"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		tick_frequency = 60.0f;
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

		float t = test_current_tick_count / tick_frequency;
		double expected_distance;

		CurrentState state = get_current_state({
			StateWithDuration{"sit_still", 1.0f},
			StateWithDuration{"expand", 1.0f},
			StateWithDuration{"sit_expanded", 1.0f},
			StateWithDuration{"shrink", 1.0f},
			StateWithDuration{"sit_shrunken", 1.0f},
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
	float tick_frequency;
	double starting_distance;
};

class TestConeConstraint : public Test {
public:
	std::string getTestName() const override { return "Cone Constraint"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		tick_frequency = 60.0f;
		total_duration_ticks = static_cast<uint32_t>(15 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		// kinematic sphere
		phyz::ConvexUnionGeometry kinem_body_geom = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(0, 0, 0), 0.5);
		kinem_body_r = p->createRigidBody(kinem_body_geom, phyz::RigidBody::KINEMATIC);
		bodies->push_back(PhysBod{ fromGeometry(kinem_body_geom), kinem_body_r });

		// pole attached under the fixed sphere
		double pole_height = 3;
		phyz::ConvexUnionGeometry swinging_pole_geom = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -pole_height, 0), 0.1, pole_height);
		swinging_pole_r = p->createRigidBody(swinging_pole_geom, phyz::RigidBody::DYNAMIC);
		bodies->push_back(PhysBod{ fromGeometry(swinging_pole_geom), swinging_pole_r });

		// add ball socket constraint
		p->addBallSocketConstraint(kinem_body_r, swinging_pole_r, mthz::Vec3(0, 0, 0));

		// add cone rotation limit constraint
		p->addConeLimitConstraint(kinem_body_r, swinging_pole_r, mthz::Vec3(0, -1, 0), MAX_ANGLE);
		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		float t = test_current_tick_count / tick_frequency;

		// generate oscillating psuedo-random motion for the kinematic sphere along the xz plane, biased towards the origin
		mthz::Vec3 kinematic_vel;
		double max_unbiased_speed = 8.0;
		std::vector<int> frequencies = { 1, -2, 3, -5, 7, -11 };
		for (int freq : frequencies) {
			double rand_direction_theta = t * freq;
			kinematic_vel += mthz::Vec3(cos(rand_direction_theta), 0, sin(rand_direction_theta)) * max_unbiased_speed / static_cast<double>(frequencies.size());
		}

		double origin_bias_strength = 0.1;
		kinematic_vel += origin_bias_strength * (mthz::Vec3(0, 0, 0) - kinem_body_r->getCOM());

		if (test_current_tick_count < total_duration_ticks) {
			kinem_body_r->setVel(kinematic_vel);
			p->timeStep();
		}

		mthz::Vec3 cyl_dir = (swinging_pole_r->getCOM() - kinem_body_r->getCOM()).normalize();
		double current_angle = acos(mthz::Vec3(0, -1, 0).dot(cyl_dir));

		if (current_angle > MAX_ANGLE + 0.1) { return TestOutcome{ TestOutcomeState::FAILED, "exceeded the cone limit angle"}; }
		if (test_current_tick_count >= total_duration_ticks) { return TestOutcome{ TestOutcomeState::PASSED }; }

		test_current_tick_count++;
		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
		p = nullptr;
		kinem_body_r = nullptr;
		swinging_pole_r = nullptr;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	phyz::RigidBody* kinem_body_r;
	phyz::RigidBody* swinging_pole_r;
	uint32_t total_duration_ticks;
	uint32_t test_current_tick_count;
	float tick_frequency;
	const double MAX_ANGLE = PI / 4.0;
};

class TestTwistConstraint : public Test {
public:
	std::string getTestName() const override { return "Twist Constraint"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		tick_frequency = 60.0f;
		test_current_tick_count = 0;
		total_duration_ticks = static_cast<uint32_t>(10 * tick_frequency);

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		// kinematic sphere
		phyz::ConvexUnionGeometry kinem_body_geom = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(0, 0, 0), 0.5);
		kinem_body_r = p->createRigidBody(kinem_body_geom, phyz::RigidBody::KINEMATIC);
		bodies->push_back(PhysBod{ fromGeometry(kinem_body_geom), kinem_body_r });

		// pole attached under the fixed sphere
		double pole_height = 3;
		phyz::ConvexUnionGeometry swinging_pole_geom = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -pole_height, 0), 0.1, pole_height);
		phyz::RigidBody* swinging_pole_r = p->createRigidBody(swinging_pole_geom, phyz::RigidBody::DYNAMIC);
		bodies->push_back(PhysBod{ fromGeometry(swinging_pole_geom), swinging_pole_r });

		// add ball socket constraint
		p->addBallSocketConstraint(kinem_body_r, swinging_pole_r, mthz::Vec3(0, 0, 0));

		// add cone rotation limit constraint
		twist_min_limit = -PI;
		twist_max_limit = 3 * PI;
		twist_constraint = p->addTwistLimitConstraint(kinem_body_r, swinging_pole_r, mthz::Vec3(0, 1, 0), twist_min_limit, twist_max_limit);
		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		float t = test_current_tick_count / tick_frequency;

		mthz::Vec3 kinem_ang_vel;

		CurrentState state = get_current_state({
			StateWithDuration{"spin_clockwise", 4.0f},
			StateWithDuration{"spin_counterclockwise", 4.0f},
		}, t);

		// switch between moving, fixing, and shrinking the distance depending on the time interval
		if (state.current.name == "spin_clockwise") {
			kinem_ang_vel = mthz::Vec3(0, -3, 0);
		}
		else if (state.current.name == "spin_counterclockwise") {
			kinem_ang_vel = mthz::Vec3(0, 3, 0);
		}

		double f = 1.0;
		double theta = f * t;
		mthz::Vec3 kinematic_vel = mthz::Vec3(cos(theta), 0, sin(theta));

		if (test_current_tick_count <= total_duration_ticks) {
			kinem_body_r->setVel(kinematic_vel);
			kinem_body_r->setAngVel(kinem_ang_vel);
			p->timeStep();
		}

		double tol = 0.1;
		double current_angle = p->getTwistConstraintCurrentAngle(twist_constraint);
		if (current_angle < twist_min_limit - tol) { return TestOutcome{ TestOutcomeState::FAILED, "Twisted below the min limit"}; }
		if (current_angle > twist_max_limit + tol) { return TestOutcome{ TestOutcomeState::FAILED, "Twisted above the max limit"}; }
		
		if (test_current_tick_count >= total_duration_ticks) {
			return TestOutcome{ TestOutcomeState::PASSED };
		}

		test_current_tick_count++;
		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
		p = nullptr;
		kinem_body_r = nullptr;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	phyz::ConstraintID twist_constraint;
	phyz::RigidBody* kinem_body_r;
	double twist_min_limit, twist_max_limit;
	uint32_t test_current_tick_count;
	float tick_frequency;
	uint32_t total_duration_ticks;
};

class TwistConstraintStressTest : public Test {
public:
	std::string getTestName() const override { return "Twist Constraint Stress Test"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		tick_frequency = 60.0f;
		test_duration_ticks = static_cast<uint32_t>(15 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		mthz::Vec3 rope_left_start_position = mthz::Vec3(0, 0, 0);
		
		double wheel_attach_block_size = 0.5;
		double wheel_thickness = 0.1;
		
		phyz::ConvexUnionGeometry wheel_attach_block_geom = phyz::ConvexUnionGeometry::box(rope_left_start_position + mthz::Vec3(-wheel_attach_block_size - wheel_thickness, -wheel_attach_block_size / 2.0, -wheel_attach_block_size / 2.0), wheel_attach_block_size, wheel_attach_block_size, wheel_attach_block_size);
		phyz::RigidBody* wheel_attack_block_r = p->createRigidBody(wheel_attach_block_geom, phyz::RigidBody::FIXED);
		bodies->push_back({ PhysBod(fromGeometry(wheel_attach_block_geom), wheel_attack_block_r) });
		rbodies.push_back(wheel_attack_block_r);

		phyz::ConvexUnionGeometry wheel_geom = phyz::ConvexUnionGeometry::cylinder(rope_left_start_position, wheel_attach_block_size / 2.0, wheel_thickness).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), rope_left_start_position);
		phyz::RigidBody* wheel_r = p->createRigidBody(wheel_geom);
		bodies->push_back({ PhysBod(fromGeometry(wheel_geom), wheel_r) });
		rbodies.push_back(wheel_r);
		
		twist_motor = p->addMotorConstraint(
			p->addHingeConstraint(wheel_attack_block_r, wheel_r, rope_left_start_position + mthz::Vec3(-wheel_thickness, 0, 0), mthz::Vec3(1, 0, 0))
		);

		mthz::Vec3 rope_segment_dimensions = { 0.1, 0.04, 0.04 };
		const int SEGMENT_COUNT = 100;
		const double TWIST_LIMIT = PI / 12.0;

		phyz::RigidBody* left_body = wheel_r;
		for (int i = 0; i < SEGMENT_COUNT; i++) {
			mthz::Vec3 segment_attach_position = rope_left_start_position + mthz::Vec3(rope_segment_dimensions.x * i, 0, 0);
			phyz::ConvexUnionGeometry segment_geom = phyz::ConvexUnionGeometry::box(segment_attach_position + mthz::Vec3(0, -rope_segment_dimensions.y / 2.0, -rope_segment_dimensions.z / 2.0), rope_segment_dimensions.x, rope_segment_dimensions.y, rope_segment_dimensions.z);
			phyz::RigidBody* segment_r = p->createRigidBody(segment_geom);
			bodies->push_back({ PhysBod(fromGeometry(segment_geom), segment_r) });
			rbodies.push_back(segment_r);

			p->addBallSocketConstraint(left_body, segment_r, segment_attach_position);
			p->addTwistLimitConstraint(left_body, segment_r, mthz::Vec3(1, 0, 0), -TWIST_LIMIT, TWIST_LIMIT);
			left_body = segment_r;
		}

		mthz::Vec3 final_attach_pos = rope_left_start_position + mthz::Vec3(rope_segment_dimensions.x * SEGMENT_COUNT, 0, 0);
		phyz::ConvexUnionGeometry right_anchor_block_geom = phyz::ConvexUnionGeometry::box(final_attach_pos + mthz::Vec3(0, -wheel_attach_block_size / 2.0, -wheel_attach_block_size / 2.0), wheel_attach_block_size, wheel_attach_block_size, wheel_attach_block_size);
		phyz::RigidBody* right_anchor_block_r = p->createRigidBody(right_anchor_block_geom, phyz::RigidBody::KINEMATIC);
		bodies->push_back({ PhysBod(fromGeometry(right_anchor_block_geom), right_anchor_block_r) });
		rbodies.push_back(right_anchor_block_r);

		p->addBallSocketConstraint(left_body, right_anchor_block_r, final_attach_pos);
		p->addTwistLimitConstraint(left_body, right_anchor_block_r, mthz::Vec3(1, 0, 0), -TWIST_LIMIT, TWIST_LIMIT);

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		float t = test_current_tick_count / tick_frequency;

		if (test_current_tick_count < test_duration_ticks) {
			p->setMotorTargetVelocity(twist_motor, 10.0, 10.0);
			p->timeStep();
		}

		// check that nothing exploded
		for (phyz::RigidBody* r : rbodies) {
			if (abs(r->getCOM().y) > 10) TestOutcome{ TestOutcomeState::FAILED, "something exploded"};
		}

		if (test_current_tick_count >= test_duration_ticks) {
			return TestOutcome{ TestOutcomeState::PASSED };
		}
		
		test_current_tick_count++;
		return TestOutcome{ TestOutcomeState::STILL_RUNNING };
	}
	void teardownTest() override {
		delete p;
		p = nullptr;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()).state == TestOutcomeState::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	phyz::ConstraintID twist_motor;
	std::vector<phyz::RigidBody*> rbodies;
	uint32_t test_current_tick_count;
	uint32_t test_duration_ticks;
	float tick_frequency;
};

class ConstraintTestsGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "Constraints"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestDistanceConstraint>());
		out.push_back(std::make_unique<TestConeConstraint>());
		out.push_back(std::make_unique<TestTwistConstraint>());
		out.push_back(std::make_unique<TwistConstraintStressTest>());
		return out;
	}
};