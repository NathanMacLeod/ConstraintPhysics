#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"
#include "../Common.h"

class TestRagdollsOnStaticMesh : public Test {
public:
	std::string getTestName() const override { return "Ragdolls On Static Mesh"; }
	bool canBeRunWithGraphics() const override { return true; }
	void getCameraInitialPosition(mthz::Vec3* cam_pos, mthz::Quaternion* cam_orient) const override { *cam_pos = mthz::Vec3(0, 7, 15); *cam_orient = mthz::Quaternion(); }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		tick_frequency = 60.0f;
		test_total_tick_duration = static_cast<uint32_t>(500 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);
		p->setPGSIterations(4, 1, 1);
		p->setSubstepCount(8);
		p->setGlobalConstraintForceMixing(0.00000001);

		// create boat
		phyz::Mesh bunny_mesh = phyz::readOBJ("resources/mesh/bunny.obj", 50.0);
		phyz::MeshInput bunny_mesh_input = phyz::generateMeshInputFromMesh(bunny_mesh, mthz::Vec3(0, 0, 0));
		boat_mesh_r = p->createRigidBody(bunny_mesh_input);
		bodies->push_back({ fromStaticMeshInput(bunny_mesh_input, color{ 0.8f, 1.0f, 1.0f, 0.5f, 0.5f, 0.63f, 51.2f }), boat_mesh_r });

		std::vector<mthz::Vec3> body_create_positions = {
			mthz::Vec3(-4, 15, -1), mthz::Vec3(-2, 15, 1), mthz::Vec3(0, 15, -1),
			mthz::Vec3(2, 15, 1), mthz::Vec3(4, 15, -1),
		};

		for (mthz::Vec3 pos : body_create_positions) {
			createRagdoll(p, bodies, pos, 0.7);
		}

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		test_current_tick_count++;

		float t = test_current_tick_count / tick_frequency;

		if (true) {
			p->timeStep();
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
	float tick_frequency;
	phyz::PhysicsEngine* p;
	phyz::RigidBody* boat_mesh_r;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
};

class TestRagdollsOnKinematicMesh : public Test {
public:
	std::string getTestName() const override { return "Ragdolls On Kinematic Mesh"; }
	bool canBeRunWithGraphics() const override { return true; }
	void getCameraInitialPosition(mthz::Vec3* cam_pos, mthz::Quaternion* cam_orient) const override { *cam_pos = mthz::Vec3(0, 7, 15); *cam_orient = mthz::Quaternion(); }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		tick_frequency = 60.0f;
		test_total_tick_duration = static_cast<uint32_t>(500 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);
		p->setPGSIterations(4, 1, 1);
		p->setSubstepCount(8);
		p->setGlobalConstraintForceMixing(0.00000001);

		// create boat
		phyz::Mesh boat_mesh = phyz::readOBJ("resources/mesh/benchy.obj", 0.2);
		phyz::MeshInput boat_mesh_input = phyz::generateMeshInputFromMesh(boat_mesh, mthz::Vec3(0, 0, 0));
		boat_mesh_r = p->createRigidBody(boat_mesh_input, false);
		bodies->push_back({ fromStaticMeshInput(boat_mesh_input, color{ 0.8f, 1.0f, 1.0f, 0.5f, 0.5f, 0.63f, 51.2f }), boat_mesh_r });

		std::vector<mthz::Vec3> body_create_positions = {
			mthz::Vec3(-4, 15, -1), mthz::Vec3(-2, 15, 1), mthz::Vec3(0, 15, -1),
			mthz::Vec3(2, 15, 1), mthz::Vec3(4, 15, -1),
		};

		for (mthz::Vec3 pos : body_create_positions) {
			createRagdoll(p, bodies, pos, 0.7);
		}

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		test_current_tick_count++;

		float t = test_current_tick_count / tick_frequency;

		// animating motionm of the boat with sin waves
		const double UP_DOWN_PERIOD = 5;
		const double UP_DOWN_AMPLITUDE = 1;
		
		const double TILT_UP_DOWN_PERIOD = 5;
		const double TILT_UP_DOWN_AMPLITUDE = PI / 48;
		const double TILD_UP_DOWN_OFFSET = TILT_UP_DOWN_PERIOD * 0.75;

		mthz::Vec3 vel = UP_DOWN_AMPLITUDE * mthz::Vec3(0,  cos(t / UP_DOWN_PERIOD), 0);
		mthz::Vec3 ang_vel = TILT_UP_DOWN_AMPLITUDE * mthz::Vec3(0, 0, cos((TILD_UP_DOWN_OFFSET + t) / TILT_UP_DOWN_PERIOD));

		if (true) {
			boat_mesh_r->setVel(vel);
			boat_mesh_r->setAngVel(ang_vel);
			p->timeStep();
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
	float tick_frequency;
	phyz::PhysicsEngine* p;
	phyz::RigidBody* boat_mesh_r;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
};

class RagdollTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "RagdollTests"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestRagdollsOnKinematicMesh>());
		out.push_back(std::make_unique<TestRagdollsOnStaticMesh>());
		return out;
	}
};