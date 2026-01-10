#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

class TestHeavyWeightOnChain : public Test {
public:
	std::string getTestName() const override { return "HolonomicBlockSolver::High mass Ratio Chain"; }
	bool canBeRunWithGraphics() const override { return true; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	void overrideCameraInitialPosition(mthz::Vec3* cam_pos, mthz::Quaternion* cam_orient) const override { cam_pos->y = -40; cam_pos->z = 100; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		double tick_frequency = 60.0;
		test_total_tick_duration = static_cast<uint32_t>(500 * tick_frequency);
		test_current_tick_count = 0;

		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0 / tick_frequency);

		mthz::Vec3 chain_pos(0, 0, 0);
		double chain_width = 0.31;
		double chain_height = 1;

		double attach_box_size = 1;
		phyz::ConvexUnionGeometry attach_box = phyz::ConvexUnionGeometry::box(chain_pos - mthz::Vec3(1, 1, 1) * attach_box_size / 2, attach_box_size, attach_box_size, attach_box_size);
		phyz::ConvexUnionGeometry chain = phyz::ConvexUnionGeometry::box(mthz::Vec3(-chain_width / 2.0, 0, -chain_width / 2.0), chain_width, -chain_height, chain_width, phyz::Material::modified_density(2));

		phyz::RigidBody* attach_box_r = p->createRigidBody(attach_box, phyz::RigidBody::FIXED);
		int n_chain = 2;
		phyz::RigidBody* previous_chain = nullptr;

		for (int i = 0; i < n_chain; i++) {
			mthz::Vec3 this_chain_pos = chain_pos - mthz::Vec3(0, i * chain_height, 0);
			phyz::ConvexUnionGeometry this_chain = chain.getTranslated(this_chain_pos);

			phyz::RigidBody* this_chain_r = p->createRigidBody(this_chain);

			if (i == 0) {
				p->addBallSocketConstraint(attach_box_r, this_chain_r, this_chain_pos);

			}
			else {
				p->addBallSocketConstraint(previous_chain, this_chain_r, this_chain_pos);
			}

			bodies->push_back({ fromGeometry(this_chain), this_chain_r });
			previous_chain = this_chain_r;
		}

		mthz::Vec3 final_chain_pos = chain_pos + mthz::Vec3(0, -n_chain * chain_height, 0);
		double ball_radius = 7;
		mthz::Vec3 ball_pos = final_chain_pos + mthz::Vec3(0, -ball_radius, 0);
		phyz::ConvexUnionGeometry ball = phyz::ConvexUnionGeometry::sphere(ball_pos, ball_radius, phyz::Material::modified_density(2));
		phyz::RigidBody* ball_r = p->createRigidBody(ball);

		bodies->push_back({ fromGeometry(attach_box), attach_box_r });

		ball_r->setVel(mthz::Vec3(7, 0, 0));

		p->addBallSocketConstraint(ball_r, previous_chain, final_chain_pos);
		bodies->push_back({ fromGeometry(ball), ball_r });

		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		if (test_current_tick_count < test_total_tick_duration) {
			p->timeStep();
		}

		test_current_tick_count++;

		if (test_current_tick_count >= test_total_tick_duration) {
			return TestOutcome::FAILED;
		}
		else { return TestOutcome::STILL_RUNNING; }
	}
	void teardownTest() override {
		delete p;
		p = nullptr;
	};
	TestOutcome runWithoutGraphics() override {
		TestOutcome outcome;
		while ((outcome = tickTestOnePhysicsStep()) == TestOutcome::STILL_RUNNING);
		return outcome;
	}
private:
	phyz::PhysicsEngine* p;
	uint32_t test_total_tick_duration;
	uint32_t test_current_tick_count;
};

class HolonomicBlockSolverTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "ThreadManager"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestHeavyWeightOnChain>());
		return out;
	}
};