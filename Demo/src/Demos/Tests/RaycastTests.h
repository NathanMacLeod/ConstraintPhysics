#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../Mesh.h"

class TestCapsulRaycasts : public Test {
public:
	std::string getTestName() const override { return "Raycasts against capsule"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t n_threads, std::vector<PhysBod>* bodies) override {
		p = new phyz::PhysicsEngine();
		if (n_threads > 0) {
			p->enableMultithreading(n_threads);
		}
		p->setStep_time(1.0);

		//create capsule
		phyz::ConvexUnionGeometry capsule_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(0, -CAPSULE_DRUM_HEIGHT / 2.0, 0), CAPSULE_RADIUS, CAPSULE_DRUM_HEIGHT);
		p->createRigidBody(capsule_geom, phyz::RigidBody::FIXED);
		return p;
	}
	TestOutcome tickTestOnePhysicsStep() override {
		assert(false);
		return TestOutcome{ TestOutcomeState::FAILED };
	}
	void teardownTest() override {
		delete p;
	};
	TestOutcome runWithoutGraphics() override {
		// shoot many rays perpendicular to the xy plane within a bounding rectangle.
		const double r_y = CAPSULE_DRUM_HEIGHT / 2.0 + CAPSULE_RADIUS + BOUNDING_RECT_BUFFER;
		const double r_x = CAPSULE_RADIUS / 2.0 + CAPSULE_RADIUS + BOUNDING_RECT_BUFFER;
		const double z = -4;
		const double TOP_CAP_CENTER_Y = CAPSULE_DRUM_HEIGHT / 2.0;
		const double BOT_CAP_CENTER_Y = -CAPSULE_DRUM_HEIGHT / 2.0;

		srand(0);
		for (int i = 0; i < 10000; i++) {
			// random point bounded within (-rx, -ry); (rx, ry)
			double x = -r_x + 2 * r_x * rand() / RAND_MAX;
			double y = -r_y + 2 * r_y * rand() / RAND_MAX;

			// determine if this ray is expected to hit the capsule
			bool should_hit;
			if (y > TOP_CAP_CENTER_Y) {
				// the ray would hit the top cap, if at all
				double r2 = x * x + (y - TOP_CAP_CENTER_Y) * (y - TOP_CAP_CENTER_Y);
				should_hit = r2 <= CAPSULE_RADIUS * CAPSULE_RADIUS;
			}
			else if (y < BOT_CAP_CENTER_Y) {
				// the ray would hit the bot cap, if at all
				double r2 = x * x + (y - BOT_CAP_CENTER_Y) * (y - BOT_CAP_CENTER_Y);
				should_hit = r2 <= CAPSULE_RADIUS * CAPSULE_RADIUS;
			}
			else {
				// the ray will hit the drum, if at all
				should_hit = abs(x) <= CAPSULE_RADIUS;
			}

			phyz::RayHitInfo raycast_result = p->raycastFirstIntersection(mthz::Vec3(x, y, z), mthz::Vec3(0, 0, 1));
			if (should_hit && !raycast_result.did_hit) {
				return TestOutcome{ TestOutcomeState::FAILED, std::format("the i={} raycast should've hit, but did not hit", i) };
			}
			if (!should_hit && raycast_result.did_hit) {
				return TestOutcome{ TestOutcomeState::FAILED, std::format("the i={} raycast should've not hit, but did hit", i) };
			}
		}

		return TestOutcome{ TestOutcomeState::PASSED };
	}
private:
	phyz::PhysicsEngine* p;
	static constexpr double CAPSULE_DRUM_HEIGHT = 1.5;
	static constexpr double CAPSULE_RADIUS = 0.75;
	static constexpr double BOUNDING_RECT_BUFFER = 0.5;
};

class RaycastTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "Raycast"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestCapsulRaycasts>());
		return out;
	}
};