#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

// assert that r1 and r2 have matching mass properties, within some error tolerance
TestOutcome compareMassProperties(phyz::RigidBody* r1, phyz::RigidBody* r2);
#ifndef CMP_IMPL
#define CMP_IMPL

static double ERROR_TOL = 0.5;

TestOutcome compareMassProperties(phyz::RigidBody* r1, phyz::RigidBody* r2) {
	if (abs(r1->getMass() - r2->getMass()) > ERROR_TOL) {
		return TestOutcome{ TestOutcomeState::FAILED, "r1, r2 masses differed larger than the tolerance"};
	}

	if ((r1->getCOM() - r2->getCOM()).mag() > ERROR_TOL) {
		return TestOutcome{ TestOutcomeState::FAILED, "r1, r2 centers of mass differed larger than the tolerance"};
	}

	mthz::Mat3 r1_tens = r1->getTensor();
	mthz::Mat3 r2_tens = r2->getTensor();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (abs(r1_tens.v[i][j] - r2_tens.v[i][j]) > ERROR_TOL) {
				return TestOutcome{ TestOutcomeState::FAILED, "r1, r2 inertia tensors differed larger than the tolerance" };
			}
		}
	}

	return TestOutcome{ TestOutcomeState::PASSED };
}

#endif

class CrossReferencePolyAgainstSphere : public Test {
public:
	std::string getTestName() const override { return "cross ref polygon vs sphere mass properties"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t thread_count, std::vector<PhysBod>* bodies) override { return nullptr; }
	TestOutcome tickTestOnePhysicsStep() override { assert(false); return TestOutcome{ TestOutcomeState::FAILED }; }
	void teardownTest() override {};
	TestOutcome runWithoutGraphics() override {
		phyz::PhysicsEngine p;

		phyz::ConvexUnionGeometry sphere_geom = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 2.0);
		phyz::RigidBody* sphere = p.createRigidBody(sphere_geom);

		phyz::ConvexUnionGeometry poly_sphere_geom = phyz::ConvexUnionGeometry::psuedoSphere(mthz::Vec3(), 2.0, 60, 60);
		phyz::RigidBody* poly_sphere = p.createRigidBody(poly_sphere_geom);

		return compareMassProperties(sphere, poly_sphere);
	}
};

class CrossReferencePolyAgainstCylinder : public Test {
public:
	std::string getTestName() const override { return "cross ref polygon vs cylinder mass properties"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t thread_count, std::vector<PhysBod>* bodies) override { return nullptr; }
	TestOutcome tickTestOnePhysicsStep() override { assert(false); return TestOutcome{ TestOutcomeState::FAILED }; }
	void teardownTest() override {};
	TestOutcome runWithoutGraphics() override {
		phyz::PhysicsEngine p;

		phyz::ConvexUnionGeometry cylinder_geom = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), 2.0, 2.0);
		phyz::RigidBody* cylinder = p.createRigidBody(cylinder_geom);

		phyz::ConvexUnionGeometry poly_cylinder_geom = phyz::ConvexUnionGeometry::polyCylinder(mthz::Vec3(), 2.0, 2.0, 60);
		phyz::RigidBody* poly_cylinder = p.createRigidBody(poly_cylinder_geom);

		return compareMassProperties(cylinder, poly_cylinder);
	}
};

class CrossReferencePolyAgainstCapsule : public Test {
public:
	std::string getTestName() const override { return "cross ref polygon vs capsule mass properties"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t thread_count, std::vector<PhysBod>* bodies) override { return nullptr; }
	TestOutcome tickTestOnePhysicsStep() override { assert(false); return TestOutcome{ TestOutcomeState::FAILED }; }
	void teardownTest() override {};
	TestOutcome runWithoutGraphics() override {
		phyz::PhysicsEngine p;

		double drum_height = 0.75;
		phyz::ConvexUnionGeometry capsule_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(0, -drum_height/2.0, 0), 0.5, drum_height);
		phyz::RigidBody* capsule = p.createRigidBody(capsule_geom);

		phyz::ConvexUnionGeometry poly_capsule_geom = phyz::ConvexUnionGeometry::polyCapsule(mthz::Vec3(), 0.5, drum_height, 45);
		phyz::RigidBody* poly_capsule = p.createRigidBody(poly_capsule_geom);

		return compareMassProperties(capsule, poly_capsule);
	}
};

class MassPropertiesTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "MassProperties"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<CrossReferencePolyAgainstSphere>());
		out.push_back(std::make_unique<CrossReferencePolyAgainstCylinder>());
		out.push_back(std::make_unique<CrossReferencePolyAgainstCapsule>());
		return out;
	}
};