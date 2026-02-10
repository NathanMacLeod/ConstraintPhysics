#pragma once
#include <string>
#include <vector>
#include <memory>
#include <functional>

enum TestExpectationStatus { REQUIRED, XFAIL, SKIP };
enum TestOutcomeState { 
	PASSED,        // test has finished and passed
	XPASSED,       // test has passed, but we expected it to fail
	XFAILED,       // test has failed, but this is expected
	FAILED,        // test has finished and failed
	SKIPPED,       // test was never run
	STILL_RUNNING, // test is still running. only used when running interactively
	RESET,         // user has requested to re-run the test. only used for running interactively
};
struct TestOutcome {
	TestOutcomeState state;
	std::string reason; // optional explanation of why the outcome occured. most commonly an explanation of why the test failed.
};

class Test {
// interface representing a single test
public:
	virtual std::string getTestName() const = 0;
	virtual bool canBeRunWithGraphics() const = 0;
	virtual void overrideCameraInitialPosition(mthz::Vec3* cam_pos, mthz::Quaternion* cam_orient) const {};
	virtual TestExpectationStatus getTestExpectation() const = 0;
	virtual phyz::PhysicsEngine* initTest(uint32_t thread_count, std::vector<PhysBod>* bodies) = 0; // if creating a physics engine instance is not applicable, this will return nullptr
	virtual TestOutcome tickTestOnePhysicsStep() = 0; // for use when running the test with graphics.
	virtual void teardownTest() = 0;
	virtual TestOutcome runWithoutGraphics() = 0;
};

class TestGroup {
// interface representing a group of related tests
public:
	virtual std::string getGroupName() const = 0;
	virtual std::vector<std::unique_ptr<Test>> getTests() const = 0;
};