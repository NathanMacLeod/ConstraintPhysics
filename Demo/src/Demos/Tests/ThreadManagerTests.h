#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/ThreadManager.h"

class TestThreadManagerDoAll : public Test {
public:
	std::string getTestName() const override { return "ThreadManager::do_all"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const { return TestExpectationStatus::REQUIRED; }
	TestOutcome runWithGraphics() override { assert(false); return TestOutcome::FAILED; };
	TestOutcome runWithoutGraphics() override {
		phyz::ThreadManager tm;
		int n_threads = 4;
		tm.init(n_threads);

		std::atomic<uint32_t> counter = 0;
		// test that if we submit a really simple job to increment counter, that the test works as expected.
		uint32_t job_count = 100000;

		// in_vector = 1, 2, 3, ..., job_count
		std::vector<uint32_t> in_vector;
		for (uint32_t i = 0; i < job_count; i++) { in_vector.push_back(i + 1); }

		// add every element of in_vector to counter
		tm.do_all(n_threads, in_vector, [&counter](uint32_t num) { counter += num; });

		uint32_t expected_value = (job_count * (job_count + 1)) / 2;
		if (counter == expected_value) { return TestOutcome::PASSED; }
		else                           { return TestOutcome::FAILED; }
	}
};

class ThreadManagerTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "ThreadManager"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestThreadManagerDoAll>());
		out.push_back(std::make_unique<TestThreadFailTest>());
		return out;
	}
};