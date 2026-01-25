#pragma once
#include "Test.h"
#include <cassert>
#include "../../../ConstraintPhysics/src/ThreadManager.h"

class TestThreadManagerDoAll : public Test {
public:
	std::string getTestName() const override { return "ThreadManager::do_all"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t thread_count, std::vector<PhysBod>* bodies) override { return nullptr; }
	TestOutcome tickTestOnePhysicsStep() override { assert(false); return TestOutcome::FAILED; }
	void teardownTest() override {};
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
		phyz::ThreadManager::JobStatus status;
		tm.submit_do_all(n_threads, &in_vector, [&counter](uint32_t num) { counter += num; }, &status);
		status.waitUntilDone();

		uint32_t expected_value = (job_count * (job_count + 1)) / 2;
		if (counter != expected_value) { return TestOutcome::FAILED; }

		// do it one more time using await_do_all
		counter = 0;
		tm.await_do_all(n_threads, &in_vector, [&counter](uint32_t num) { counter += num; });

		if (counter != expected_value) { return TestOutcome::FAILED; }
		return TestOutcome::PASSED;
	}
};

class TestThreadManagerSubmit : public Test {
public:
	std::string getTestName() const override { return "ThreadManager::submit"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t thread_count, std::vector<PhysBod>* bodies) override { return nullptr; }
	TestOutcome tickTestOnePhysicsStep() override { assert(false); return TestOutcome::FAILED; }
	void teardownTest() override {};
	TestOutcome runWithoutGraphics() override {
		phyz::ThreadManager tm;
		int n_threads = 4;
		tm.init(n_threads);

		std::atomic_bool flag = false;
		
		phyz::ThreadManager::JobStatus status;
		tm.submit([&flag]() { flag = true; }, &status);
		status.waitUntilDone();

		if (flag) { return TestOutcome::PASSED; }
		else { return TestOutcome::FAILED; }
	}
};

class TestThreadManagerJobStatusWorksAsExpected : public Test {
public:
	std::string getTestName() const override { return "ThreadManager::jobstatus dependencies"; }
	bool canBeRunWithGraphics() const override { return false; }
	TestExpectationStatus getTestExpectation() const override { return TestExpectationStatus::REQUIRED; }
	phyz::PhysicsEngine* initTest(uint32_t thread_count, std::vector<PhysBod>* bodies) override { return nullptr; }
	TestOutcome tickTestOnePhysicsStep() override { assert(false); return TestOutcome::FAILED; }
	void teardownTest() override {};
	TestOutcome runWithoutGraphics() override {
		phyz::ThreadManager tm;
		int n_threads = 4;
		tm.init(n_threads);

		// create jobs that will wait on other jobs. confirm everything works as expected.

		// Job1 needs Job2 to finish before it exits
		// tasks in Job2 needs Job3 to finish before they exits.
		// Job3 waits on a signal from the main thread to complete.

		phyz::ThreadManager::JobStatus job1_status;
		phyz::ThreadManager::JobStatus job2_status;
		phyz::ThreadManager::JobStatus job3_status;

		std::atomic_bool job1_started_flag = false;
		std::atomic_uint32_t job2_started_counter = false;
		std::atomic_bool job3_started_flag = false;

		std::atomic_bool job1_finished_flag = false;
		std::atomic_uint32_t job2_finished_counter = false;
		std::atomic_bool job3_finished_flag = false;

		std::mutex job3_acquired_mutex;
		job3_acquired_mutex.lock();

		//job1
		tm.submit([&job1_started_flag, &job1_finished_flag, &job2_status]() {
			job1_started_flag = true;
			job2_status.waitUntilDone();
			job1_finished_flag = true;
		}, &job1_status);

		//job2
		uint32_t job2_task_count = 10;
		uint32_t job2_thread_count = 2;
		std::vector<uint32_t> v(job2_task_count);
		tm.submit_do_all(job2_thread_count, &v, [&job2_started_counter, &job2_finished_counter, &job3_status](uint32_t) {
			job2_started_counter++;
			job3_status.waitUntilDone();
			job2_finished_counter++;
		}, &job2_status);

		//job3
		tm.submit([&job3_started_flag, &job3_finished_flag, &job3_acquired_mutex]() {
			job3_started_flag = true;
			job3_acquired_mutex.lock();
			job3_finished_flag = true;
		}, &job3_status);

		//wait until all tasks have started
		uint32_t max_wait = 1000;
		uint32_t total_waited = 0;
		uint32_t sleep_interval = 10;
		while (total_waited < max_wait && (!job1_started_flag || !job3_started_flag || job2_started_counter < job2_thread_count)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_interval));
			total_waited += sleep_interval;
		}

		// this is the state everything should get stuck at.
		if (!job1_started_flag) { return TestOutcome::FAILED; }
		if (!job3_started_flag) { return TestOutcome::FAILED; }
		if (job2_started_counter != job2_thread_count) { return TestOutcome::FAILED; }

		job3_acquired_mutex.unlock();

		// all jobs should be able to finish now
		job3_status.waitUntilDone();
		job2_status.waitUntilDone();
		job1_status.waitUntilDone();

		if (!job1_finished_flag) { return TestOutcome::FAILED; }
		if (!job3_finished_flag) { return TestOutcome::FAILED; }
		if (job2_finished_counter != job2_task_count) { return TestOutcome::FAILED; }

		return TestOutcome::PASSED;
	}
};

class ThreadManagerTestGroup : public TestGroup {
public:
	std::string getGroupName() const override { return "ThreadManager"; }
	std::vector<std::unique_ptr<Test>> getTests() const override {
		std::vector<std::unique_ptr<Test>> out;
		out.push_back(std::make_unique<TestThreadManagerDoAll>());
		out.push_back(std::make_unique<TestThreadManagerSubmit>());
		out.push_back(std::make_unique<TestThreadManagerJobStatusWorksAsExpected>());
		return out;
	}
};