#pragma once
#include <string>
#include <vector>
#include <memory>

// required: run this test, and if it does not pass, we consider the entire test suite failed
// xfail: run this test, but if it fails, we do not consider the entire test suite failed
// skip: do not run this test
enum TestExpectationStatus { REQUIRED, XFAIL, SKIP };
enum TestOutcome { PASSED, XPASSED, XFAILED, FAILED, SKIPPED };

class Test {
// interface representing a single test
public:
	virtual std::string getTestName() const = 0;
	virtual bool canBeRunWithGraphics() const = 0;
	virtual TestExpectationStatus getTestExpectation() const = 0;
	virtual TestOutcome runWithGraphics() = 0;
	virtual TestOutcome runWithoutGraphics() = 0;
};

class TestGroup {
// interface representing a group of related tests
public:
	virtual std::string getGroupName() const = 0;
	virtual std::vector<std::unique_ptr<Test>> getTests() const = 0;
};