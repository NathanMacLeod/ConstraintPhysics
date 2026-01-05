#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include <chrono>
#include <map>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

#include "Tests/Test.h"
#include "Tests/ThreadManagerTests.h"

class UnitTestsRunner : public DemoScene {
public:
	UnitTestsRunner(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~UnitTestsRunner() override {}

	std::map<std::string, std::string> askParameters() override {
		std::map<std::string, std::string> out;

		/*out["graphics_enabled"] = pickParameterFromOptions(
			"Choose whether to run with or without graphics enabled (y/n): ", { "y", "n" }
		);*/

		/*out["which_scene"] = pickParameterFromOptions(
			"Select the suite of tests to run.\n\
1) All - run all tests\n\
Select which scene to run: ", { "1" }
);*/

		return out;
	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"I. K", "Raise / Lower scissor lift, if present"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}


	void run() override {
		bool rendering_enabled = false;// parameters["graphics_enabled"] == "y";

		if (rendering_enabled) {
			rndr::init(properties.window_width, properties.window_height, "Performance Demos");
		}

		std::vector<std::unique_ptr<TestGroup>> test_groups;
		test_groups.push_back(std::make_unique<ThreadManagerTestGroup>());

		std::vector<std::unique_ptr<Test>> tests;
		for (const std::unique_ptr<TestGroup>& gr : test_groups) {
			for (std::unique_ptr<Test>& t : gr->getTests()) {
				tests.push_back(std::move(t));
			}
		}
		
		printf("Collected %d tests...\n", static_cast<uint32_t>(tests.size()));

		std::vector<std::string> passed_tests;
		std::vector<std::string> failed_tests;
		std::vector<std::string> xpassed_tests;
		std::vector<std::string> xfailed_tests;
		std::vector<std::string> skipped_tests;
		

		if (!rendering_enabled) {
			for (std::unique_ptr<Test>& t : tests) {
				std::string test_name = t->getTestName();
				if (t->getTestExpectation() == TestExpectationStatus::SKIP) {
					printf("(%s): SKIPPED\n", test_name.c_str());
					skipped_tests.push_back(t->getTestName());
					continue;
				}

				auto t1 = std::chrono::system_clock::now();
				TestOutcome outcome = t->runWithoutGraphics();
				auto t2 = std::chrono::system_clock::now();
				float duration = std::chrono::duration<float, std::milli>(t2 - t1).count();

				const char* START_GREEN_TEXT = "\x1B[32m";
				const char* START_RED_TEXT = "\x1B[31m";
				const char* START_YELLOW_TEXT = "\x1B[33m";
				const char* END_TEXT_COLORING = "\033[0m";

				if (outcome == PASSED && t->getTestExpectation() == TestExpectationStatus::XFAIL) {
					printf("(%s): %sXPASSED%s | duration %fms\n", test_name.c_str(), START_GREEN_TEXT, END_TEXT_COLORING, duration);
					xpassed_tests.push_back(test_name);
				}
				else if (outcome == PASSED) {
					printf("(%s): %sPASSED%s | duration %fms\n", test_name.c_str(), START_GREEN_TEXT, END_TEXT_COLORING, duration);
					passed_tests.push_back(test_name);
				}
				else if (outcome == FAILED && t->getTestExpectation() == TestExpectationStatus::XFAIL) {
					printf("(%s): %sXFAILED%s | duration %fms\n", test_name.c_str(), START_YELLOW_TEXT, END_TEXT_COLORING, duration);
					xfailed_tests.push_back(test_name);
				}
				else if (outcome == FAILED) {
					printf("(%s): %sFAILED%s | duration %fms\n", test_name.c_str(), START_RED_TEXT, END_TEXT_COLORING, duration);
					failed_tests.push_back(test_name);
				}
			}

			printf("\n=~=~=~=~=~=~=Summary=~=~=~=~=~=~=\n");
			printf("Passed: %d\nFailed: %d\nXPassed: %d\nXFailed: %d\nSkipped: %d\n",
				   static_cast<uint32_t>(passed_tests.size()), static_cast<uint32_t>(failed_tests.size()), static_cast<uint32_t>(xpassed_tests.size()),
				   static_cast<uint32_t>(xfailed_tests.size()), static_cast<uint32_t>(skipped_tests.size()));

			if (failed_tests.size() > 0) {
				printf("\n----Failed-Tests----\n");
				for (const std::string& test_name : failed_tests) { printf("%s\n", test_name.c_str()); }
			}

			printf("\nPress enter to exit.\n");
			fgetc(stdin);
			manager->deselectCurrentScene();
		}
		else {
			//nothing yet
		}
	}
};