#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include <chrono>
#include <map>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

#include "Tests/Test.h"
#include "Tests/ThreadManagerTests.h"
#include "Tests/CollisionTests.h"
#include "Tests/HolonomicBlockSolvingTests.h"

class UnitTestsRunner : public DemoScene {
private:
	bool paused;
	std::vector<PhysBod> active_models;
	
	TestOutcome runTestWithGraphics(phyz::PhysicsEngine* test_pengine, std::unique_ptr<Test>& test) {
		TestOutcome outcome = TestOutcome::STILL_RUNNING;

		assert(test_pengine != nullptr); // cant think of a need for a non-physics based right now.
		// basic rendering stuff
		mthz::Vec3 pos(0, 2, 10);
		mthz::Quaternion orient;
		// optional, test can choose if it wants a different camera
		test->overrideCameraInitialPosition(&pos, &orient);
		double mv_speed = 2;
		double rot_speed = 1;

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float fElapsedTime;
		double phyz_time = 0;
		double timestep_duration = test_pengine->getStep_time();

		// adding rendering for contact points
		Mesh contact_ball_mesh = fromGeometry(phyz::ConvexUnionGeometry::merge(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.03), phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), 0.02, 0.1)), { 1.0, 0, 0 });
		struct Contact {
			mthz::Vec3 p;
			mthz::Vec3 n;
		};
		std::vector<Contact> all_contact_points;

		test_pengine->registerCollisionAction(phyz::CollisionTarget::all(), phyz::CollisionTarget::all(), [&](phyz::RigidBody* b1, phyz::RigidBody* b2,
			const std::vector<phyz::Manifold>& manifold) {
				for (const phyz::Manifold& m : manifold) {
					for (phyz::ContactP p : m.points) {
						all_contact_points.push_back({ p.pos, m.normal });
					}
				}
			}
		);

		//int tick_count = 0;

		while (rndr::render_loop(&fElapsedTime)) {
			// Listening to user input
			if (rndr::getKeyDown(GLFW_KEY_W)) {
				pos += orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_S)) {
				pos += orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
			}
			if (rndr::getKeyDown(GLFW_KEY_A)) {
				pos += orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_D)) {
				pos += orient.applyRotation(mthz::Vec3(1, 0, 0) * fElapsedTime * mv_speed);
			}

			if (rndr::getKeyDown(GLFW_KEY_UP)) {
				orient = orient * mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
			}
			else if (rndr::getKeyDown(GLFW_KEY_DOWN)) {
				orient = orient * mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
			}
			if (rndr::getKeyDown(GLFW_KEY_LEFT)) {
				orient = mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
			}
			else if (rndr::getKeyDown(GLFW_KEY_RIGHT)) {
				orient = mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
			}

			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				return TestOutcome::RESET;
			}
			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				paused = !paused;
			}

			/*if (tick_count == 55) {
				paused = true;
			}*/

			// running the test
			if (!paused) {
				phyz_time += fElapsedTime;
				phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			}
			else if (rndr::getKeyPressed(GLFW_KEY_T)) {
				phyz_time += timestep_duration; //advance exactly one frame
			}

			while (outcome == TestOutcome::STILL_RUNNING && phyz_time > timestep_duration) {
//				printf("%d\n", tick_count++);
				all_contact_points.clear();
				phyz_time -= timestep_duration;
				outcome = test->tickTestOnePhysicsStep();
			}

			// rendering
			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = pos;
			mthz::Quaternion cam_orient = orient;

			mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

			float aspect_ratio = (float)properties.window_height / properties.window_width;
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 500.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
			shader.setUniform3f("u_ambient_light", 0.4f, 0.4f, 0.4f);
			shader.setUniform3f("u_pointlight_pos", static_cast<float>(trnsfm_light_pos.x), static_cast<float>(trnsfm_light_pos.y), static_cast<float>(trnsfm_light_pos.z));
			shader.setUniform3f("u_pointlight_col", 0.6f, 0.6f, 0.6f);
			shader.setUniform1i("u_Asleep", false);

			for (const PhysBod& b : active_models) {

				Mesh transformed_mesh = getTransformed(b.mesh, b.r->getPos(), b.r->getOrientation(), cam_pos, cam_orient, b.r->getAsleep(), color{ 1.0f, 0.0f, 0.0f });

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			for (Contact c : all_contact_points) {
				mthz::Quaternion rot;
				double d = mthz::Vec3(0, 1, 0).dot(c.n);
				if (d < -0.99999) {
					rot = mthz::Quaternion(PI, mthz::Vec3(0, 0, 1));
				}
				else if (d < 0.99999) {
					mthz::Vec3 axis = mthz::Vec3(0, 1, 0).cross(c.n).normalize();
					double ang = acos(d);
					rot = mthz::Quaternion(ang, axis);
				}

				Mesh transformed_mesh = getTransformed(contact_ball_mesh, c.p, rot, cam_pos, cam_orient, false, color{ 1.0f, 0.0f, 0.0f });

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);

			if (outcome != TestOutcome::STILL_RUNNING) { return outcome; }
		}
	}

public:
	UnitTestsRunner(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~UnitTestsRunner() override {}

	std::map<std::string, std::string> askParameters() override {
		std::map<std::string, std::string> out;

		out["graphics_enabled"] = pickParameterFromOptions(
			"Choose whether to run with or without graphics enabled (y/n): ", { "y", "n" }
		);

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
			ControlDescription{"P", "Pause / Unpause the current test"},
			ControlDescription{"T", "Advance the test one tick when paused"},
			ControlDescription{"R", "Reset the current test"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	void run() override {
		bool rendering_enabled = parameters["graphics_enabled"] == "y";

		if (rendering_enabled) {
			paused = false;
			rndr::init(properties.window_width, properties.window_height, "Performance Demos");
		}

		std::vector<std::unique_ptr<TestGroup>> test_groups;
		test_groups.push_back(std::make_unique<ThreadManagerTestGroup>());
		test_groups.push_back(std::make_unique<HolonomicBlockSolverTestGroup>());
		test_groups.push_back(std::make_unique<CollisionTestGroup>());

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


		for (std::unique_ptr<Test>& t : tests) {
			std::string test_name = t->getTestName();
			if (t->getTestExpectation() == TestExpectationStatus::SKIP) {
				printf("(%s): SKIPPED\n", test_name.c_str());
				skipped_tests.push_back(t->getTestName());
				continue;
			}

			auto t1 = std::chrono::system_clock::now();
			TestOutcome outcome;
			do {
				phyz::PhysicsEngine* test_pengine = t->initTest(properties.n_threads, &active_models);
				if (rendering_enabled && t->canBeRunWithGraphics()) { outcome = runTestWithGraphics(test_pengine, t); }
				else { outcome = t->runWithoutGraphics(); }
				t->teardownTest();
				active_models.clear();
			} while (outcome == TestOutcome::RESET);
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

		if (rendering_enabled) {
			rndr::terminate();
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
};