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
#include "Tests/ConstraintTests.h"
#include "Tests/MassPropertyTests.h"
#include "Tests/RagdollTests.h"
#include "Tests/RaycastTests.h"
#include "Tests/TriangleMeshTests.h"

class UnitTestsRunner : public DemoScene {
private:
	bool paused;
	std::vector<PhysBod> active_models;
	std::vector<std::unique_ptr<TestGroup>> test_groups;

	mthz::Vec3 camera_pos;
	mthz::Quaternion camera_orient;
	
	TestOutcome runTestWithGraphics(phyz::PhysicsEngine* test_pengine, std::unique_ptr<Test>& test, bool reset_camera) {
		TestOutcome outcome = TestOutcome{ TestOutcomeState::STILL_RUNNING };

		assert(test_pengine != nullptr); // cant think of a need for a non-physics based right now.
		// basic rendering stuff
		if (reset_camera) {
			test->getCameraInitialPosition(&camera_pos, &camera_orient);
		}
		double mv_speed = 2;
		double rot_speed = 1;

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		rndr::Shader line_shader("resources/shaders/LineDraw.shader");
		shader.bind();

		bool object_highlighted = false;
		unsigned int highlighted_object_id = -1;

		bool something_hovered;
		mthz::Vec3 hover_pos;
		Mesh hover_ball = fromGeometry(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.01), { 1.0, 1.0, 1.0 });
		
		float fElapsedTime;
		double phyz_time = 0;
		double timestep_duration = test_pengine->getStep_time();

		// adding rendering for contact points
		Mesh contact_ball_mesh = fromGeometry(phyz::ConvexUnionGeometry::merge(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.03), phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), 0.02, 0.1)), { 1.0, 0, 0 });

		bool color_by_manifold = false;
		struct Contact {
			mthz::Vec3 p;
			mthz::Vec3 n;
			color c;
		};
		std::vector<Contact> all_contact_points;

		test_pengine->registerCollisionAction(phyz::CollisionTarget::all(), phyz::CollisionTarget::all(), [&](phyz::RigidBody* b1, phyz::RigidBody* b2,
			const std::vector<phyz::Manifold>& manifold) {
				for (const phyz::Manifold& m : manifold) {
					for (phyz::ContactP p : m.points) {

						// generate a psuedo random color from the magicID- should make a clear visualization a contact is preserved by its magicID
						uint64_t uid = std::hash<phyz::MagicID>{}(p.magicID);
						color c = {
							((uid & 0x0000FF) >> 0) / 255.0f,
							((uid & 0x00FF00) >> 8) / 255.0f,
							((uid & 0xFF0000) >> 16) / 255.0f
						};
						
						all_contact_points.push_back({ p.pos, m.normal, c });
					}
				}
			}
		);

		

		int tick_count = 0;

		rndr::lockMouse();
		double mouse_sensitivity = 0.0015;
		rndr::MousePos mouse_position = rndr::getMousePosition();

		while (rndr::render_loop(&fElapsedTime)) {
			// Listening to user input
			if (rndr::getKeyDown(GLFW_KEY_W)) {
				camera_pos += camera_orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_S)) {
				camera_pos += camera_orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
			}
			if (rndr::getKeyDown(GLFW_KEY_A)) {
				camera_pos += camera_orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_D)) {
				camera_pos += camera_orient.applyRotation(mthz::Vec3(1, 0, 0) * fElapsedTime * mv_speed);
			}
			
			if (rndr::getKeyDown(GLFW_KEY_ESCAPE)) {
				outcome.state = TestOutcomeState::CANCELED;
				return outcome;
			}
			if (rndr::getKeyDown(GLFW_KEY_LEFT_CONTROL) && rndr::getKeyPressed(GLFW_KEY_S)) {
				outcome.state = TestOutcomeState::SKIPPED;
				return outcome;
			}

			// mouse controlled camera movement
			rndr::MousePos new_mouse = rndr::getMousePosition();
			double mouse_delta_x = new_mouse.x - mouse_position.x;
			double mouse_delta_y = new_mouse.y - mouse_position.y;
			mouse_position = new_mouse;

			camera_orient = camera_orient * mthz::Quaternion(mouse_sensitivity * mouse_delta_y, mthz::Vec3(1, 0, 0));
			camera_orient = mthz::Quaternion(-mouse_sensitivity * mouse_delta_x, mthz::Vec3(0, 1, 0)) * camera_orient;

			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				return TestOutcome{ TestOutcomeState::RESET };
			}
			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				paused = !paused;
			}

			mthz::Vec3 camera_dir = camera_orient.applyRotation(mthz::Vec3(0, 0, -1));
			phyz::RayHitInfo hit_info = test_pengine->raycastFirstIntersection(camera_pos, camera_dir);
			if (hit_info.did_hit) {
				something_hovered = true;
				hover_pos = hit_info.hit_position;
			}
			else {
				something_hovered = false;
			}

			if (rndr::getMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT)) {

				if (hit_info.did_hit) {
					object_highlighted = true;
					highlighted_object_id = hit_info.hit_object->getID();
					printf("selected object id: %u\n", highlighted_object_id);

					if (hit_info.hit_object->geometry_type == phyz::RigidBody::STATIC_MESH) {
						phyz::RigidBody* r = hit_info.hit_object;
						phyz::StaticMeshGeometry& body_mesh = r->mesh;
						uint32_t hit_face_index = body_mesh.testRayIntersection(camera_pos, camera_dir).hit_triangle_inedex;
						printf("\thit triangle index: %u\n", hit_face_index);

						// print c++ code to construct a new static mesh of just this triangle, preseving the gauss map info
						phyz::StaticMeshFace hit_mesh_face = body_mesh.get_triangle(hit_face_index);
						std::string verts_argument = "{";
						std::string half_edges_argument = "{";
						for (int i = 0; i < 3; i++) {
							char suffix = i == 2 ? '}' : ',';
							phyz::StaticMeshVertex vert = body_mesh.get_vertex(hit_mesh_face.vertex_indices[i]);
							// construct gauss map arg
							std::string vert_gauss_map_arg = "{}";
							if (vert.valid_normal_gauss_map.size() > 0) {
								vert_gauss_map_arg = "{";
								for (mthz::Vec3 v : vert.valid_normal_gauss_map) {
									vert_gauss_map_arg += std::format("{{{},{},{}}},", v.x, v.y, v.z);
								}
								vert_gauss_map_arg[vert_gauss_map_arg.size() - 1] = '}'; // replace final , with a }
							}
							// add the whole vert struct to the arg
							verts_argument += std::format("phyz::StaticMeshVertex{{mthz::Vec3({},{},{}),0,{}}}{}", vert.p.x, vert.p.y, vert.p.z, vert_gauss_map_arg, suffix);

							phyz::StaticMeshHalfEdge edge = body_mesh.get_half_edge(hit_mesh_face.half_edge_indices[i]);
							//add the half edge struct to the arg. all the 0s are args that are not needed, as the debug StaticMeshConstructor replaces them anyways
							half_edges_argument += std::format(
								"phyz::StaticMeshHalfEdge{{0,0,0,0,0,0,mthz::Vec3({},{},{}),0,{},mthz::Vec3({},{},{}),mthz::Vec3({},{},{})}}{}",
								edge.out_direction.x, edge.out_direction.y, edge.out_direction.z,
								edge.has_gauss_arc,
								edge.gauss_arc_g1.x, edge.gauss_arc_g1.y, edge.gauss_arc_g1.z,
								edge.gauss_arc_g2.x, edge.gauss_arc_g2.y, edge.gauss_arc_g2.z,
								suffix
							);
						}

						printf("phyz::StaticMeshGeometry({%s}, {%s});\n", verts_argument.c_str(), half_edges_argument.c_str());
					}
				}
				else {
					object_highlighted = false;
				}
			}

			if (tick_count == 96) {
				//for (auto itr = active_models.begin(); itr != active_models.end();) {
				//	PhysBod& pb = *itr;
				//	if (pb.r->getID() == 10 || pb.r->getMovementType() == phyz::RigidBody::FIXED) { itr++; }
				//	else {
				//		test_pengine->removeRigidBody(pb.r);
				//		itr = active_models.erase(itr); 
				//	}
				//}
				//paused = true;
			}

			// running the test
			if (!paused) {
				phyz_time += fElapsedTime;
				phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			}
			else if (rndr::getKeyPressed(GLFW_KEY_T)) {
				phyz_time += timestep_duration; //advance exactly one frame
				printf("tick: %d\n", tick_count);
			}

			while (outcome.state == TestOutcomeState::STILL_RUNNING && phyz_time > timestep_duration) {
				all_contact_points.clear();
				phyz_time -= timestep_duration;
				outcome = test->tickTestOnePhysicsStep();
				tick_count++;
			}

			// rendering
			rndr::clear(rndr::color(0.9f, 0.9f, 0.95f));
			batch_array.flush();
			shader.bind();

			mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = camera_orient.conjugate().applyRotation(pointlight_pos - camera_pos);

			float aspect_ratio = (float)properties.window_height / properties.window_width;
			rndr::Mat4 proj_mat = rndr::Mat4::proj(0.1f, 500.0f, 2.0f, 2.0f * aspect_ratio, 60.0f);
			shader.setUniformMat4f("u_P", proj_mat);
			shader.setUniform3f("u_ambient_light", 1.0f, 1.0f, 1.0f);
			shader.setUniform3f("u_pointlight_pos", static_cast<float>(trnsfm_light_pos.x), static_cast<float>(trnsfm_light_pos.y), static_cast<float>(trnsfm_light_pos.z));
			shader.setUniform3f("u_pointlight_col", 1.0f, 1.0f, 1.0f);
			shader.setUniform1i("u_Asleep", false);

			for (const PhysBod& b : active_models) {

				bool is_highlighted = object_highlighted && b.r->getID() == highlighted_object_id;
				color override_color = is_highlighted ? color{ 1.0, 1.0, 0.0 } : color{ 1.0, 0.0, 0.0 };
				bool color_overriden = is_highlighted || b.r->getAsleep();

				Mesh transformed_mesh = getTransformed(b.mesh, b.r->getPos(), b.r->getOrientation(), camera_pos, camera_orient, color_overriden, override_color);

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

				Mesh transformed_mesh = getTransformed(contact_ball_mesh, c.p, rot, camera_pos, camera_orient, true, c.c);

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			if (something_hovered) {
				Mesh transformed_mesh = getTransformed(hover_ball, hover_pos, mthz::Quaternion(), camera_pos, camera_orient);

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);

			batch_array.flush();

			if (outcome.state != TestOutcomeState::STILL_RUNNING) { return outcome; }
		}

		// we should never actually hit this
		assert(false);
		return outcome;
	}

public:
	UnitTestsRunner(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties), paused(false) {
		test_groups.push_back(std::make_unique<CollisionTestGroup>());
		test_groups.push_back(std::make_unique<HolonomicBlockSolverTestGroup>());
		test_groups.push_back(std::make_unique<ConstraintTestsGroup>());
		test_groups.push_back(std::make_unique<ThreadManagerTestGroup>());
		test_groups.push_back(std::make_unique<MassPropertiesTestGroup>());
		test_groups.push_back(std::make_unique<RagdollTestGroup>());
		test_groups.push_back(std::make_unique<RaycastTestGroup>());
		test_groups.push_back(std::make_unique<TriangleMeshTestGroup>());
	}

	~UnitTestsRunner() override {}

	std::map<std::string, std::string> askParameters() override {
		std::map<std::string, std::string> out;

		// pick whether to run all test groups, or choose a specific one.
		std::string test_groups_options = "The available test groups are the following:\n";
		for (int i = 0; i <= test_groups.size(); i++) {
			std::string option_name = (i == test_groups.size()) ? "Run All Groups" : test_groups[i]->getGroupName();
			test_groups_options += std::format("({}) {}\n", i, option_name);
		}
		test_groups_options += "\n";

		out["selected_test_group"] = pickInteger(
			test_groups_options + "Choose to run one, or all of the above tests: ", 0, static_cast<int>(test_groups.size())
		);
		
		// if only running a specific test group, also choose to all tests in the group, or only a specific test
		int selected_indx = std::stoi(out["selected_test_group"]);
		if (selected_indx != test_groups.size()) {
			const std::unique_ptr<TestGroup>& selected_grp = test_groups[selected_indx];
			if (selected_grp->getTests().size() == 1) {
				out["selected_tests"] = "0";
			}
			else {
				std::string test_options = "The available tests in the selected group are the following:\n";
				for (int i = 0; i <= selected_grp->getTests().size(); i++) {
					std::string option_name = (i == selected_grp->getTests().size()) ? "Run All Tests" : selected_grp->getTests()[i]->getTestName();
					test_options += std::format("({}) {}\n", i, option_name);
				}
				test_options += "\n";

				out["selected_tests"] = pickInteger(
					test_options + "Choose to run one, or all of the above tests: ", 0, static_cast<int>(selected_grp->getTests().size())
				);
			}
		}

		// choose whether to run in interactive mode with graphics
		out["graphics_enabled"] = pickParameterFromOptions(
			"Choose whether to run with or without graphics enabled (y/n): ", { "y", "n" }
		);

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
			bool backface_culling_enabled = false;
			rndr::init(properties.window_width, properties.window_height, "Performance Demos", backface_culling_enabled);
		}
		
		std::vector<std::unique_ptr<Test>> tests;
		int selected_test_group_index = std::stoi(parameters["selected_test_group"]);
		if (selected_test_group_index == test_groups.size()) {
			// user selected to run all groups
			for (const std::unique_ptr<TestGroup>& gr : test_groups) {
				for (std::unique_ptr<Test>& t : gr->getTests()) {
					tests.push_back(std::move(t));
				}
			}
		}
		else {
			const std::unique_ptr<TestGroup>& selected_grp = test_groups[selected_test_group_index];
			int selected_test_index = std::stoi(parameters["selected_tests"]);
			if (selected_test_index == selected_grp->getTests().size()) {
				//user selected to run all tests in group
				for (std::unique_ptr<Test>& t : selected_grp->getTests()) {
					tests.push_back(std::move(t));
				}
			}
			else {
				tests.push_back(std::move(selected_grp->getTests()[selected_test_index]));
			}
		}	
		
		printf("Collected %d tests...\n", static_cast<uint32_t>(tests.size()));

		std::vector<std::string> passed_tests;
		std::vector<std::pair<std::string, std::string>> failed_tests;
		std::vector<std::string> xpassed_tests;
		std::vector<std::string> xfailed_tests;
		std::vector<std::string> skipped_tests;

		bool skip_all_remaining_tests = false;
		for (std::unique_ptr<Test>& t : tests) {
			std::string test_name = t->getTestName();
			if (skip_all_remaining_tests || t->getTestExpectation() == TestExpectationStatus::SKIP) {
				printf("(%s): SKIPPED\n", test_name.c_str());
				skipped_tests.push_back(t->getTestName());
				continue;
			}

			auto t1 = std::chrono::system_clock::now();
			TestOutcome outcome;
			bool reset_camera_on_test_start = true;
			do {
				phyz::PhysicsEngine* test_pengine = t->initTest(properties.n_threads, &active_models);
				if (rendering_enabled && t->canBeRunWithGraphics()) { outcome = runTestWithGraphics(test_pengine, t, reset_camera_on_test_start); }
				else { outcome = t->runWithoutGraphics(); }
				t->teardownTest();
				active_models.clear();
				if (outcome.state == TestOutcomeState::RESET) { reset_camera_on_test_start = false; }
			} while (outcome.state == TestOutcomeState::RESET);
			auto t2 = std::chrono::system_clock::now();
			float duration = std::chrono::duration<float, std::milli>(t2 - t1).count();

			const char* START_GREEN_TEXT = "\x1B[32m";
			const char* START_RED_TEXT = "\x1B[31m";
			const char* START_YELLOW_TEXT = "\x1B[33m";
			const char* END_TEXT_COLORING = "\033[0m";

			if (outcome.state == PASSED && t->getTestExpectation() == TestExpectationStatus::XFAIL) {
				printf("(%s): %sXPASSED%s | duration %fms\n", test_name.c_str(), START_GREEN_TEXT, END_TEXT_COLORING, duration);
				xpassed_tests.push_back(test_name);
			}
			else if (outcome.state == PASSED) {
				printf("(%s): %sPASSED%s | duration %fms\n", test_name.c_str(), START_GREEN_TEXT, END_TEXT_COLORING, duration);
				passed_tests.push_back(test_name);
			}
			else if (outcome.state == FAILED && t->getTestExpectation() == TestExpectationStatus::XFAIL) {
				printf("(%s): %sXFAILED%s | duration %fms\n", test_name.c_str(), START_YELLOW_TEXT, END_TEXT_COLORING, duration);
				xfailed_tests.push_back(test_name);
			}
			else if (outcome.state == FAILED) {
				printf("(%s): %sFAILED%s | duration %fms\n", test_name.c_str(), START_RED_TEXT, END_TEXT_COLORING, duration);
				failed_tests.push_back(std::pair(test_name, outcome.reason));
			}
			else if (outcome.state == TestOutcomeState::SKIPPED) {
				// user input indicated to skip this specific test:
				printf("(%s): %sSKIPPED%s | duration %fms\n", test_name.c_str(), START_YELLOW_TEXT, END_TEXT_COLORING, duration);
				skipped_tests.push_back(t->getTestName());
			}
			else if (outcome.state == TestOutcomeState::CANCELED) {
				// user input indicated they want to stop running remaining tests:
				skip_all_remaining_tests = true;
				printf("(%s): %sCANCELED%s | duration %fms\n", test_name.c_str(), START_YELLOW_TEXT, END_TEXT_COLORING, duration);
				skipped_tests.push_back(t->getTestName());
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
			for (const std::pair<std::string, std::string>& test : failed_tests) {
				printf("%s\n", test.first.c_str()); //print failed test name
				if (test.second != "") { printf("\t%s\n", test.second.c_str()); } //print reason, if there is one.
			}
		}
		

		printf("\nPress enter to exit.\n");
		fgetc(stdin);
		manager->deselectCurrentScene();
	}
};