#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include <chrono>
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class PerformanceProfilingScenes : public DemoScene {
public:
	PerformanceProfilingScenes(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~PerformanceProfilingScenes() override {

	}

	std::map<std::string, std::string> askParameters() override {
		std::map<std::string, std::string> out;

		out["graphics_enabled"] = pickParameterFromOptions(
			"Choose whether to run with or without graphics enabled (y/n): ", { "y", "n" }
		);

		out["which_scene"] = pickParameterFromOptions(
			"Options are the following:\n1) Box Pile - a bunch of cubes together \n\
2) Block Tower - circular tower of blocks\n\
3) Ragdoll Pile - one big pile of ragdolls. note ragdolls behave a bit oddly currently since rotation limits for ball sockets haven't been implemented yet\n\
4) Dispersed Ragdolls - 20 seperate piles of ragdolls\n\
Select which scene to run: ", { "1", "2", "3", "4"}
		);

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

	static void addBrickRing(phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, mthz::Vec3 block_dim, double radius, double n_blocks, mthz::Vec3 pos, int n_layers) {
		phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(mthz::Vec3(0.0, 0.0, -block_dim.z / 2.0), block_dim.x, block_dim.y, block_dim.z);

		double dtheta = 2 * PI / n_blocks;
		for (int i = 0; i < n_layers; i++) {
			double theta_offset = i * dtheta / 2;
			for (double theta = 0; theta < 2 * PI; theta += dtheta) {
				mthz::Quaternion rotation(theta + theta_offset, mthz::Vec3(0, 1, 0));

				mthz::Vec3 block_pos = pos + mthz::Vec3(0.0, block_dim.y * i, 0.0) + rotation.applyRotation(mthz::Vec3(radius, 0, 0));

				phyz::ConvexUnionGeometry block_oriented = block.getRotated(rotation).getTranslated(block_pos);
				phyz::RigidBody* r = p->createRigidBody(block_oriented);
				Mesh m = { fromGeometry(block_oriented) };
				bodies->push_back(PhysBod{ m, r });
			}
		}
	}

	static void addRagdoll(phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, mthz::Vec3 pos, double scale=1.0) {
		// TODO:
		// 1. add cone constraints + twist constraints 
		// 2. add capsules, and use them as the primary collider
		// 3. segment the torso into multiple capsules

		//torso
		mthz::Vec3 torso_dim = mthz::Vec3(0.5, 2, 1.5) * scale;
		mthz::Vec3 torso_lower_corner = pos - torso_dim / 2;
		phyz::ConvexUnionGeometry torso_geom = phyz::ConvexUnionGeometry::box(torso_lower_corner, torso_dim.x, torso_dim.y, torso_dim.z);
		phyz::RigidBody* torso_r = p->createRigidBody(torso_geom);
		bodies->push_back(PhysBod{ fromGeometry(torso_geom), torso_r });

		//head
		mthz::Vec3 neck_pos = pos + mthz::Vec3(0, torso_dim.y / 2, 0);
		double head_radius = 0.5 * scale;
		phyz::ConvexUnionGeometry head_geom = phyz::ConvexUnionGeometry::sphere(neck_pos + mthz::Vec3(0, head_radius, 0), head_radius);
		phyz::RigidBody* head_r = p->createRigidBody(head_geom);
		bodies->push_back(PhysBod{ fromGeometry(head_geom), head_r });

		//neck hinge
		phyz::ConstraintID neck_hinge = p->addHingeConstraint(torso_r, head_r, neck_pos, mthz::Vec3(0, 1, 0));
		p->addMotorConstraint(neck_hinge, -PI/2.0, PI/2.0);

		//thighs
		double thigh_radius = 0.3 * scale;
		double thigh_length = 1.5 * scale;
		double thigh_offset_from_center = 0.2 * torso_dim.z;

		//right thigh
		mthz::Vec3 right_thigh_bottom_center = pos + mthz::Vec3(0, -torso_dim.y / 2, 0) + mthz::Vec3(0, -thigh_length, -thigh_offset_from_center);
		phyz::ConvexUnionGeometry right_thigh_geom = phyz::ConvexUnionGeometry::cylinder(right_thigh_bottom_center, thigh_radius, thigh_length);
		phyz::RigidBody* right_thigh_r = p->createRigidBody(right_thigh_geom);
		bodies->push_back(PhysBod{ fromGeometry(right_thigh_geom), right_thigh_r });

		//hip joint to right thigh
		p->addBallSocketConstraint(right_thigh_r, torso_r, right_thigh_bottom_center + mthz::Vec3(0, thigh_length, 0));

		//left thigh
		mthz::Vec3 left_thigh_bottom_center = pos + mthz::Vec3(0, -torso_dim.y / 2, 0) + mthz::Vec3(0, -thigh_length, thigh_offset_from_center);
		phyz::ConvexUnionGeometry left_thigh_geom = phyz::ConvexUnionGeometry::cylinder(left_thigh_bottom_center, thigh_radius, thigh_length);
		phyz::RigidBody* left_thigh_r = p->createRigidBody(left_thigh_geom);
		bodies->push_back(PhysBod{ fromGeometry(left_thigh_geom), left_thigh_r });

		//hip joint to left thigh
		p->addBallSocketConstraint(left_thigh_r, torso_r, left_thigh_bottom_center + mthz::Vec3(0, thigh_length, 0));

		// shins
		double shin_radius = 0.25 * scale;
		double shin_length = 1.5 * scale;

		//right shin
		mthz::Vec3 right_shin_bottom_center = right_thigh_bottom_center + mthz::Vec3(0, -shin_length, 0);
		phyz::ConvexUnionGeometry right_shin_geom = phyz::ConvexUnionGeometry::cylinder(right_shin_bottom_center, shin_radius, shin_length);
		phyz::RigidBody* right_shin_r = p->createRigidBody(right_shin_geom);
		bodies->push_back(PhysBod{ fromGeometry(right_shin_geom), right_shin_r });

		// right knee joint
		phyz::ConstraintID right_knee_joint = p->addHingeConstraint(right_shin_r, right_thigh_r, right_thigh_bottom_center, mthz::Vec3(0, 0, 1));
		p->addMotorConstraint(right_knee_joint, 0, PI * 0.9);

		//left shin
		mthz::Vec3 left_shin_bottom_center = left_thigh_bottom_center + mthz::Vec3(0, -shin_length, 0);
		phyz::ConvexUnionGeometry left_shin_geom = phyz::ConvexUnionGeometry::cylinder(left_shin_bottom_center, shin_radius, shin_length);
		phyz::RigidBody* left_shin_r = p->createRigidBody(left_shin_geom);
		bodies->push_back(PhysBod{ fromGeometry(left_shin_geom), left_shin_r });

		// left knee joint
		phyz::ConstraintID left_knee_joint = p->addHingeConstraint(left_shin_r, left_thigh_r, left_thigh_bottom_center, mthz::Vec3(0, 0, 1));
		p->addMotorConstraint(left_knee_joint, 0, PI * 0.9);

		// feet
		double foot_length = 1 * scale;
		double foot_width = 0.5 * scale;
		double foot_height = 0.25 * scale;
		mthz::Vec3 foot_offset_from_bottom_of_shin = mthz::Vec3(foot_length - shin_radius, foot_height, foot_width / 2.0);

		// right foot
		phyz::ConvexUnionGeometry right_foot_geom = phyz::ConvexUnionGeometry::box(right_shin_bottom_center - foot_offset_from_bottom_of_shin, foot_length, foot_height, foot_width);
		phyz::RigidBody* right_foot_r = p->createRigidBody(right_foot_geom);
		bodies->push_back(PhysBod{ fromGeometry(right_foot_geom), right_foot_r });

		//right ankle joint
		p->addBallSocketConstraint(right_foot_r, right_shin_r, right_shin_bottom_center);

		// left foot
		phyz::ConvexUnionGeometry left_foot_geom = phyz::ConvexUnionGeometry::box(left_shin_bottom_center - foot_offset_from_bottom_of_shin, foot_length, foot_height, foot_width);
		phyz::RigidBody* left_foot_r = p->createRigidBody(left_foot_geom);
		bodies->push_back(PhysBod{ fromGeometry(left_foot_geom), left_foot_r });

		// left ankle joint
		p->addBallSocketConstraint(left_foot_r, left_shin_r, left_shin_bottom_center);

		// upper arm
		double upper_arm_radius = 0.2 * scale;
		double upper_arm_length = 1.5 * scale;

		//right upper arm
		mthz::Vec3 right_shoulder_position = pos + mthz::Vec3(0, torso_dim.y / 2.0 - upper_arm_radius, -torso_dim.z / 2.0);
		phyz::ConvexUnionGeometry right_upper_arm_geom = phyz::ConvexUnionGeometry::cylinder(right_shoulder_position, upper_arm_radius, upper_arm_length)
			.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(1, 0, 0)), right_shoulder_position);
		phyz::RigidBody* right_upper_arm_r = p->createRigidBody(right_upper_arm_geom);
		bodies->push_back(PhysBod{ fromGeometry(right_upper_arm_geom), right_upper_arm_r });

		// right shoulder joint
		p->addBallSocketConstraint(right_upper_arm_r, torso_r, right_shoulder_position);

		//left upper arm
		mthz::Vec3 left_shoulder_position = pos + mthz::Vec3(0, torso_dim.y / 2.0 - upper_arm_radius, torso_dim.z / 2.0);
		phyz::ConvexUnionGeometry left_upper_arm_geom = phyz::ConvexUnionGeometry::cylinder(left_shoulder_position, upper_arm_radius, upper_arm_length)
																				  .getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), left_shoulder_position);
		phyz::RigidBody* left_upper_arm_r = p->createRigidBody(left_upper_arm_geom);
		bodies->push_back(PhysBod{ fromGeometry(left_upper_arm_geom), left_upper_arm_r });

		// left shoulder joint
		p->addBallSocketConstraint(left_upper_arm_r, torso_r, left_shoulder_position);

		// forearm
		double fore_arm_radius = 0.15 * scale;
		double fore_arm_length = 1.5 * scale;

		// right forearm
		mthz::Vec3 right_elbow_position = right_shoulder_position + mthz::Vec3(0, 0, -upper_arm_length);
		phyz::ConvexUnionGeometry right_fore_arm_geom = phyz::ConvexUnionGeometry::cylinder(right_elbow_position, upper_arm_radius, upper_arm_length)
			.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(1, 0, 0)), right_elbow_position);
		phyz::RigidBody* right_fore_arm_r = p->createRigidBody(right_fore_arm_geom);
		bodies->push_back(PhysBod{ fromGeometry(right_fore_arm_geom), right_fore_arm_r });

		// right elbow joint
		phyz::ConstraintID right_elbow_constraint = p->addHingeConstraint(right_upper_arm_r, right_fore_arm_r, right_elbow_position, mthz::Vec3(0, 1, 0));
		p->addMotorConstraint(right_elbow_constraint, -PI * 0.9, 0);

		// left forearm
		mthz::Vec3 left_elbow_position = left_shoulder_position + mthz::Vec3(0, 0, upper_arm_length);
		phyz::ConvexUnionGeometry left_fore_arm_geom = phyz::ConvexUnionGeometry::cylinder(left_elbow_position, upper_arm_radius, upper_arm_length)
			                                                                     .getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), left_elbow_position);
		phyz::RigidBody* left_fore_arm_r = p->createRigidBody(left_fore_arm_geom);
		bodies->push_back(PhysBod{ fromGeometry(left_fore_arm_geom), left_fore_arm_r });

		// left elbow joint
		phyz::ConstraintID left_elbow_constraint = p->addHingeConstraint(left_upper_arm_r, left_fore_arm_r, left_elbow_position, mthz::Vec3(0, 1, 0));
		p->addMotorConstraint(left_elbow_constraint, 0, PI * 0.9);
	}

	enum SceneOptions { BLOCK_PILE, BLOCK_TOWER, RAGDOLL_PILE, DISPERSED_RAGDOLLS };

	void run() override {
		bool rendering_enabled = parameters["graphics_enabled"] == "y";
		SceneOptions selected_scene = BLOCK_PILE;
		std::string user_scene_input = parameters["which_scene"];
		if (user_scene_input == "1") selected_scene = BLOCK_PILE;
		else if (user_scene_input == "2") selected_scene = BLOCK_TOWER;
		else if (user_scene_input == "3") selected_scene = RAGDOLL_PILE;
		else if (user_scene_input == "4") selected_scene = DISPERSED_RAGDOLLS;

		if (rendering_enabled) {
			rndr::init(properties.window_width, properties.window_height, "Performance Demos");
		}

		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(false);

		p.setHolonomicSolverCFM(0.00001);

		double timestep = 1 / 60.0;
		p.setStep_time(timestep);

		std::vector<PhysBod> bodies;

		//************************
		//*******BASE PLATE*******
		//************************
		double s = 500;
		phyz::ConvexUnionGeometry geom2 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-s / 2, -2, -s / 2), s, 2, s);
		Mesh m2 = fromGeometry(geom2);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, phyz::RigidBody::FIXED);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		bodies.push_back({ m2, r2 });


		double height_max = 4.45;
		double height_min = 1;
		double height_delta_vel = 1.5;
		phyz::ConstraintID scissor_height_right;
		phyz::ConstraintID scissor_height_left;

		// setup for specific scenes
		if (selected_scene == BLOCK_PILE) {
			mthz::Vec3 pile_pos = mthz::Vec3(0, 0, 0);
			double block_size = 1;
			for (int x_indx = 0; x_indx < 10; x_indx++) {
				for (int y_indx = 0; y_indx < 20; y_indx++) {
					for (int z_indx = 0; z_indx < 10; z_indx++) {
						mthz::Vec3 box_pos = pile_pos + mthz::Vec3(x_indx * block_size, y_indx * block_size, z_indx * block_size);
						phyz::ConvexUnionGeometry box = phyz::ConvexUnionGeometry::box(box_pos, block_size, block_size, block_size);
						phyz::RigidBody* box_r = p.createRigidBody(box);
						bodies.push_back({ fromGeometry(box), box_r });
					}
				}
			}
		}
		else if (selected_scene == BLOCK_TOWER) {
			double radius = 4;
			mthz::Vec3 block_dim(1, 1, 2);
			mthz::Vec3 pos = mthz::Vec3(0, -0, 0);
			addBrickRing(&p, &bodies, block_dim, radius, 12, pos, 80);
		}
		else if (selected_scene == RAGDOLL_PILE) {
			mthz::Vec3 pos = mthz::Vec3(0, 8, 0);

			double y_offset = 6;
			for (int i = 0; i < 50; i++) {
				addRagdoll(&p, &bodies, pos + mthz::Vec3(0, y_offset * i, 0));
			}
		}
		else if (selected_scene == DISPERSED_RAGDOLLS) {
			mthz::Vec3 pos = mthz::Vec3(0, 8, 0);

			double y_spacing = 6;
			double pile_spacing = 20;

			for (int x_offset = 0; x_offset < 5; x_offset++) {
				for (int y_offset = 0; y_offset < 10; y_offset++) {
					for (int z_offset = 0; z_offset < 4; z_offset++) {
						addRagdoll(&p, &bodies, pos + mthz::Vec3(x_offset * pile_spacing, y_offset * y_spacing, z_offset * pile_spacing));
					}
				}
			}
		}

		p.setGravity(mthz::Vec3(0, -6.0, 0));
		if (!rendering_enabled) {
			// have main thread watching for user input on stdin to cancel.

			std::atomic<bool> exit = false;

			printf("Running profiling scene without rendering. Press Enter to stop the scene.\n");

			std::thread run_thread = std::thread([&]() {
				int update_frequency = 1000;
				while (!exit) {
					auto t1 = std::chrono::system_clock::now();
					for (int i = 0; !exit && i < update_frequency; i++) {
						p.timeStep();
					}
					auto t2 = std::chrono::system_clock::now();
					float duration = std::chrono::duration<float>(t2 - t1).count();
					if (!exit) {
						printf("1000 steps took %f seconds\n", duration);
					}
				}
			});

			if (fgetc(stdin)) {
				exit = true;
				run_thread.join();
				manager->deselectCurrentScene();
				return;
			}
		}
		else {
			bool lock_cam = false;
			mthz::Vec3 pos(25, 3, 0);
			mthz::Quaternion orient(PI / 2.0, mthz::Vec3(0, 1, 0));

			rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
			rndr::Shader shader("resources/shaders/Basic.shader");
			shader.bind();

			float t = 0;
			float fElapsedTime;

			double mv_speed = 2;
			double rot_speed = 1;

			double phyz_time = 0;
			int tick_count = 0;

			bool paused = false;

			while (rndr::render_loop(&fElapsedTime)) {

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
				if (rndr::getKeyPressed(GLFW_KEY_G)) {
					double block_size = 1.0;
					double block_speed = 15;

					mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
					phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(pos, 0.4, 0.4, 0.4);// .getRotated(mthz::Quaternion(PI / 4, mthz::Vec3(1, 0, 0)), pos);
					//phyz::ConvexUnionGeometry poly = phyz::ConvexUnionGeometry::polyCylinder(pos + mthz::Vec3(0, -0.5, 0), 1, 1);
					phyz::RigidBody* block_r = p.createRigidBody(block);

					block_r->setVel(camera_dir * block_speed);

					bodies.push_back({ fromGeometry(block), block_r });
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

				t += fElapsedTime;

				if (rndr::getKeyPressed(GLFW_KEY_Y)) {
					lock_cam = !lock_cam;

				}

				if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
					manager->deselectCurrentScene();
					return;
				}

				if (rndr::getKeyPressed(GLFW_KEY_P)) {
					paused = !paused;
				}

				double slow_factor = 1;

				if (!paused) {
					phyz_time += fElapsedTime;
				}
				phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
				while (phyz_time > slow_factor * timestep) {
					phyz_time -= slow_factor * timestep;
					p.timeStep();
					tick_count++;
				}

				rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
				batch_array.flush();

				mthz::Vec3 cam_pos = pos;
				mthz::Quaternion cam_orient = orient;

				mthz::Vec3 pointlight_pos(0.0, 225.0, 0.0);
				mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

				float aspect_ratio = (float)properties.window_height / properties.window_width;
				shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 550.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
				shader.setUniform3f("u_ambient_light", 0.4f, 0.4f, 0.4f);
				shader.setUniform3f("u_pointlight_pos", static_cast<float>(trnsfm_light_pos.x), static_cast<float>(trnsfm_light_pos.y), static_cast<float>(trnsfm_light_pos.z));
				shader.setUniform3f("u_pointlight_col", 0.6f, 0.6f, 0.6f);
				shader.setUniform1i("u_Asleep", false);

				for (const PhysBod& b : bodies) {

					Mesh transformed_mesh = getTransformed(b.mesh, b.r->getPos(), b.r->getOrientation(), cam_pos, cam_orient, b.r->getAsleep(), color{ 1.0f, 0.0f, 0.0f });

					if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
						rndr::draw(batch_array, shader);
						batch_array.flush();
					}
					batch_array.push(transformed_mesh.vertices.data(), static_cast<int>(transformed_mesh.vertices.size()), transformed_mesh.indices);
				}

				rndr::draw(batch_array, shader);
			}
		}
	}
};