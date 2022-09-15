#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "iostream"

class ImageDemo : public DemoScene {
public:
	ImageDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~ImageDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"G", "toggle free-look and lock to vehicle"},
			ControlDescription{"B", "Lock the rear right wheel"},
			ControlDescription{"I. K", "Incrase, Decrease throttle"},
			ControlDescription{"J, L", "Steer left, right"},
		};
	}

	static void render_progress_bar(float percent, int width, bool done) {
		if (!done) {
			printf("[");
			int prog = percent * width;
			for (int i = 0; i < width; i++) {
				if (i <= prog) {
					printf("#");
				}
				else {
					printf("-");
				}
			}
			printf("] %d%% %\r", (int)(100 * percent));
		}
		else {
			printf("[");
			for (int i = 0; i < width; i++) {
				printf("#");
			}
			printf("] %%100\n");
		}
	}

	static struct PosState {
		mthz::Vec3 pos;
		mthz::Quaternion orient;
	};

	static struct BodyHistory {
		BodyHistory(phyz::RigidBody* r) : r(r) {}

		phyz::RigidBody* r;
		std::vector<PosState> history;
	};

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "Car Demo");

		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		//p.setSleepingEnabled(false);
		p.setPGSIterations(45, 35);

		bool paused = false;
		bool slow = false;
		bool lock_cam = true;

		std::vector<PhysBod> bodies;
		std::vector<BodyHistory> moving_bodies;

		mthz::Vec3 base_dim(8.5, 0.25, 1.0);
		phyz::Geometry base = phyz::Geometry::box(mthz::Vec3(), base_dim.x, base_dim.y, base_dim.z);
		double box_height = 48;
		phyz::Geometry negx_wall = phyz::Geometry::box(mthz::Vec3(0, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
		phyz::Geometry posx_wall = phyz::Geometry::box(mthz::Vec3(base_dim.x - base_dim.y, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
		phyz::Geometry back_wall = phyz::Geometry::box(mthz::Vec3(base_dim.y, base_dim.y, 0), base_dim.x - 2 * base_dim.y, box_height, base_dim.y);
		phyz::Geometry front_wall = phyz::Geometry::box(mthz::Vec3(0, 0, base_dim.z), base_dim.x, base_dim.y + box_height, base_dim.y);

		double effective_width = base_dim.x - 2 * base_dim.y;
		int n_rows = 4;
		int n_pins = 3;
		double pin_gap = effective_width / (n_pins + 1);
		double row_verticle_spacing = 2;
		double cylinder_radius = 0.15;
		double pin_start_y = 10;
		for (int i = 0; i < n_rows; i++) {
			double start_x = base_dim.y + pin_gap * ((i % 2 == 0) ? 1 : 1.5);
			for (int j = 0; j < ((i % 2 == 0) ? n_pins : n_pins - 1); j++) {
				mthz::Vec3 pin_pos = mthz::Vec3(start_x + pin_gap * j, pin_start_y + row_verticle_spacing * i, base_dim.y);
				phyz::Geometry pin = phyz::Geometry::cylinder(pin_pos,cylinder_radius, base_dim.z - base_dim.y)
					.getRotated(mthz::Quaternion(3.1415926535/2.0, mthz::Vec3(1, 0, 0)), pin_pos);

				bodies.push_back({ fromGeometry(pin),  p.createRigidBody(pin, true) });
			}
		}

		double spinner_y = 23;
		double spinner_radius = effective_width / 4.5;
		
		mthz::Vec3 spinner1_pos = mthz::Vec3(base_dim.y + effective_width/4.0, spinner_y, base_dim.y);
		mthz::Vec3 spinner2_pos = mthz::Vec3(base_dim.y + effective_width * 3 / 4.0, spinner_y, base_dim.y);
		phyz::Geometry spinner1 = phyz::Geometry::gear(spinner1_pos, cylinder_radius * 2, spinner_radius, base_dim.z - base_dim.y, 4)
			.getRotated(mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), spinner1_pos);
		phyz::Geometry spinner2 = phyz::Geometry::gear(spinner2_pos, cylinder_radius * 2, spinner_radius, base_dim.z - base_dim.y, 4)
			.getRotated(mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), spinner2_pos);

		double block_start_height = box_height;
		for (int i = 0; i < 16; i++) {
			for (int j = 0; j < 64; j++) {
				for (int k = 0; k < 2; k++) {
					double size = effective_width / 32;
					mthz::Vec3 pos(base_dim.y + effective_width/2.0 + (i - 2) * size, block_start_height + 1.5 * j * size, base_dim.y + k * size);
					phyz::Geometry cube = phyz::Geometry::box(pos, size, size, size);
					phyz::RigidBody* r = p.createRigidBody(cube);

					moving_bodies.push_back(BodyHistory(r));
					bodies.push_back({ fromGeometry(cube), r });
				}
			}
		}

		phyz::RigidBody* base_r = p.createRigidBody(base, true);
		Mesh base_m = fromGeometry(base);
		phyz::RigidBody* negx_wall_r = p.createRigidBody(negx_wall, true);
		Mesh negx_wall_m = fromGeometry(negx_wall);
		phyz::RigidBody* posx_wall_r = p.createRigidBody(posx_wall, true);
		Mesh posx_wall_m = fromGeometry(posx_wall);
		phyz::RigidBody* back_wall_r = p.createRigidBody(back_wall, true);
		Mesh back_wall_m = fromGeometry(back_wall);
		phyz::RigidBody* front_wall_r = p.createRigidBody(front_wall, true);
		phyz::RigidBody* spinner1_r = p.createRigidBody(spinner1);
		Mesh spinner1_m = fromGeometry(spinner1);
		phyz::RigidBody* spinner2_r = p.createRigidBody(spinner2);
		Mesh spinner2_m = fromGeometry(spinner2);

		phyz::MotorID spinner1_motor = p.addHingeConstraint(front_wall_r, spinner1_r, spinner1_pos, spinner1_pos, mthz::Vec3(0, 0, 1), mthz::Vec3(0, 0, 1));
		p.setMotor(spinner1_motor, 0.5, 1000000);
		phyz::MotorID spinner2_motor = p.addHingeConstraint(front_wall_r, spinner2_r, spinner2_pos, spinner2_pos, mthz::Vec3(0, 0, 1), mthz::Vec3(0, 0, 1));
		p.setMotor(spinner2_motor, -0.5, 1000000);

		bodies.push_back({ base_m, base_r });
		bodies.push_back({ negx_wall_m, negx_wall_r });
		bodies.push_back({ posx_wall_m, posx_wall_r });
		bodies.push_back({ back_wall_m, back_wall_r });
		bodies.push_back({ spinner1_m, spinner1_r });
		bodies.push_back({ spinner2_m, spinner2_r });

		moving_bodies.push_back(BodyHistory(spinner1_r));
		moving_bodies.push_back(BodyHistory(spinner2_r));

		mthz::Vec3 cam_pos = mthz::Vec3(4, 8, 16);
		mthz::Quaternion cam_orient;

		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		
		p.setGravity(mthz::Vec3(0, -4.9, 0));

		printf("Precomputing...\n");
		int curr_percent = -1;
		double frame_time = 1 / 120.0;
		int steps_per_frame = 3;
		double simulation_time = 30;
		int progress_bar_width = 50;

		p.setStep_time(frame_time / steps_per_frame);
		for (BodyHistory& b : moving_bodies) {
			b.history.reserve(simulation_time / frame_time);
			b.history.push_back({ b.r->getPos(), b.r->getOrientation() });
		}

		int n_frames = 0;
		for (double t = 0; t <= simulation_time; t += frame_time) {
			int new_percent = 100 * t / simulation_time;
			if (new_percent > curr_percent) {
				curr_percent = new_percent;
				render_progress_bar(t / simulation_time, progress_bar_width, false);
			}

			for (int i = 0; i < steps_per_frame; i++) {
				p.timeStep();
			}
			for (BodyHistory& b : moving_bodies) {
				b.history.push_back({ b.r->getPos(), b.r->getOrientation() });
			}
			n_frames++;
		}
		render_progress_bar(curr_percent, progress_bar_width, true);

		//printf("Press enter to start:\n");
		//std::string unused;
		//std::getline(std::cin, unused);

		double t = 0;
		float fElapsedTime;
		int curr_frame = -1;

		while(rndr::render_loop(&fElapsedTime)) {

			t += fElapsedTime;

			//if (!paused) {
			//	if (slow) {
			//		phyz_time += fElapsedTime * 0.1;
			//	}
			//	else {
			//		phyz_time += fElapsedTime;
			//	}
			//}
			//phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			//if (phyz_time > 1 / 30.0) {
			//	//	printf("n objects: %d\n", p.getNumBodies());
			//}
			//while (phyz_time > frame_time / steps_per_frame) {
			//	phyz_time -= frame_time / steps_per_frame;
			//	p.timeStep();
			//}

			int new_frame = t / frame_time;
			if (new_frame != curr_frame && new_frame < n_frames) {
				curr_frame = new_frame;
				for (const BodyHistory& b : moving_bodies) {
					b.r->setOrientation(b.history[curr_frame].orient);
					b.r->setToPosition(b.history[curr_frame].pos);
				}
			}

			if (rndr::getKeyDown(GLFW_KEY_W)) {
				cam_pos += cam_orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_S)) {
				cam_pos += cam_orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
			}
			if (rndr::getKeyDown(GLFW_KEY_A)) {
				cam_pos += cam_orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_D)) {
				cam_pos += cam_orient.applyRotation(mthz::Vec3(1, 0, 0) * fElapsedTime * mv_speed);
			}

			if (rndr::getKeyDown(GLFW_KEY_UP)) {
				cam_orient = cam_orient * mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
			}
			else if (rndr::getKeyDown(GLFW_KEY_DOWN)) {
				cam_orient = cam_orient * mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
			}
			if (rndr::getKeyDown(GLFW_KEY_LEFT)) {
				cam_orient = mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * cam_orient;
			}
			else if (rndr::getKeyDown(GLFW_KEY_RIGHT)) {
				cam_orient = mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * cam_orient;
			}

			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				t = 0;
			}

			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

			for (const PhysBod& b : bodies) {
				shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0) * rndr::Mat4::cam_view(cam_pos, cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
				shader.setUniform1i("u_Asleep", false && b.r->getAsleep());
				rndr::draw(*b.mesh.va, *b.mesh.ib, shader);
			}
		}

		//while (rndr::render_loop(&fElapsedTime)) {

		//	if (rndr::getKeyDown(GLFW_KEY_W)) {
		//		cam_pos += cam_orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
		//	}
		//	else if (rndr::getKeyDown(GLFW_KEY_S)) {
		//		cam_pos += cam_orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
		//	}
		//	if (rndr::getKeyDown(GLFW_KEY_A)) {
		//		cam_pos += cam_orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
		//	}
		//	else if (rndr::getKeyDown(GLFW_KEY_D)) {
		//		cam_pos += cam_orient.applyRotation(mthz::Vec3(1, 0, 0) * fElapsedTime * mv_speed);
		//	}

		//	if (rndr::getKeyDown(GLFW_KEY_UP)) {
		//		cam_orient = cam_orient * mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
		//	}
		//	else if (rndr::getKeyDown(GLFW_KEY_DOWN)) {
		//		cam_orient = cam_orient * mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
		//	}
		//	if (rndr::getKeyDown(GLFW_KEY_LEFT)) {
		//		cam_orient = mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * cam_orient;
		//	}
		//	else if (rndr::getKeyDown(GLFW_KEY_RIGHT)) {
		//		cam_orient = mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * cam_orient;
		//	}

		//	t += fElapsedTime;

		//	if (!paused) {
		//		if (slow) {
		//			phyz_time += fElapsedTime * 0.1;
		//		}
		//		else {
		//			phyz_time += fElapsedTime;
		//		}
		//	}
		//	phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
		//	if (phyz_time > 1 / 30.0) {
		//		//	printf("n objects: %d\n", p.getNumBodies());
		//	}
		//	while (phyz_time > timestep) {
		//		phyz_time -= timestep;
		//		p.timeStep();
		//	}


		//	rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

		//	for (const PhysBod& b : bodies) {
		//		shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0) * rndr::Mat4::cam_view(cam_pos, cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
		//		shader.setUniform1i("u_Asleep", b.r->getAsleep());
		//		rndr::draw(*b.mesh.va, *b.mesh.ib, shader);

		//	}
		//}
	}
};