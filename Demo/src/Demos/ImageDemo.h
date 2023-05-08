#pragma once
#define OLC_PGE_APPLICATION
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include <iostream>
#include "olcPixelGameEngine.h"
#include <inttypes.h>
#include <io.h>
#include <fcntl.h>
#include <cassert>

class ImageDemo : public DemoScene {
private:
	void render_progress_bar(float percent, int width, bool done) {
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

	struct PosState {
		mthz::Vec3 pos;
		mthz::Quaternion orient;
	};

	struct BodyHistory {
		BodyHistory(phyz::RigidBody* r, const phyz::Geometry& g, color c={0.4, 0.4, 0.4}) : r(r), g(g), color(c) {}

		phyz::RigidBody* r;
		phyz::Geometry g;
		std::vector<PosState> history;
		color color;
	};

	bool fileExists(const std::string& s) {
		std::ifstream f(s);
		return f.good();
	}

	color averagePixels(olc::Sprite& s, double start_x, double start_y, double end_x, double end_y, double dim_x, double dim_y) {
		color average = { 0.0f, 0.0f, 0.0f };
		int n_pixel = 0;

		//0.5 is to check for inclusion of center of pixel, not top left corner

		for (int x = start_x * s.width / dim_x + 0.5; x <= end_x * s.width / dim_x + 0.5; x++) {
			for (int y = start_y * s.height / dim_y + 0.5; y <= end_y * s.height / dim_y + 0.5; y++) {
				if (x >= 0 && x < s.width && y >= 0 && y < s.height) {
					olc::Pixel p = s.GetPixel(x, y);
					average.r += p.r / 255.0f; average.g += p.g / 255.0f; average.b += p.b / 255.0f;
					n_pixel++;
				}
			}
		}
		average.r /= n_pixel; average.g /= n_pixel; average.b /= n_pixel;
		return average;
	}

	static const int BUFFER_SIZE = 512;
	int buffer_indx;
	int buffer_capacity = 0;
	double buffer[BUFFER_SIZE];

	double read64(int fd) {
		if (buffer_indx >= buffer_capacity) {
			buffer_indx = 0;
			buffer_capacity = _read(fd, buffer, sizeof(double) * BUFFER_SIZE) / sizeof(double);
			assert(buffer_capacity != 0);
		}
		return buffer[buffer_indx++];
	}

	void write64(int fd, double v) {
		if (buffer_indx >= BUFFER_SIZE) {
			flush(fd);
		}
		buffer[buffer_indx++] = v;
	}

	void flush(int fd) {
		_write(fd, buffer, sizeof(double) * buffer_indx);
		buffer_indx = 0;
	}

	void readComputation(std::string filename, std::vector<BodyHistory>* bodies, int n_frames) {
		int fd;
		buffer_indx = 0;
		buffer_capacity = 0;
		_sopen_s(&fd, filename.c_str(), _O_BINARY | _O_RDONLY, _SH_DENYRW, _S_IREAD);
		for (int i = 0; i < n_frames; i++) {
			for (BodyHistory& b : *bodies) {
				double x = read64(fd); double y = read64(fd); double z = read64(fd);
				double r = read64(fd); double i = read64(fd); double j = read64(fd); double k = read64(fd);
				b.history.push_back({ mthz::Vec3(x, y, z), mthz::Quaternion(r, i, j, k) });
			}
		}
		_close(fd);
	}

	void writeComputation(std::string filename, const std::vector<BodyHistory>& bodies, int n_frames) {
		int fd;
		buffer_indx = 0;
		_sopen_s(&fd, filename.c_str(), _O_CREAT | _O_BINARY | _O_WRONLY, _SH_DENYRW, _S_IWRITE);
		for (int i = 0; i < n_frames; i++) {
			for (const BodyHistory& b : bodies) {
				PosState p = b.history[i];
				write64(fd, p.pos.x); write64(fd, p.pos.y); write64(fd, p.pos.z);
				write64(fd, p.orient.r); write64(fd, p.orient.i); write64(fd, p.orient.j); write64(fd, p.orient.k);
			}
		}
		flush(fd);
		_close(fd);
	}

public:
	ImageDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~ImageDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when the simulation starts"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"R", "Re-run simulation"},
			ControlDescription{"ESC", "Return to main menu"}
		};
	}

	void run() override {

		bool default_coloring = false;
		printf("Give a filepath for an image to desplay. Enter empty for default coloring: ");
		std::string image_file = "";
		do {
			std::getline(std::cin, image_file);
			if (image_file == "") {
				default_coloring = true;
				break;
			}
			else if (!fileExists(image_file)) {
				printf("File not found, enter again: ");
			}
			else {
				break;
			}
		} while (1);

		printf("Select level of detail. Note computation time scales massively. Options are 1, 2, 3: ");
		std::string detail_input;
		int level_of_detail;
		do {
			std::getline(std::cin, detail_input);
			if (detail_input != "1" && detail_input != "2" && detail_input != "3") {
				printf("Invalid input, enter again. Valid inputs are 1, 2, 3: ");
			}
			else {
				level_of_detail = atoi(detail_input.c_str());
				break;
			}
		} while (1);

		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(true);
		p.setPGSIterations(45, 35);

		bool paused = false;
		bool slow = false;
		bool lock_cam = true;

		std::vector<BodyHistory> pre_bodies;
		std::vector<int> recolor_body_indexes;

		mthz::Vec3 base_dim(8.5, 0.25, 0.75);
		phyz::Geometry base = phyz::Geometry::box(mthz::Vec3(), base_dim.x, base_dim.y, base_dim.z);
		double box_height = 48;
		phyz::Geometry negx_wall = phyz::Geometry::box(mthz::Vec3(0, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
		phyz::Geometry posx_wall = phyz::Geometry::box(mthz::Vec3(base_dim.x - base_dim.y, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
		phyz::Geometry back_wall = phyz::Geometry::box(mthz::Vec3(0, 0, 0), base_dim.x, box_height + base_dim.y, -base_dim.y);
		phyz::Geometry front_wall = phyz::Geometry::box(mthz::Vec3(0, 0, base_dim.z), base_dim.x, base_dim.y + box_height, base_dim.y);

		p.setOctreeParams(60, 0.25, mthz::Vec3(base_dim.x/2.0, box_height/2.0, base_dim.z/2.0));

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
				mthz::Vec3 pin_pos = mthz::Vec3(start_x + pin_gap * j, pin_start_y + row_verticle_spacing * i, 0);
				phyz::Geometry pin = phyz::Geometry::cylinder(pin_pos,cylinder_radius, base_dim.z)
					.getRotated(mthz::Quaternion(PI/2.0, mthz::Vec3(1, 0, 0)), pin_pos);

				pre_bodies.push_back(BodyHistory(p.createRigidBody(pin, true), pin, color{130, 0, 0}));
			}
		}

		double spinner_y = 23;
		double spinner_radius = effective_width / 4.5;
		
		mthz::Vec3 spinner1_pos = mthz::Vec3(base_dim.y + effective_width/4.0, spinner_y, 0);
		mthz::Vec3 spinner2_pos = mthz::Vec3(base_dim.y + effective_width * 3 / 4.0, spinner_y, 0);
		phyz::Geometry spinner1 = phyz::Geometry::gear(spinner1_pos, cylinder_radius * 2, spinner_radius, base_dim.z, 4)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner1_pos);
		phyz::Geometry spinner2 = phyz::Geometry::gear(spinner2_pos, cylinder_radius * 2, spinner_radius, base_dim.z, 4)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner2_pos);

		double block_start_height = box_height;
		double cube_size = effective_width / (16 * level_of_detail);
		for (int i = 0; i < 8 * level_of_detail; i++) {
			for (int j = 0; j < 56 * level_of_detail; j++) {
				for (int k = 0; k < level_of_detail; k++) {
					mthz::Vec3 pos(base_dim.y + effective_width/2.0 + (i - 4 * level_of_detail) * cube_size, block_start_height + 1.5 * j * cube_size, base_dim.y + k * cube_size);
					phyz::Geometry ball = phyz::Geometry::sphere(pos, cube_size/2.0);
					phyz::RigidBody* r = p.createRigidBody(ball);

					recolor_body_indexes.push_back(pre_bodies.size());
					pre_bodies.push_back(BodyHistory(r, ball));
				}
			}
		}

		phyz::RigidBody* base_r = p.createRigidBody(base, true);
		phyz::RigidBody* negx_wall_r = p.createRigidBody(negx_wall, true);
		phyz::RigidBody* posx_wall_r = p.createRigidBody(posx_wall, true);
		phyz::RigidBody* back_wall_r = p.createRigidBody(back_wall, true);
		phyz::RigidBody* front_wall_r = p.createRigidBody(front_wall, true);
		phyz::RigidBody* spinner1_r = p.createRigidBody(spinner1);
		phyz::RigidBody* spinner2_r = p.createRigidBody(spinner2);

		phyz::ConstraintID spinner1_motor = p.addHingeConstraint(front_wall_r, spinner1_r, spinner1_pos, mthz::Vec3(0, 0, 1));
		p.setMotor(spinner1_motor, 10000000, 0.5);
		phyz::ConstraintID spinner2_motor = p.addHingeConstraint(front_wall_r, spinner2_r, spinner2_pos, mthz::Vec3(0, 0, 1));
		p.setMotor(spinner2_motor, 10000000, -0.5);

		pre_bodies.push_back(BodyHistory(base_r, base));

		pre_bodies.push_back(BodyHistory(negx_wall_r, negx_wall));
		pre_bodies.push_back(BodyHistory(posx_wall_r, posx_wall));
		pre_bodies.push_back(BodyHistory(back_wall_r, back_wall));
		pre_bodies.push_back(BodyHistory(spinner1_r, spinner1, color{ 130, 0, 0 }));
		pre_bodies.push_back(BodyHistory(spinner2_r, spinner2, color{ 130, 0, 0 }));

		mthz::Vec3 cam_pos = mthz::Vec3(4, 8, 16);
		mthz::Quaternion cam_orient;

		double mv_speed = 2;
		double rot_speed = 1;

		p.setGravity(mthz::Vec3(0, -4.9, 0));

		int curr_percent = -1;
		double frame_time = 1 / 120.0;
		int steps_per_frame = 2;
		double simulation_time = 75;
		int n_frames = simulation_time / frame_time + 1;
		
		printf("Checking for existing precomputation...\n");
		char precompute_file[256];
		sprintf_s(precompute_file, "resources/precomputations/precomputation_lod%d.txt", level_of_detail);
		if (fileExists(precompute_file)) {
			printf("File found\n");
			readComputation(precompute_file, &pre_bodies, n_frames);
			for (const BodyHistory& b : pre_bodies) {
				b.r->setOrientation(b.history.back().orient);
				b.r->setToPosition(b.history.back().pos);
			}
		}
		else {
			printf("File not found. Computing...\n");
			int progress_bar_width = 50;
			p.setStep_time(frame_time / steps_per_frame);
			for (BodyHistory& b : pre_bodies) {
				b.history.reserve(simulation_time / frame_time);
				b.history.push_back({ b.r->getPos(), b.r->getOrientation() });
			}

			float physics_time = 0;
			float update_time = 0;
			for (int i = 0; i < n_frames; i++) {
				double t = i * frame_time;
				int new_percent = 100 * t / simulation_time;
				if (new_percent > curr_percent) {
					curr_percent = new_percent;
					render_progress_bar(t / simulation_time, progress_bar_width, false);
				}

				auto t1 = std::chrono::system_clock::now();
				for (int i = 0; i < steps_per_frame; i++) {
					p.timeStep();
				}
				auto t2 = std::chrono::system_clock::now();
				for (BodyHistory& b : pre_bodies) {
					b.history.push_back({ b.r->getPos(), b.r->getOrientation() });
				}
				auto t3 = std::chrono::system_clock::now();
				update_time += std::chrono::duration<float>(t3 - t2).count();
				physics_time += std::chrono::duration<float>(t2 - t1).count();
			}
			render_progress_bar(curr_percent, progress_bar_width, true);
			printf("Saving computation to file...\n");
			writeComputation(precompute_file, pre_bodies, n_frames);
		}

		if (default_coloring) {
			for (int indx : recolor_body_indexes) {
				int col_pick = (pre_bodies[indx].r->getCOM().x - base_dim.y) * 6 / effective_width;

				switch (col_pick) {
				case 0:
					pre_bodies[indx].color = color{ 1.0, 0.0, 0 };
					break;
				case 1:
					pre_bodies[indx].color = color{ 1.0, 0.5, 0 };
					break;
				case 2:
					pre_bodies[indx].color = color{ 1.0, 1.0, 0 };
					break;
				case 3:
					pre_bodies[indx].color = color{ 0.0, 1.0, 0 };
					break;
				case 4:
					pre_bodies[indx].color = color{ 0.0, 0.0, 1.0 };
					break;
				case 5:
					pre_bodies[indx].color = color{ 0.29, 0.0, 0.5 };
					break;
				}
			}
		}
		else {
			olc::Sprite image(image_file);
			for (int indx : recolor_body_indexes) {
				phyz::RigidBody* r = pre_bodies[indx].r;

				mthz::Vec3 com_rel = r->getCOM() - mthz::Vec3(base_dim.y, base_dim.y, 0);
				pre_bodies[indx].color = averagePixels(image, com_rel.x - cube_size / 2.0, effective_width - com_rel.y - cube_size / 2.0, com_rel.x + cube_size / 2.0, effective_width - com_rel.y + cube_size / 2.0, effective_width, effective_width);
			}
		}

		printf("Press enter to start:\n");
		std::string unused;
		std::getline(std::cin, unused);

		rndr::init(properties.window_width, properties.window_height, "Image Demo");

		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		std::vector<PhysBod> bodies;
		for (const BodyHistory& b : pre_bodies) {
			bodies.push_back({ fromGeometry(b.g, b.color), b.r,});
		}

		double t = 0;
		float fElapsedTime;
		int curr_frame = -1;

		while(rndr::render_loop(&fElapsedTime)) {

			t += fElapsedTime;

			int new_frame = t / frame_time;
			if (new_frame != curr_frame && new_frame < n_frames) {
				curr_frame = new_frame;
				for (const BodyHistory& b : pre_bodies) {
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
			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				for (PhysBod b : bodies) {
					delete b.mesh.ib;
					delete b.mesh.va;
				}
				manager->deselectCurrentScene();
				return;
			}

			rndr::clear(rndr::color(0.7f, 0.7f, 0.7f));

			for (const PhysBod& b : bodies) {
				mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
				mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

				shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(cam_pos, cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
				shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0));
				shader.setUniform3f("u_ambient_light", 1.0f, 1.0f, 1.0f);
				shader.setUniform3f("u_pointlight_pos", trnsfm_light_pos.x, trnsfm_light_pos.y, trnsfm_light_pos.z);
				shader.setUniform3f("u_pointlight_col", 0.0, 0.0, 0.0);
				shader.setUniform1i("u_Asleep", false);
				rndr::draw(*b.mesh.va, *b.mesh.ib, shader);

			}
		}
	}
};