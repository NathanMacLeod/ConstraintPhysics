#pragma once
#define OLC_PGE_APPLICATION
#include "DemoScene.h"
#include "../ProgressBar.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include <iostream>
#include "olcPixelGameEngine.h"
#include <inttypes.h>
//#include <io.h>
#include <fcntl.h>
#include <cassert>
#include <chrono>

class ImageDemo : public DemoScene {
private:

	struct PosState {
		mthz::Vec3 pos;
		mthz::Quaternion orient;
	};

	struct BodyHistory {
		BodyHistory(phyz::RigidBody* r, const phyz::ConvexUnionGeometry& g, color c={0.4f, 0.4f, 0.4f}) : r(r), g(g), color(c), ray_hit_count(0) {}

		phyz::RigidBody* r;
		phyz::ConvexUnionGeometry g;
		std::vector<PosState> history;
		color color;
		int ray_hit_count;
	};

	bool fileExists(const std::string& s) {
		std::ifstream f(s);
		return f.good();
	}

	enum GeometryType { BALLS, CUBES, TETRAS, DODECS,  STEL_DODS};
	enum ColorScheme { RAINBOW, IMG_SQR_AVG, IMG_RAYCAST };

	int GeomTypeStackHeight(GeometryType t) {
		switch (t) {
		case BALLS: return 56;
		case CUBES: return 32;
		case TETRAS: return 90;
		case DODECS: return 70;
		case STEL_DODS: return 100;
		default: assert(false); return -1;
		}
	}

	color averagePixels(olc::Sprite& s, double start_x, double start_y, double end_x, double end_y, double dim_x, double dim_y) {
		color average = { 0.0f, 0.0f, 0.0f };
		int n_pixel = 0;

		//0.5 is to check for inclusion of center of pixel, not top left corner

		for (int x = static_cast<int>(start_x * s.width / dim_x + 0.5); x <= static_cast<int>(end_x * s.width / dim_x + 0.5); x++) {
			for (int y = static_cast<int>(start_y * s.height / dim_y + 0.5); y <= static_cast<int>(end_y * s.height / dim_y + 0.5); y++) {
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
			buffer_capacity = read(fd, buffer, sizeof(double) * BUFFER_SIZE) / sizeof(double);
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
		write(fd, buffer, sizeof(double) * buffer_indx);
		buffer_indx = 0;
	}

	void readComputation(std::string filename, std::vector<BodyHistory>* bodies, int n_frames) {
		int fd;
		buffer_indx = 0;
		buffer_capacity = 0;
		fd = open(filename.c_str(), O_RDONLY);

		int progress_bar_width = 50;
		ProgressBarState progress_bar_state{ INITIALIZING };

		for (int i = 0; i < n_frames; i++) {
			progress_bar_state = render_progress_bar((double) i / n_frames, progress_bar_width, false, progress_bar_state, 0.33, true);
			for (BodyHistory& b : *bodies) {
				double x = read64(fd); double y = read64(fd); double z = read64(fd);
				double r = read64(fd); double i = read64(fd); double j = read64(fd); double k = read64(fd);
				b.history.push_back({ mthz::Vec3(x, y, z), mthz::Quaternion(r, i, j, k) });
			}
		}

		render_progress_bar(1.0, progress_bar_width, true, progress_bar_state);
		close(fd);
	}

	void writeComputation(std::string filename, const std::vector<BodyHistory>& bodies, int n_frames) {
		int fd;
		buffer_indx = 0;
		fd = open(filename.c_str(), O_CREAT);
		for (int i = 0; i < n_frames; i++) {
			for (const BodyHistory& b : bodies) {
				PosState p = b.history[i];
				write64(fd, p.pos.x); write64(fd, p.pos.y); write64(fd, p.pos.z);
				write64(fd, p.orient.r); write64(fd, p.orient.i); write64(fd, p.orient.j); write64(fd, p.orient.k);
			}
		}
		flush(fd);
		close(fd);
	}

public:
	ImageDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~ImageDemo() override {

	}

	std::map<std::string, std::string> askParameters() override {
		std::map<std::string, std::string> out;
		
		out["color_pattern"] = pickParameterFromOptions(
			"Select coloring pattern. D (Default rainbow), A (Image file, averging color of a square around object), R (Image file, painting objects using raycast): ",
			{"D", "A", "R"}
		);

		if (caseIndefferentStringEquals(out["color_pattern"], "A") || caseIndefferentStringEquals(out["color_pattern"], "R")) {
			out["filepath"] = askCustomParameterValue(
				"Give filepath of the image to draw: ", [=](std::string s) -> bool { return fileExists(s); }, "File not found, enter again: "
			);
		}

		out["level_of_detail"] = pickParameterFromOptions(
			"Select level of detail. Note computation time scales massively. Options are 1, 2, 3: ", { "1", "2", "3" }
		);

		out["shape"] = pickParameterFromOptions(
			"Select the shapes to use in the simulation. Options are B (Balls), C (Cubes), T (Tetrahedrons!), D (Dodecahedrons!), S (Stellated Dodecahedrons!!) (! = expensive): ",
			{"B", "C", "T", "D", "S"}
		);

		return out;
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

		GeometryType geometry_type;
		if      (caseIndefferentStringEquals(parameters["shape"], "B")) geometry_type = BALLS;
		else if (caseIndefferentStringEquals(parameters["shape"], "C")) geometry_type = CUBES;
		else if (caseIndefferentStringEquals(parameters["shape"], "T")) geometry_type = TETRAS;
		else if (caseIndefferentStringEquals(parameters["shape"], "D")) geometry_type = DODECS;
		else if (caseIndefferentStringEquals(parameters["shape"], "S")) geometry_type = STEL_DODS;
		else assert(false);

		ColorScheme color_scheme;
		if      (caseIndefferentStringEquals(parameters["color_pattern"], "D")) color_scheme = RAINBOW;
		else if (caseIndefferentStringEquals(parameters["color_pattern"], "A")) color_scheme = IMG_SQR_AVG;
		else if (caseIndefferentStringEquals(parameters["color_pattern"], "R")) color_scheme = IMG_RAYCAST;
		else assert(false);

		int level_of_detail = atoi(parameters["level_of_detail"].c_str());

		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(false);
		p.setPGSIterations(45, 35);
		p.setAABBTreeMarginSize(0.05);
		p.setBroadphase(phyz::AABB_TREE);
		p.setPrintPerformanceData(false);

		bool paused = false;
		bool slow = false;
		bool lock_cam = true;

		std::vector<BodyHistory> pre_bodies;
		std::vector<size_t> recolor_body_indexes;

		mthz::Vec3 base_dim(8.5, 0.25, 0.75);
		phyz::ConvexUnionGeometry base = phyz::ConvexUnionGeometry::box(mthz::Vec3(), base_dim.x, base_dim.y, base_dim.z);
		double box_height = 48;
		phyz::ConvexUnionGeometry negx_wall = phyz::ConvexUnionGeometry::box(mthz::Vec3(0, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
		phyz::ConvexUnionGeometry posx_wall = phyz::ConvexUnionGeometry::box(mthz::Vec3(base_dim.x - base_dim.y, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
		phyz::ConvexUnionGeometry back_wall = phyz::ConvexUnionGeometry::box(mthz::Vec3(0, 0, 0), base_dim.x, box_height + base_dim.y, -base_dim.y, phyz::Material::ice());
		phyz::ConvexUnionGeometry front_wall = phyz::ConvexUnionGeometry::box(mthz::Vec3(0, 0, base_dim.z), base_dim.x, base_dim.y + box_height, base_dim.y, phyz::Material::ice());

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
				phyz::ConvexUnionGeometry pin = phyz::ConvexUnionGeometry::cylinder(pin_pos,cylinder_radius, base_dim.z)
					.getRotated(mthz::Quaternion(PI/2.0, mthz::Vec3(1, 0, 0)), pin_pos);

				pre_bodies.push_back(BodyHistory(p.createRigidBody(pin, phyz::RigidBody::FIXED), pin, color{130, 0, 0}));
			}
		}

		double spinner_y = 23;
		double spinner_radius = effective_width / 4.5;
		
		mthz::Vec3 spinner1_pos = mthz::Vec3(base_dim.y + effective_width/4.0, spinner_y, 0);
		mthz::Vec3 spinner2_pos = mthz::Vec3(base_dim.y + effective_width * 3 / 4.0, spinner_y, 0);
		double spinner_density = 10000;
		phyz::ConvexUnionGeometry spinner1 = phyz::ConvexUnionGeometry::gear(spinner1_pos, cylinder_radius * 2, spinner_radius, base_dim.z, 4, false, phyz::Material::modified_density(spinner_density))
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner1_pos);
		phyz::ConvexUnionGeometry spinner2 = phyz::ConvexUnionGeometry::gear(spinner2_pos, cylinder_radius * 2, spinner_radius, base_dim.z, 4, false, phyz::Material::modified_density(spinner_density))
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner2_pos);

		double block_start_height = box_height;
		double cube_size = effective_width / (16 * level_of_detail);

		phyz::ConvexUnionGeometry stellated_dodecahedron_shape;
		if (geometry_type == STEL_DODS) {
			stellated_dodecahedron_shape = phyz::ConvexUnionGeometry::stellatedDodecahedron(mthz::Vec3(), 1, 0.7);
			std::vector<phyz::AABB> aabbs;
			for (const phyz::ConvexPrimitive& p : stellated_dodecahedron_shape.getPolyhedra()) {
				aabbs.push_back(p.gen_AABB());
			}
			phyz::AABB big_aabb = phyz::AABB::combine(aabbs);
			double real_size = std::max<double>(big_aabb.max.x - big_aabb.min.x, std::max<double>(big_aabb.max.y - big_aabb.min.y, big_aabb.max.z - big_aabb.min.z));

			stellated_dodecahedron_shape = stellated_dodecahedron_shape.getScaled(cube_size / real_size);
		}

		for (int i = 0; i < 8 * level_of_detail; i++) {
			for (int j = 0; j < GeomTypeStackHeight(geometry_type) * level_of_detail; j++) {
				for (int k = 0; k < level_of_detail; k++) {
					mthz::Vec3 pos(base_dim.y + effective_width/2.0 + (i - 4 * level_of_detail) * cube_size, block_start_height + 1.5 * j * cube_size, base_dim.y + k * cube_size);
					phyz::ConvexUnionGeometry g;
					switch (geometry_type) {
					case BALLS:
						//low friction to ease issues
						g = phyz::ConvexUnionGeometry::sphere(pos, cube_size / 2.0, phyz::Material{ phyz::CFM{phyz::USE_GLOBAL}, 1.0, 0.3, 0.5, 0.8 });
						break;
					case CUBES:
						g = phyz::ConvexUnionGeometry::box(pos - 0.5 * mthz::Vec3(cube_size, cube_size, cube_size), cube_size, cube_size, cube_size);
						break;
					case TETRAS:
					{
						mthz::Vec3 p1 = pos - 0.5 * mthz::Vec3(cube_size, cube_size, cube_size);
						mthz::Vec3 p2 = p1 + mthz::Vec3(cube_size, 0, cube_size);
						mthz::Vec3 p3 = p1 + mthz::Vec3(0, cube_size, cube_size);
						mthz::Vec3 p4 = p1 + mthz::Vec3(cube_size, cube_size, 0);
						g = phyz::ConvexUnionGeometry::tetra(p1, p2, p3, p4);
					}
					break;
					case DODECS:
						g = phyz::ConvexUnionGeometry::regDodecahedron(pos, cube_size);
						break;
					case STEL_DODS:
						g = stellated_dodecahedron_shape.getTranslated(pos);
						break;
					}

					phyz::RigidBody* r = p.createRigidBody(g);

					recolor_body_indexes.push_back(pre_bodies.size());
					pre_bodies.push_back(BodyHistory(r, g));
				}
			}
		}

		phyz::RigidBody* base_r = p.createRigidBody(base, phyz::RigidBody::FIXED);
		phyz::RigidBody* negx_wall_r = p.createRigidBody(negx_wall, phyz::RigidBody::FIXED);
		phyz::RigidBody* posx_wall_r = p.createRigidBody(posx_wall, phyz::RigidBody::FIXED);
		phyz::RigidBody* back_wall_r = p.createRigidBody(back_wall, phyz::RigidBody::FIXED);
		phyz::RigidBody* front_wall_r = p.createRigidBody(front_wall, phyz::RigidBody::FIXED);

		phyz::RigidBody* spinner1_r = p.createRigidBody(spinner1);
		phyz::RigidBody* spinner2_r = p.createRigidBody(spinner2);

		phyz::ConstraintID spinner1_motor = p.addMotorConstraint(p.addHingeConstraint(front_wall_r, spinner1_r, spinner1_pos, mthz::Vec3(0, 0, 1)));
		p.setMotorTargetVelocity(spinner1_motor, 100000000000, 0.5);
		phyz::ConstraintID spinner2_motor = p.addMotorConstraint(p.addHingeConstraint(front_wall_r, spinner2_r, spinner2_pos, mthz::Vec3(0, 0, 1)));
		p.setMotorTargetVelocity(spinner2_motor, 100000000000, -0.5);

		p.disallowCollision(spinner1_r, negx_wall_r);
		p.disallowCollision(spinner2_r, posx_wall_r);

		pre_bodies.push_back(BodyHistory(base_r, base));

		pre_bodies.push_back(BodyHistory(negx_wall_r, negx_wall));
		pre_bodies.push_back(BodyHistory(posx_wall_r, posx_wall));
		pre_bodies.push_back(BodyHistory(back_wall_r, back_wall));
		pre_bodies.push_back(BodyHistory(spinner1_r, spinner1, color{ 130, 0, 0 }));
		pre_bodies.push_back(BodyHistory(spinner2_r, spinner2, color{ 130, 0, 0 }));

		mthz::Vec3 cam_pos = mthz::Vec3(4.25, 12, 22);//mthz::Vec3(4, 13, 24);//mthz::Vec3(9.441140, 20.246, 5.9803);
		mthz::Quaternion cam_orient;// = mthz::Quaternion(0.9217, 0.214225, 0.31549, -0.071306).normalize();

		double mv_speed = 2;
		double rot_speed = 1;

		p.setGravity(mthz::Vec3(0, -4.9, 0));

		ProgressBarState progress_bar_state = { INITIALIZING };
		double frame_time = 1 / 120.0;
		int steps_per_frame = 2;
		double simulation_time = 75;
		int n_frames = static_cast<int>(simulation_time / frame_time) + 1;
		int progress_bar_width = 50;

		printf("Checking for existing precomputation...\n");
		char precompute_file[256];
		sprintf(precompute_file, "resources/precomputations/precomputation_lod%d_%c.txt", level_of_detail, toupper(parameters["shape"][0]));
		if (fileExists(precompute_file)) {
			printf("File found. Loading...\n");
			readComputation(precompute_file, &pre_bodies, n_frames);
			for (const BodyHistory& b : pre_bodies) {
				b.r->setOrientation(b.history.back().orient);
				b.r->setToPosition(b.history.back().pos);
			}
		}
		else {
			printf("File not found. Computing...\n");
			
			p.setStep_time(frame_time / steps_per_frame);
			for (BodyHistory& b : pre_bodies) {
				b.history.reserve(static_cast<size_t>(simulation_time / frame_time));
				b.history.push_back({ b.r->getPos(), b.r->getOrientation() });
			}

			float physics_time = 0;
			float update_time = 0;

			for (int i = 0; i < n_frames; i++) {
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

				progress_bar_state = render_progress_bar(i * frame_time / simulation_time, progress_bar_width, false, progress_bar_state, 0.33, true);
				
			}
			render_progress_bar(1.0, progress_bar_width, true, progress_bar_state);
			printf("Saving computation to file...\n");
			writeComputation(precompute_file, pre_bodies, n_frames);
		}

		switch(color_scheme) {
		case RAINBOW:
		{
			for (size_t indx : recolor_body_indexes) {
				int col_pick = static_cast<int>((pre_bodies[indx].r->getCOM().x - base_dim.y) * 6 / effective_width);

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
					pre_bodies[indx].color = color{ 0.29f, 0.0f, 0.5f };
					break;
				}
			}
		}
		break;
		case IMG_SQR_AVG:
		{
			olc::Sprite image(parameters["filepath"]);
			for (size_t indx : recolor_body_indexes) {
				phyz::RigidBody* r = pre_bodies[indx].r;

				mthz::Vec3 com_rel = r->getCOM() - mthz::Vec3(base_dim.y, base_dim.y, 0);
				pre_bodies[indx].color = averagePixels(image, com_rel.x - cube_size / 2.0, effective_width - com_rel.y - cube_size / 2.0, com_rel.x + cube_size / 2.0, effective_width - com_rel.y + cube_size / 2.0, effective_width, effective_width);
			}
		}
		break;
		case IMG_RAYCAST:
		{
			p.forceAABBTreeUpdate();
			std::unordered_map<phyz::RigidBody*, BodyHistory*> body_map;
			for (size_t indx : recolor_body_indexes) {
				body_map[pre_bodies[indx].r] = &pre_bodies[indx];
			}

			olc::Sprite image(parameters["filepath"]);

			printf("Computing Raycasts...\n");

			progress_bar_state = ProgressBarState{ INITIALIZING };
			double canvas_min_x = base_dim.y;
			double canvas_min_y = base_dim.y;
			double canvas_width = effective_width;
			double canvas_height = effective_width;
			
			double pixel_width = canvas_width / image.width;
			double pixel_height = canvas_height / image.height;

			int total_pixel_count = image.width * image.height;

			for (int px = 0; px < image.width; px++) {
				for (int py = 0; py < image.height; py++) {


					progress_bar_state = render_progress_bar((px * image.height + py) / (float) total_pixel_count, progress_bar_width, false, progress_bar_state, 0.33, true);

					olc::Pixel color = image.GetPixel(px, py);

					int pixel_substep_count = 4;
					for (int sx = 0; sx < pixel_substep_count; sx++) {
						for (int sy = 0; sy < pixel_substep_count; sy++) {

							double ray_origin_x = canvas_min_x + (px + (sx + 1) / (pixel_substep_count + 1)) * pixel_width;
							double ray_origin_y = canvas_min_y + canvas_height - (py + (sy + 1) / (pixel_substep_count + 1)) * pixel_height;

							double ray_origin_z = 5.0;
							mthz::Vec3 ray_dir(0, 0, -1);
							mthz::Vec3 ray_origin(ray_origin_x, ray_origin_y, ray_origin_z);

							phyz::RayHitInfo hit = p.raycastFirstIntersection(ray_origin, ray_dir, { front_wall_r });
							if (hit.did_hit) {
								if (body_map.find(hit.hit_object) != body_map.end()) {
									BodyHistory* hit_body = body_map[hit.hit_object];

									hit_body->color.r += color.r / 255.0f;
									hit_body->color.g += color.g / 255.0f;
									hit_body->color.b += color.b / 255.0f;

									hit_body->ray_hit_count++;
								}
							}
						}
					}
				}
			}

			render_progress_bar(1.0, progress_bar_width, true, progress_bar_state);

			for (size_t indx : recolor_body_indexes) {
				if (pre_bodies[indx].ray_hit_count == 0) {
					phyz::RigidBody* r = pre_bodies[indx].r;

					mthz::Vec3 com_rel = r->getCOM() - mthz::Vec3(base_dim.y, base_dim.y, 0);
					pre_bodies[indx].color = averagePixels(image, com_rel.x - cube_size / 2.0, effective_width - com_rel.y - cube_size / 2.0, com_rel.x + cube_size / 2.0, effective_width - com_rel.y + cube_size / 2.0, effective_width, effective_width);
				}
				else {
					pre_bodies[indx].color.r /= pre_bodies[indx].ray_hit_count;
					pre_bodies[indx].color.g /= pre_bodies[indx].ray_hit_count;
					pre_bodies[indx].color.b /= pre_bodies[indx].ray_hit_count;
				}
			}
		}
		break;
		}

		printf("Press enter to start:\n");
		std::string unused;
		std::getline(std::cin, unused);

		rndr::init(properties.window_width, properties.window_height, "Image Demo");

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		std::vector<PhysBod> bodies;
		for (const BodyHistory& b : pre_bodies) {
			bodies.push_back({ fromGeometry(b.g, b.color), b.r,});
		}

		double t = 0;
		float fElapsedTime;
		int curr_frame = -1;

		phyz::ThreadManager helper_threads;
		if (properties.n_threads != 0) {
			helper_threads.init(properties.n_threads);
		}

		std::vector<Mesh> transformed_meshs(bodies.size());
		for (int i = 0; i < bodies.size(); i++) {
			transformed_meshs[i] = bodies[i].mesh;
		}

		while(rndr::render_loop(&fElapsedTime)) {

			t += fElapsedTime;

			int new_frame = static_cast<int>(t / frame_time);
			if (new_frame != curr_frame && new_frame < n_frames) {
				curr_frame = new_frame;
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

			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				p.forceAABBTreeUpdate();

				mthz::Vec3 camera_dir = cam_orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::RayHitInfo hit_info = p.raycastFirstIntersection(cam_pos, camera_dir, { front_wall_r });

				if (hit_info.did_hit) {
					for (size_t indx : recolor_body_indexes) {
						if (pre_bodies[indx].r == hit_info.hit_object) {
							printf("Hit index: %d\n", static_cast<uint32_t>(indx));
						}
					}
				}
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
				manager->deselectCurrentScene();
				return;
			}

			rndr::clear(rndr::color(0.7f, 0.7f, 0.7f));
			batch_array.flush();


			mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

			float aspect_ratio = (float)properties.window_height / properties.window_width;
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 50.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
			shader.setUniform3f("u_ambient_light", 1.0f, 1.0f, 1.0f);
			shader.setUniform3f("u_pointlight_pos", static_cast<float>(trnsfm_light_pos.x), static_cast<float>(trnsfm_light_pos.y), static_cast<float>(trnsfm_light_pos.z));
			shader.setUniform3f("u_pointlight_col", 0.0f, 0.0f, 0.0f);
			shader.setUniform1i("u_Asleep", false);

			auto setTransformedMatrix = [&](int indx) {
				const BodyHistory& b = pre_bodies[indx];
				writeTransformedTo(bodies[indx].mesh, &transformed_meshs[indx], b.history[curr_frame].pos, b.history[curr_frame].orient, cam_pos, cam_orient, false, color{1.0, 0.0, 0.0});
			};

			if (properties.n_threads == 0) {
				for (int i = 0; i < bodies.size(); i++) {
					setTransformedMatrix(i);
				}
			}
			else {
				std::vector<int> indexes(bodies.size());
				for (int i = 0; i < bodies.size(); i++) {
					indexes[i] = i;
				}
				helper_threads.do_all<int>(properties.n_threads, indexes, setTransformedMatrix);
			}

			for (Mesh& m : transformed_meshs) {
				if (batch_array.remainingVertexCapacity() <= m.vertices.size() || batch_array.remainingIndexCapacity() < m.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}

				batch_array.push(m.vertices.data(), static_cast<int>(m.vertices.size()), m.indices);
			}

			rndr::draw(batch_array, shader);
		}
	}
};