#pragma once
#include "DemoScene.h"
#include "../ProgressBar.h"
#include "../Mesh.h"
#include "olcPixelGameEngine.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

#include <iostream>

class VideoDemo : public DemoScene {
public:
	VideoDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~VideoDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"I. K", "Raise, Lower crane arm"},
			ControlDescription{"J, L", "Rotate crane counter-clockwise, clockwise"},
			ControlDescription{"R", "Reset tower"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	bool fileExists(const std::string& s) {
		std::ifstream f(s);
		return f.good();
	}

	static const int BUFFER_SIZE = 512 * 8;
	int buffer_indx;
	int buffer_capacity = 0;
	unsigned char buffer[BUFFER_SIZE];

	unsigned char read8(int fd) {
		if (buffer_indx >= buffer_capacity) {
			buffer_indx = 0;
			buffer_capacity = _read(fd, buffer, sizeof(unsigned char) * BUFFER_SIZE) / sizeof(unsigned char);
			assert(buffer_capacity != 0);
		}
		return buffer[buffer_indx++];
	}

	void write8(int fd, unsigned char v) {
		if (buffer_indx >= BUFFER_SIZE) {
			flush(fd);
		}
		buffer[buffer_indx++] = v;
	}

	void flush(int fd) {
		_write(fd, buffer, sizeof(unsigned char) * buffer_indx);
		buffer_indx = 0;
	}

	void readComputation(std::string filename, std::vector<color>* colors) {
		int fd;
		buffer_indx = 0;
		buffer_capacity = 0;
		_sopen_s(&fd, filename.c_str(), _O_BINARY | _O_RDONLY, _SH_DENYRW, _S_IREAD);

		int progress_bar_width = 50;
		ProgressBarState progress_bar_state{ INITIALIZING };

		int n_spheres;
		int i1 = read8(fd);
		int i2 = read8(fd);
		int i3 = read8(fd);
		int i4 = read8(fd);
		n_spheres = (0xFF000000 & (i4 << 24)) | (0x00FF0000 & (i3 << 16)) | (0x0000FF00 & (i2 << 8)) | (0x000000FF & i1);

		for (int i = 0; i < n_spheres; i++) {
			progress_bar_state = render_progress_bar(i / (float)n_spheres, progress_bar_width, false, progress_bar_state, 0.33, true);
			colors->push_back(color{ read8(fd) / 255.0f, read8(fd) / 255.0f, read8(fd) / 255.0f });
		}

		render_progress_bar(1.0, progress_bar_width, true, progress_bar_state);
		_close(fd);
	}

	void writeComputation(std::string filename, const std::vector<color>& colors) {
		int fd;
		buffer_indx = 0;
		_sopen_s(&fd, filename.c_str(), _O_CREAT | _O_BINARY | _O_WRONLY, _SH_DENYRW, _S_IWRITE);

		uint32_t n_spheres = static_cast<uint32_t>(colors.size());
		write8(fd,  0x000000FF & n_spheres);
		write8(fd, (0x0000FF00 & n_spheres) >> 8);
		write8(fd, (0x00FF0000 & n_spheres) >> 16);
		write8(fd, (0xFF000000 & n_spheres) >> 24);

		for (color c : colors) {
			write8(fd, static_cast<uint8_t>(c.r * 255.0f));
			write8(fd, static_cast<uint8_t>(c.g * 255.0f));
			write8(fd, static_cast<uint8_t>(c.b * 255.0f));
		}
		flush(fd);
		_close(fd);
	}

	olc::Sprite getFrame(int frame_count) {
		char url[256];
		sprintf_s(url, "resources/bad_apple/bad_apple_frames/bad_apple_frame%d.png", frame_count);

		return olc::Sprite(url);
	}

	struct ColorHitInfo {
		color color_sum;
		int hit_count = 0;
	};

	std::string message =  "You will need the frames of Bad Apple to generate the precomputation. For size reasons, the frames are not included in the repository.\n"
		"a zip of the files can be downloaded from here:\n"
		"https://drive.google.com/file/d/1shzoR-C-iM0UoIdzUDJGmQgXZ7tUFtMA/view?usp=sharing\n\n"
		"Simply unzip the contents into the directory Demo/resources/precomputations/bad_apple\n"
		"Such that the file path of each frame is Demo/resources/bad_apple/bad_apple_frames/bad_apple_frame_i.png\n";

	void run() override {

		printf("%s", message.c_str());

		printf("\nPress any key to continue: \n");
		std::getchar();

		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::ThreadManager helper_threads;
		if (properties.n_threads != 0) {
			helper_threads.init(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(false); //never going to happen anyway
		p.setPGSIterations(3, 1);
		std::string precomputation_file = "resources/precomputations/bad_apple_colors.txt";
		bool precompute_colors_mode = !fileExists(precomputation_file);

		bool lock_cam = true;
		
		color yellow = { 249 / 255.0f, 186 / 255.0f, 50 / 255.0f };
		color blue = { 66 / 255.0f, 110 / 255.0f, 134 / 255.0f };
		color orange = { 240 / 255.0f, 129 / 255.0f, 15 / 255.0f };

		std::vector<TransformablePhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;
		
		double ball_radius = 0.07;

		double funnel_radius = 16;
		double funnel_tube_height = 2.5;
		double funne_tube_radius = 6 * ball_radius;
		mthz::Vec3 funnel_pos = mthz::Vec3(0, 0, 0);
		phyz::ConvexUnionGeometry funnel = phyz::ConvexUnionGeometry::funnel(mthz::Vec3(), 5 * ball_radius, funnel_tube_height, funnel_radius, PI / 2 * 0.8, 0.15, 30);

		double dropper_height = 6.2;
		double dropper_radius = 4;
		double dropper_ramp_radius = 2 * ball_radius;
		double dropper_ramp_length = 12;
		double dropper_ramp_angle = 0.18 * PI / 2;
		double dropper_ramp_offset = 4;

		double dropper_height_above_ramp = 0.6;
		double dropper_ramp_gap = 0.33;
		double dropper_extruder_radius = ball_radius * 2;
		double dropper_extruder_height = 0.5;
		double dropper_extruder_thickness = 0.05;
		double dropper_extruder_cube_size = 0.5;
			
		mthz::Vec3 dropper_ramp_pos = funnel_pos + mthz::Vec3(dropper_radius, dropper_height, dropper_ramp_offset);
		phyz::ConvexUnionGeometry dropper_ramp = phyz::ConvexUnionGeometry::uShape(dropper_ramp_pos, dropper_ramp_radius, 2 * dropper_ramp_radius, dropper_ramp_length).getRotated(mthz::Quaternion(PI / 2.0 - dropper_ramp_angle, mthz::Vec3(1, 0, 0)), dropper_ramp_pos);

		mthz::Vec3 dropper_extruder_pos = dropper_ramp_pos + mthz::Vec3(0, dropper_height_above_ramp + (dropper_ramp_length - dropper_ramp_gap) * sin(dropper_ramp_angle), (dropper_ramp_length - dropper_ramp_gap) * cos(dropper_ramp_angle));
		phyz::ConvexUnionGeometry dropper_extruder_tube = phyz::ConvexUnionGeometry::ring(dropper_extruder_pos, dropper_extruder_radius, dropper_extruder_radius + dropper_extruder_thickness, dropper_extruder_height);
		phyz::ConvexUnionGeometry dropper_extruder_cube = phyz::ConvexUnionGeometry::box(dropper_extruder_pos + mthz::Vec3(-dropper_extruder_cube_size / 2.0, dropper_extruder_height, -dropper_extruder_cube_size / 2.0), dropper_extruder_cube_size, dropper_extruder_cube_size, dropper_extruder_cube_size);
		phyz::ConvexUnionGeometry extruder = { dropper_extruder_tube, dropper_extruder_cube };

		std::vector<mthz::Vec3> dropper_spawn_positions;
		int n_droppers = 10;
		for (int i = 0; i < n_droppers; i++) {
			mthz::Quaternion rot = mthz::Quaternion(2 * PI * i / n_droppers, mthz::Vec3(0, 1, 0));
			phyz::ConvexUnionGeometry dropper_ramp_rotated = dropper_ramp.getRotated(rot, funnel_pos);
			phyz::ConvexUnionGeometry extruder_rotated = extruder.getRotated(rot, funnel_pos);

			bodies.push_back(TransformablePhysBod(fromGeometry(dropper_ramp_rotated, blue), p.createRigidBody(dropper_ramp_rotated, phyz::RigidBody::FIXED)));
			bodies.push_back(TransformablePhysBod(fromGeometry(extruder_rotated, yellow), p.createRigidBody(extruder_rotated, phyz::RigidBody::FIXED)));

			mthz::Vec3 ball_spawn_pos = rot.applyRotation(dropper_extruder_pos + mthz::Vec3(0, dropper_extruder_height / 2.0, 0));
			dropper_spawn_positions.push_back(ball_spawn_pos);
		}

		double bucket_funnel_gap = 1;
		double bucket_radius = 7.33 * ball_radius;
		double bucket_height = 28 * ball_radius;
		double bucket_thickness = 0.1;
		double bucket_bottom_density = 6.5 * bucket_height;

		mthz::Vec3 bucket_position = funnel_pos + mthz::Vec3(0, -bucket_funnel_gap - bucket_height - bucket_thickness, 0);
		phyz::ConvexUnionGeometry bucket_floor = phyz::ConvexUnionGeometry::polyCylinder(bucket_position, bucket_radius + bucket_thickness, bucket_thickness, 10, phyz::Material::modified_density(bucket_bottom_density));
		phyz::ConvexUnionGeometry bucket_wall = phyz::ConvexUnionGeometry::ring(bucket_position + mthz::Vec3(0, bucket_thickness, 0), bucket_radius, bucket_radius + bucket_thickness, bucket_height, 10);
		phyz::ConvexUnionGeometry bucket = { bucket_floor, bucket_wall };

		double bucket_pivot_height = bucket_height / 4.0;
		double bucket_support_block_width = bucket_radius;
		double bucket_support_block_height = 1.5 * bucket_support_block_width;
		double bucket_support_block_thickness = bucket_support_block_width / 2.0;
		double bucket_support_gap = 0.01;
		double bucket_rest_angle = PI * 0.6;

		mthz::Vec3 bucket_pivot_pos = bucket_position + mthz::Vec3(0, bucket_pivot_height, 0);


		phyz::ConvexUnionGeometry bucket_support_block1 = phyz::ConvexUnionGeometry::box(bucket_pivot_pos + mthz::Vec3(-bucket_radius - bucket_thickness - bucket_support_gap - bucket_support_block_thickness, -bucket_support_block_height + bucket_support_block_width / 2.0, -bucket_support_block_width / 2.0),
			bucket_support_block_thickness, bucket_support_block_height, bucket_support_block_width);
		phyz::ConvexUnionGeometry bucket_support_block2 = phyz::ConvexUnionGeometry::box(bucket_pivot_pos + mthz::Vec3(bucket_radius + bucket_thickness + bucket_support_gap, -bucket_support_block_height + bucket_support_block_width / 2.0, -bucket_support_block_width / 2.0),
			bucket_support_block_thickness, bucket_support_block_height, bucket_support_block_width);

		phyz::ConvexUnionGeometry bucket_support_block = { bucket_support_block1, bucket_support_block2 };

		double bucket_rest_rod_radius = 0.1;
		double bucket_rest_rod_length = 3 * bucket_radius;
		mthz::Vec3 bucket_rest_rod_pos = bucket_position + mthz::Vec3(-bucket_rest_rod_length / 2.0, bucket_height * 0.75, bucket_radius + bucket_thickness + bucket_rest_rod_radius);

		phyz::ConvexUnionGeometry bucket_rest_rod = phyz::ConvexUnionGeometry::polyCylinder(bucket_rest_rod_pos, bucket_rest_rod_radius, bucket_rest_rod_length).getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1)), bucket_rest_rod_pos)
			.getRotated(mthz::Quaternion(bucket_rest_angle, mthz::Vec3(1, 0, 0)), bucket_pivot_pos);

		double ramp_length = 16;
		double ramp_width = 30 * ball_radius;
		double ramp_thickness = 0.1;
		double ramp_guard_height = 0.1;
		double ramp_angle = 0.2 * PI / 2;
		double ramp_height_under_bucket = 1.0;

		mthz::Vec3 ramp_position = bucket_position + mthz::Vec3(-ramp_width / 2.0, -ramp_height_under_bucket, 0);

		phyz::ConvexUnionGeometry ramp_base = phyz::ConvexUnionGeometry::box(ramp_position, ramp_width, ramp_thickness, ramp_length);
		phyz::ConvexUnionGeometry ramp_guard1 = phyz::ConvexUnionGeometry::box(ramp_position + mthz::Vec3(-ramp_thickness, 0, 0), ramp_thickness, ramp_thickness + ramp_guard_height, ramp_length);
		phyz::ConvexUnionGeometry ramp_guard2 = phyz::ConvexUnionGeometry::box(ramp_position + mthz::Vec3(ramp_width, 0, 0), ramp_thickness, ramp_thickness + ramp_guard_height, ramp_length);
		phyz::ConvexUnionGeometry ramp = { ramp_base, ramp_guard1, ramp_guard2 };
		ramp = ramp.getRotated(mthz::Quaternion(ramp_angle, mthz::Vec3(1, 0, 0)), ramp_position);

		double display_case_thickness = 0.1;
		double display_gap = 4 * ball_radius;
		double display_guard_height = 0.5;
		double display_height = 50 * ball_radius;
		double bottom_density = 11500;
		mthz::Vec3 display_position = ramp_position + mthz::Vec3(0, -ramp_length * sin(ramp_angle) - display_height, ramp_length * cos(ramp_angle) + ramp_thickness * sin(ramp_angle));

		phyz::ConvexUnionGeometry display_rear_wall = phyz::ConvexUnionGeometry::box(display_position + mthz::Vec3(0, 0, -display_case_thickness), ramp_width, display_height, ramp_thickness);
		phyz::ConvexUnionGeometry display_negx_wall = phyz::ConvexUnionGeometry::box(display_position + mthz::Vec3(-display_case_thickness, 0, -display_case_thickness), display_case_thickness, display_height + display_guard_height, 2 * display_case_thickness + display_gap);
		phyz::ConvexUnionGeometry display_posx_wall = phyz::ConvexUnionGeometry::box(display_position + mthz::Vec3(ramp_width, 0, -display_case_thickness), display_case_thickness, display_height + display_guard_height, 2 * display_case_thickness + display_gap);
		phyz::ConvexUnionGeometry display_front_wall = phyz::ConvexUnionGeometry::box(display_position + mthz::Vec3(0, 0, display_gap), ramp_width, display_height, ramp_thickness);
		phyz::ConvexUnionGeometry display_front_guard = phyz::ConvexUnionGeometry::box(display_position + mthz::Vec3(0, display_height, display_gap), ramp_width, display_guard_height, ramp_thickness);
		phyz::ConvexUnionGeometry display_bottom = phyz::ConvexUnionGeometry::box(display_position + mthz::Vec3(-display_case_thickness, -display_case_thickness, -display_case_thickness), 2 * display_case_thickness + ramp_width, display_case_thickness, 2 * display_case_thickness + display_gap, phyz::Material::modified_density(bottom_density));

		phyz::RigidBody* display_rear_wall_r = p.createRigidBody(display_rear_wall, phyz::RigidBody::FIXED);
		bodies.push_back(TransformablePhysBod(fromGeometry(display_rear_wall, blue), display_rear_wall_r));

		phyz::RigidBody* display_posx_wall_r = p.createRigidBody(display_posx_wall, phyz::RigidBody::FIXED);
		bodies.push_back(TransformablePhysBod(fromGeometry(display_posx_wall, blue), display_posx_wall_r));

		phyz::RigidBody* display_negx_wall_r = p.createRigidBody(display_negx_wall, phyz::RigidBody::FIXED);
		bodies.push_back(TransformablePhysBod(fromGeometry(display_negx_wall, blue), display_negx_wall_r));

		phyz::RigidBody* display_front_wall_r = p.createRigidBody(display_front_wall, phyz::RigidBody::FIXED);

		phyz::RigidBody* display_front_guard_r = p.createRigidBody(display_front_guard, phyz::RigidBody::FIXED);
		bodies.push_back(TransformablePhysBod(fromGeometry(display_front_guard, blue), display_front_guard_r));

		phyz::RigidBody* display_bottom_r = p.createRigidBody(display_bottom);
		bodies.push_back(TransformablePhysBod(fromGeometry(display_bottom, blue), display_bottom_r));

		p.disallowCollision(display_bottom_r, display_posx_wall_r);
		p.disallowCollision(display_bottom_r, display_negx_wall_r);
		phyz::ConstraintID bottom_hinge = p.addHingeConstraint(display_rear_wall_r, display_bottom_r, display_position, mthz::Vec3(1, 0, 0));
		p.setMotorTargetPosition(bottom_hinge, 10000, 0);

		phyz::RigidBody* ramp_r = p.createRigidBody(ramp, phyz::RigidBody::FIXED);
		bodies.push_back(TransformablePhysBod(fromGeometry(ramp, blue), ramp_r));

		phyz::RigidBody* bucket_r = p.createRigidBody(bucket);
		phyz::RigidBody* bucket_support_r = p.createRigidBody(bucket_support_block, phyz::RigidBody::FIXED);

		bucket_r->setSleepDisabled(true);
		p.addHingeConstraint(bucket_r, bucket_support_r, bucket_pivot_pos, mthz::Vec3(1, 0, 0), PI * 0.03);

		bodies.push_back(TransformablePhysBod(fromGeometry(bucket, blue), bucket_r));
		bodies.push_back(TransformablePhysBod(fromGeometry(bucket_support_block, yellow), bucket_support_r));

		phyz::RigidBody* bucket_rest_rod_r = p.createRigidBody(bucket_rest_rod, phyz::RigidBody::FIXED);
		bodies.push_back({ fromGeometry(bucket_rest_rod, yellow), bucket_rest_rod_r });
		
		double delete_box_height = -50;
		double delete_box_dim = 10000;
		phyz::ConvexUnionGeometry delete_box = phyz::ConvexUnionGeometry::box(mthz::Vec3(-delete_box_dim / 2.0, delete_box_height - delete_box_dim, -delete_box_dim / 2.0), delete_box_dim, delete_box_dim, delete_box_dim);
		phyz::RigidBody* delete_box_r = p.createRigidBody(delete_box, phyz::RigidBody::FIXED);

		phyz::RigidBody* funnel_r = p.createRigidBody(funnel, phyz::RigidBody::FIXED);
		bodies.push_back(TransformablePhysBod(fromGeometry(funnel, orange), funnel_r));

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos = display_position + mthz::Vec3(ramp_width / 2.0, display_height / 2.0, 6);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 60.0;

		int ticks_per_balldrop = static_cast<int>(0.25 / timestep);
		int display_release_tick_counter = 0;
		int ticks_for_release_time = static_cast<int>(3 / timestep);
		bool next_frame_trigger_reset = true;

		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -7.0, 0));

		p.registerCollisionAction(phyz::CollisionTarget::with(bucket_r), phyz::CollisionTarget::with(bucket_rest_rod_r), [&](phyz::RigidBody* b1, phyz::RigidBody* b2, const std::vector<phyz::Manifold>& manifold) {
			p.setMotorTargetPosition(bottom_hinge, 10000, -PI/2.0);

			display_release_tick_counter = ticks_for_release_time;
			display_bottom_r->setMovementType(phyz::RigidBody::DYNAMIC);
		});

		int tick_count = 0;
		int frame_count = 52;
		int N_FRAMES = 6572;
		int n_balls_spawned = 0;
		double video_aspect_ratio = 360.0 / 480.0;
		unsigned int starting_id = p.getNextBodyID(); //id of all spawned balls >= this number

		if (precompute_colors_mode) {
			printf("Precomputation not found. Calculating the colors of the balls and saving to Demo/resources/precomputations/bad_apple_colors.txt\n");

			int progress_bar_width = 50;
			ProgressBarState progress_bar_state{ INITIALIZING };
			progress_bar_state = render_progress_bar(0.0f, progress_bar_width, false, progress_bar_state, 0.1);


			std::vector<ColorHitInfo> ball_colors;
			while (frame_count <= N_FRAMES) {
				phyz_time -= timestep;
				p.timeStep();

				tick_count++;

				//trigger next frame
				if (display_release_tick_counter == ticks_for_release_time && next_frame_trigger_reset) {
					next_frame_trigger_reset = false;
					//first time this is triggered there are no balls in the display
					if (frame_count != 0) {
						olc::Sprite image = getFrame(frame_count);
						if (image.width == 0) printf("Warning: image width is 0, high chance the file might have not been found\n");

						double canvas_min_x = display_position.x;
						double canvas_min_y = display_position.y;
						double canvas_width = ramp_width;
						double canvas_height = video_aspect_ratio * ramp_width;

						double pixel_width = canvas_width / image.width;
						double pixel_height = canvas_height / image.height;

						int total_pixel_count = image.width * image.height;

						for (int px = 0; px < image.width; px++) {
							for (int py = 0; py < image.height; py++) {

								olc::Pixel col = image.GetPixel(px, py);

								int pixel_substep_count = 2;
								for (int sx = 0; sx < pixel_substep_count; sx++) {
									for (int sy = 0; sy < pixel_substep_count; sy++) {

										double ray_origin_x = canvas_min_x + (px + (sx + 1) / (pixel_substep_count + 1)) * pixel_width;
										double ray_origin_y = canvas_min_y + canvas_height - (py + (sy + 1) / (pixel_substep_count + 1)) * pixel_height;

										double ray_origin_z = display_position.z + 5.0;
										mthz::Vec3 ray_dir(0, 0, -1);
										mthz::Vec3 ray_origin(ray_origin_x, ray_origin_y, ray_origin_z);

										phyz::RayHitInfo hit = p.raycastFirstIntersection(ray_origin, ray_dir, { display_front_wall_r });
										if (hit.did_hit && hit.hit_object->getID() >= starting_id) {
											ball_colors[hit.hit_object->getID() - starting_id].color_sum.r += col.r / 255.0f;
											ball_colors[hit.hit_object->getID() - starting_id].color_sum.g += col.g / 255.0f;
											ball_colors[hit.hit_object->getID() - starting_id].color_sum.b += col.b / 255.0f;
											ball_colors[hit.hit_object->getID() - starting_id].hit_count++;
										}
									}
								}
							}
						}
					}
					frame_count += 2;

					progress_bar_state = render_progress_bar(frame_count / (float)N_FRAMES, progress_bar_width, false, progress_bar_state, 0.1);
				}

				display_release_tick_counter--;
				if (display_release_tick_counter == 0) {
					next_frame_trigger_reset = true;
					p.setMotorTargetPosition(bottom_hinge, 10000, 0);
				}

				
				if (display_release_tick_counter < 0 && abs(p.getMotorAngularPosition(bottom_hinge)) < 0.01) {
					display_bottom_r->setMovementType(phyz::RigidBody::FIXED);
				}

				if (tick_count % ticks_per_balldrop == 0) {
					for (mthz::Vec3 spawn_pos : dropper_spawn_positions) {
						phyz::ConvexUnionGeometry ball = phyz::ConvexUnionGeometry::sphere(spawn_pos, ball_radius);
						phyz::RigidBody* ball_r = p.createRigidBody(ball);
						bodies.push_back(TransformablePhysBod(fromGeometry(ball), ball_r));

						ball_colors.push_back(ColorHitInfo{ color{ 0.0f, 0.0f, 0.0f }, 0 });

						p.registerCollisionAction(phyz::CollisionTarget::with(ball_r), phyz::CollisionTarget::with(delete_box_r), [&, ball_r](phyz::RigidBody* b1, phyz::RigidBody* b2, const std::vector<phyz::Manifold>& manifold) {
							p.removeRigidBody(ball_r);

							for (int i = 0; i < bodies.size(); i++) {
								if (bodies[i].r == ball_r) {
									bodies.erase(bodies.begin() + i);
									break;
								}
							}
						});
					}
				}
			}

			progress_bar_state = render_progress_bar(1.0, progress_bar_width, true, progress_bar_state);

			std::vector<color> finalized_colors(ball_colors.size());
			for (int i = 0; i < ball_colors.size(); i++) {
				finalized_colors[i] = ball_colors[i].hit_count == 0 ? blue : color{ ball_colors[i].color_sum.r / ball_colors[i].hit_count, ball_colors[i].color_sum.g / ball_colors[i].hit_count, ball_colors[i].color_sum.b / ball_colors[i].hit_count };
			}

			writeComputation("resources/precomputations/bad_apple_colors.txt", finalized_colors);

			printf("To run using the computed ball colors, re-run the demo\n");

			manager->deselectCurrentScene();
			return;
		}
		else {
			printf("Precomputation found. Loading...\n");

			rndr::init(properties.window_width, properties.window_height, "Wrecking Ball Demo");

			std::vector<color> ball_colors;
			readComputation("resources/precomputations/bad_apple_colors.txt", &ball_colors);
			rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
			rndr::Shader shader("resources/shaders/Basic.shader");
			shader.bind();

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

				if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
					manager->deselectCurrentScene();
					return;
				}


				phyz_time += fElapsedTime;
				phyz_time = std::min<double>(phyz_time, 1.0 / 24.0);
				while (phyz_time > timestep) {
					phyz_time -= timestep;
					p.timeStep();

					tick_count++;

					display_release_tick_counter--;
					if (display_release_tick_counter == 0) {
						p.setMotorTargetPosition(bottom_hinge, 10000, 0);
					}
					if (display_release_tick_counter < 0 && abs(p.getMotorAngularPosition(bottom_hinge)) < 0.01) {
						display_bottom_r->setMovementType(phyz::RigidBody::FIXED);
					}

					if (tick_count % ticks_per_balldrop == 0 && p.getNextBodyID() - starting_id < ball_colors.size()) {
						for (mthz::Vec3 spawn_pos : dropper_spawn_positions) {
							n_balls_spawned++;
							phyz::ConvexUnionGeometry ball = phyz::ConvexUnionGeometry::sphere(spawn_pos, ball_radius);
							phyz::RigidBody* ball_r = p.createRigidBody(ball);
							int ball_id = ball_r->getID();
							bodies.push_back(TransformablePhysBod(fromGeometry(ball, ball_colors[ball_id - starting_id]), ball_r));

							p.registerCollisionAction(phyz::CollisionTarget::with(ball_r), phyz::CollisionTarget::with(delete_box_r), [&, ball_r](phyz::RigidBody* b1, phyz::RigidBody* b2, const std::vector<phyz::Manifold>& manifold) {
								p.removeRigidBody(ball_r);

								for (int i = 0; i < bodies.size(); i++) {
									if (bodies[i].r == ball_r) {
										bodies.erase(bodies.begin() + i);
										break;
									}
								}
							});
						}
					}
				}

				//the const is a lie
				auto setTransformedMatrix = [&](const TransformablePhysBod& b) {
					writeTransformedTo(b.mesh, &((TransformablePhysBod&)b).transformed_mesh, b.r->getPos(), b.r->getOrientation(), pos, orient, b.r->getAsleep(), color{ 1.0, 0.0, 0.0 });
				};

				if (properties.n_threads == 0) {
					for (TransformablePhysBod& b : bodies) {
						setTransformedMatrix(b);
					}
				}
				else {
					helper_threads.do_all<TransformablePhysBod>(properties.n_threads, bodies, setTransformedMatrix);
				}

				rndr::clear(rndr::color(248 / 255.0f, 241 / 255.0f, 229 / 255.0f));
				batch_array.flush();

				mthz::Vec3 cam_pos = pos;
				mthz::Quaternion cam_orient = orient;

				mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
				mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

				float aspect_ratio = (float)properties.window_height / properties.window_width;
				shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 50.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
				shader.setUniform3f("u_ambient_light", 0.95f, 0.95f, 0.95f);
				shader.setUniform3f("u_pointlight_pos", static_cast<float>(trnsfm_light_pos.x), static_cast<float>(trnsfm_light_pos.y), static_cast<float>(trnsfm_light_pos.z));
				shader.setUniform3f("u_pointlight_col", 0.05f, 0.05f, 0.05f);
				shader.setUniform1i("u_Asleep", false);

				for (TransformablePhysBod& b : bodies) {

					if (batch_array.remainingVertexCapacity() <= b.transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < b.transformed_mesh.indices.size()) {
						rndr::draw(batch_array, shader);
						batch_array.flush();
					}
					batch_array.push(b.transformed_mesh.vertices.data(), static_cast<uint32_t>(b.transformed_mesh.vertices.size()), b.transformed_mesh.indices);
				}

				rndr::draw(batch_array, shader);
			}
		}
	}
};