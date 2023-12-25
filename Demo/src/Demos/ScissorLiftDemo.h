#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class ScissorLiftDemo : public DemoScene {
public:
	ScissorLiftDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~ScissorLiftDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"I. K", "Move forklift forward, reverse"},
			ControlDescription{"J, L", "Turn forklift left, right"},
			ControlDescription{"T, G", "Raise, lower lift"},
			ControlDescription{"F, H", "Tilt lift forward, back"},
			ControlDescription{"Y", "Toggle first-person perspective"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	struct CreateScissorliftOut {
		std::vector<phyz::RigidBody*> all_bodies;

		phyz::RigidBody* top_front_right_rung_r;
		mthz::Vec3 top_front_right_pos;
		phyz::RigidBody* top_rear_right_rung_r;
		mthz::Vec3 top_rear_right_pos;
		phyz::RigidBody* top_front_left_rung_r;
		mthz::Vec3 top_front_left_pos;
		phyz::RigidBody* top_rear_left_rung_r;
		mthz::Vec3 top_rear_left_pos;
		phyz::RigidBody* bot_front_right_rung_r;
		mthz::Vec3 bot_front_right_pos;
		phyz::RigidBody* bot_rear_right_rung_r;
		mthz::Vec3 bot_rear_right_pos;
		phyz::RigidBody* bot_front_left_rung_r;
		mthz::Vec3 bot_front_left_pos;
		phyz::RigidBody* bot_rear_left_rung_r;
		mthz::Vec3 bot_rear_left_pos;

		mthz::Vec3 bot_left_middle_pos;
		mthz::Vec3 bot_right_middle_pos;
		mthz::Vec3 top_left_middle_pos;
		mthz::Vec3 top_right_middle_pos;
	};

	CreateScissorliftOut createScissors(std::vector<PhysBod>* bodies, phyz::PhysicsEngine* p, mthz::Vec3 pos, double right_left_spacing, double rung_length, double rung_width, double rung_thickness, int num_levels) {
		CreateScissorliftOut out;
		
		double hinge_dist_from_edge = rung_width / 2;
		double rung_height = (rung_length - 2 * hinge_dist_from_edge) / sqrt(2);
		phyz::ConvexUnionGeometry rung = phyz::ConvexUnionGeometry::box(mthz::Vec3(0, -hinge_dist_from_edge, -hinge_dist_from_edge), rung_thickness, rung_length, rung_width);
		phyz::ConvexUnionGeometry rear_rung = rung.getRotated(mthz::Quaternion(PI / 4, mthz::Vec3(1, 0, 0)));
		phyz::ConvexUnionGeometry front_rung = rung.getRotated(mthz::Quaternion(-PI / 4, mthz::Vec3(1, 0, 0)));

		phyz::RigidBody* prev_rear_right_rung = nullptr;
		phyz::RigidBody* prev_front_right_rung = nullptr;
		phyz::RigidBody* prev_rear_left_rung = nullptr;
		phyz::RigidBody* prev_front_left_rung = nullptr;

		for (int i = 0; i < num_levels; i++) {

			mthz::Vec3 elevation_change(0, rung_height * i, 0);

			mthz::Vec3 rear_right_pos = pos + elevation_change;
			phyz::ConvexUnionGeometry rear_right_rung = rear_rung.getTranslated(rear_right_pos);
			mthz::Vec3 front_right_pos = pos + mthz::Vec3(rung_thickness, 0, rung_height) + elevation_change;
			phyz::ConvexUnionGeometry front_right_rung = front_rung.getTranslated(front_right_pos);
			mthz::Vec3 rear_left_pos = pos + mthz::Vec3(right_left_spacing - rung_thickness, 0, 0) + elevation_change;
			phyz::ConvexUnionGeometry rear_left_rung = rear_rung.getTranslated(rear_left_pos);
			mthz::Vec3 front_left_pos = pos + mthz::Vec3(right_left_spacing - 2 * rung_thickness, 0, rung_height) + elevation_change;
			phyz::ConvexUnionGeometry front_left_rung = front_rung.getTranslated(front_left_pos);

			phyz::RigidBody* rear_right_r = p->createRigidBody(rear_right_rung);
			bodies->push_back({ fromGeometry(rear_right_rung), rear_right_r });
			phyz::RigidBody* front_right_r = p->createRigidBody(front_right_rung);
			bodies->push_back({ fromGeometry(front_right_rung), front_right_r });
			phyz::RigidBody* rear_left_r = p->createRigidBody(rear_left_rung);
			bodies->push_back({ fromGeometry(rear_left_rung), rear_left_r });
			phyz::RigidBody* front_left_r = p->createRigidBody(front_left_rung);
			bodies->push_back({ fromGeometry(front_left_rung), front_left_r });

			out.all_bodies.push_back(rear_right_r);
			out.all_bodies.push_back(front_right_r);
			out.all_bodies.push_back(rear_left_r);
			out.all_bodies.push_back(front_left_r);

			mthz::Vec3 right_middle_attach_pos = rear_right_pos + mthz::Vec3(0, rung_height / 2.0, rung_height / 2.0);
			mthz::Vec3 left_middle_attach_pos = rear_left_pos + mthz::Vec3(0, rung_height / 2.0, rung_height / 2.0);

			p->addHingeConstraint(rear_right_r, front_right_r, right_middle_attach_pos, mthz::Vec3(1, 0, 0));
			p->addHingeConstraint(rear_left_r, front_left_r, left_middle_attach_pos, mthz::Vec3(1, 0, 0));

			if (i + 1 == num_levels) {
				out.top_front_left_pos = rear_left_pos + mthz::Vec3(0, rung_height, 0);
				out.top_front_left_rung_r = rear_left_r;
				out.top_front_right_pos = rear_right_pos + mthz::Vec3(0, rung_height, 0);
				out.top_front_right_rung_r = rear_right_r;
				out.top_rear_left_pos = front_left_pos + mthz::Vec3(0, rung_height, 0);
				out.top_rear_left_rung_r = front_left_r;
				out.top_rear_right_pos = front_right_pos + mthz::Vec3(0, rung_height, 0);
				out.top_rear_right_rung_r = front_right_r;

				out.top_left_middle_pos = left_middle_attach_pos;
				out.top_right_middle_pos = right_middle_attach_pos;
			}
			if (i == 0) {
				out.bot_front_left_pos = front_left_pos;
				out.bot_front_left_rung_r = front_left_r;
				out.bot_front_right_pos = front_right_pos;
				out.bot_front_right_rung_r = front_right_r;
				out.bot_rear_left_pos = rear_left_pos;
				out.bot_rear_left_rung_r = rear_left_r;
				out.bot_rear_right_pos = rear_right_pos;
				out.bot_rear_right_rung_r = rear_right_r;

				out.bot_left_middle_pos = left_middle_attach_pos;
				out.bot_right_middle_pos = right_middle_attach_pos;
			}
			else {
				p->addHingeConstraint(prev_front_right_rung, rear_right_r, rear_right_pos, mthz::Vec3(1, 0, 0));
				p->addHingeConstraint(prev_rear_right_rung, front_right_r, front_right_pos, mthz::Vec3(1, 0, 0));
				p->addHingeConstraint(prev_front_left_rung, rear_left_r, rear_left_pos, mthz::Vec3(1, 0, 0));
				p->addHingeConstraint(prev_rear_left_rung, front_left_r, front_left_pos, mthz::Vec3(1, 0, 0));
			}

			prev_front_right_rung = front_right_r;
			prev_rear_right_rung = rear_right_r;
			prev_front_left_rung = front_left_r;
			prev_rear_left_rung = rear_left_r;
		}

		return out;
	}

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "Wrecking Ball Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(false);
		p.setPGSIterations(35, 15, 10);
		double timestep = 1 / 120.0;
		p.setStep_time(timestep);

		bool lock_cam = false;

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

		//************************
		//******Scissor Lift******
		//************************
		mthz::Vec3 scissor_lift_pos = mthz::Vec3(0, 2, 0);

		double base_width = 1;
		double base_length = 3;
		double base_thickness = 0.1;
		mthz::Vec3 base_position = scissor_lift_pos + mthz::Vec3(-base_width / 2.0, 0, -base_length / 2.0);
		phyz::ConvexUnionGeometry base_bottom = phyz::ConvexUnionGeometry::box(base_position, base_width, base_thickness, base_length);

		double base_side_height = 0.3;
		mthz::Vec3 base_rear_wall_pos = base_position + mthz::Vec3(0, base_thickness, 0);
		phyz::ConvexUnionGeometry base_rear_wall = phyz::ConvexUnionGeometry::box(base_rear_wall_pos, base_width, base_side_height, base_thickness);

		mthz::Vec3 base_front_wall_pos = base_rear_wall_pos + mthz::Vec3(0, 0, base_length - base_thickness);
		phyz::ConvexUnionGeometry base_front_wall = phyz::ConvexUnionGeometry::box(base_front_wall_pos, base_width, base_side_height, base_thickness);

		mthz::Vec3 base_right_wall_pos = base_rear_wall_pos + mthz::Vec3(0, 0, base_thickness);
		phyz::ConvexUnionGeometry base_right_wall = phyz::ConvexUnionGeometry::box(base_right_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);

		mthz::Vec3 base_left_wall_pos = base_rear_wall_pos + mthz::Vec3(base_width - base_thickness, 0, base_thickness);
		phyz::ConvexUnionGeometry base_left_wall = phyz::ConvexUnionGeometry::box(base_left_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);
		
		phyz::ConvexUnionGeometry base_geom = { base_bottom, base_rear_wall, base_front_wall, base_right_wall, base_left_wall };

		double wheel_radius = 0.3;
		double wheel_thickness = 0.15;
		double wheel_dist_off_bottom = 0.1;
		double wheel_dist_off_front = 0.1;
		phyz::ConvexUnionGeometry wheel = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), wheel_radius, wheel_thickness).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)));

		mthz::Vec3 front_right_wheel_pos = base_position + mthz::Vec3(0, wheel_dist_off_bottom, base_length - wheel_dist_off_front);
		phyz::ConvexUnionGeometry front_right_wheel = wheel.getTranslated(mthz::Vec3(front_right_wheel_pos));

		mthz::Vec3 rear_right_wheel_pos = base_position + mthz::Vec3(0, wheel_dist_off_bottom, wheel_dist_off_front);
		phyz::ConvexUnionGeometry rear_right_wheel = wheel.getTranslated(mthz::Vec3(rear_right_wheel_pos));

		mthz::Vec3 front_left_wheel_pos = base_position + mthz::Vec3(base_width + wheel_thickness, wheel_dist_off_bottom, base_length - wheel_dist_off_front);
		phyz::ConvexUnionGeometry front_left_wheel = wheel.getTranslated(mthz::Vec3(front_left_wheel_pos));

		mthz::Vec3 rear_left_wheel_pos = base_position + mthz::Vec3(base_width + wheel_thickness, wheel_dist_off_bottom, wheel_dist_off_front);
		phyz::ConvexUnionGeometry rear_left_wheel = wheel.getTranslated(mthz::Vec3(rear_left_wheel_pos));

		double dist_from_rear = 0.33;
		double dist_above_base = 0.1;
		double scissor_thickness = 0.03;
		CreateScissorliftOut scissor_out = createScissors(&bodies, &p, base_rear_wall_pos + mthz::Vec3(base_thickness, dist_above_base, dist_from_rear), base_width - 2 * base_thickness, 1, 0.1, scissor_thickness, 7);

		double slider_width = 0.1;
		mthz::Vec3 right_bottom_slider_pos = scissor_out.bot_front_right_pos + mthz::Vec3(-scissor_thickness, -slider_width / 2.0, -slider_width / 2.0);
		phyz::ConvexUnionGeometry bottom_right_slider = phyz::ConvexUnionGeometry::box(right_bottom_slider_pos, scissor_thickness, slider_width, slider_width, phyz::Material::modified_density(1000));
		phyz::ConvexUnionGeometry bottom_left_slider = bottom_right_slider.getTranslated(mthz::Vec3(base_width - scissor_thickness - 2 * base_thickness, 0, 0));
		phyz::ConvexUnionGeometry bottom_slider = { bottom_right_slider, bottom_left_slider };

		mthz::Vec3 right_top_slider_pos = scissor_out.top_front_right_pos + mthz::Vec3(0, -slider_width / 2.0, -slider_width / 2.0);
		phyz::ConvexUnionGeometry top_right_slider = phyz::ConvexUnionGeometry::box(right_top_slider_pos, scissor_thickness, slider_width, slider_width, phyz::Material::modified_density(1000));
		mthz::Vec3 left_top_slider_pos = right_top_slider_pos + mthz::Vec3(base_width - scissor_thickness - 2 * base_thickness, 0, 0);
		phyz::ConvexUnionGeometry top_left_slider = phyz::ConvexUnionGeometry::box(left_top_slider_pos, scissor_thickness, slider_width, slider_width);
		phyz::ConvexUnionGeometry top_slider = { top_right_slider, top_left_slider };

		mthz::Vec3 platform_pos = mthz::Vec3(base_position.x, right_top_slider_pos.y + right_bottom_slider_pos.y - base_position.y - base_thickness, base_position.z);
		phyz::ConvexUnionGeometry base_platform = phyz::ConvexUnionGeometry::box(platform_pos, base_width, base_thickness, base_length);
		
		mthz::Vec3 platform_rear_wall_pos = platform_pos + mthz::Vec3(0, -base_side_height, 0);
		phyz::ConvexUnionGeometry platform_rear_wall = phyz::ConvexUnionGeometry::box(platform_rear_wall_pos, base_width, base_side_height, base_thickness);

		mthz::Vec3 platform_front_wall_pos = platform_rear_wall_pos + mthz::Vec3(0, 0, base_length - base_thickness);
		phyz::ConvexUnionGeometry platform_front_wall = phyz::ConvexUnionGeometry::box(platform_front_wall_pos, base_width, base_side_height, base_thickness);

		mthz::Vec3 platform_right_wall_pos = platform_rear_wall_pos + mthz::Vec3(0, 0, base_thickness);
		phyz::ConvexUnionGeometry platform_right_wall = phyz::ConvexUnionGeometry::box(platform_right_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);

		mthz::Vec3 platform_left_wall_pos = platform_rear_wall_pos + mthz::Vec3(base_width - base_thickness, 0, base_thickness);
		phyz::ConvexUnionGeometry platform_left_wall = phyz::ConvexUnionGeometry::box(platform_left_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);

		phyz::ConvexUnionGeometry platform_geom = { base_platform, platform_rear_wall, platform_front_wall, platform_right_wall, platform_left_wall };

		phyz::RigidBody* base_r = p.createRigidBody(base_geom);
		bodies.push_back({ fromGeometry(base_geom), base_r });
		phyz::RigidBody* front_right_wheel_r = p.createRigidBody(front_right_wheel);
		bodies.push_back({ fromGeometry(front_right_wheel), front_right_wheel_r });
		phyz::RigidBody* rear_right_wheel_r = p.createRigidBody(rear_right_wheel);
		bodies.push_back({ fromGeometry(rear_right_wheel), rear_right_wheel_r });
		phyz::RigidBody* front_left_wheel_r = p.createRigidBody(front_left_wheel);
		bodies.push_back({ fromGeometry(front_left_wheel), front_left_wheel_r });
		phyz::RigidBody* rear_left_wheel_r = p.createRigidBody(rear_left_wheel);
		bodies.push_back({ fromGeometry(rear_left_wheel), rear_left_wheel_r });
		phyz::RigidBody* bottom_slider_r = p.createRigidBody(bottom_slider);
		bodies.push_back({ fromGeometry(bottom_slider), bottom_slider_r });
		phyz::RigidBody* top_slider_r = p.createRigidBody(top_slider);
		bodies.push_back({ fromGeometry(top_slider), top_slider_r });
		phyz::RigidBody* platform_r = p.createRigidBody(platform_geom);
		bodies.push_back({ fromGeometry(platform_geom), platform_r });

		p.addHingeConstraint(base_r, front_right_wheel_r, front_right_wheel_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(base_r, rear_right_wheel_r, rear_right_wheel_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(base_r, front_left_wheel_r, front_left_wheel_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(base_r, rear_left_wheel_r, rear_left_wheel_pos, mthz::Vec3(1, 0, 0));

		p.addHingeConstraint(base_r, scissor_out.bot_rear_right_rung_r, scissor_out.bot_rear_right_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(base_r, scissor_out.bot_rear_left_rung_r, scissor_out.bot_rear_left_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(bottom_slider_r, scissor_out.bot_front_right_rung_r, scissor_out.bot_front_right_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(bottom_slider_r, scissor_out.bot_front_left_rung_r, scissor_out.bot_front_left_pos, mthz::Vec3(1, 0, 0));
		p.addSliderConstraint(base_r, bottom_slider_r, bottom_slider_r->getCOM(), mthz::Vec3(0, 0, 1));

		p.addHingeConstraint(top_slider_r, scissor_out.top_rear_right_rung_r, scissor_out.top_front_right_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(top_slider_r, scissor_out.top_rear_left_rung_r, scissor_out.top_front_left_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(platform_r, scissor_out.top_front_right_rung_r, scissor_out.top_rear_right_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(platform_r, scissor_out.top_front_left_rung_r, scissor_out.top_rear_left_pos, mthz::Vec3(1, 0, 0));
		p.addSliderConstraint(platform_r, top_slider_r, top_slider_r->getCOM(), mthz::Vec3(0, 0, 1));
		
		phyz::ConstraintID scissor_heght_right = p.addDistanceConstraint(scissor_out.bot_rear_right_rung_r, scissor_out.top_rear_right_rung_r, scissor_out.bot_right_middle_pos, scissor_out.top_right_middle_pos);
		phyz::ConstraintID scissor_heght_left = p.addDistanceConstraint(scissor_out.bot_rear_left_rung_r, scissor_out.top_rear_left_rung_r, scissor_out.bot_left_middle_pos, scissor_out.top_left_middle_pos);

		double height_max = 5.45;
		double height_min = 0.8;
		double height_delta_vel = 1.5;

		std::vector<phyz::RigidBody*> all_bodies = scissor_out.all_bodies;
		all_bodies.push_back(base_r);
		all_bodies.push_back(front_right_wheel_r);
		all_bodies.push_back(rear_right_wheel_r);
		all_bodies.push_back(front_left_wheel_r);
		all_bodies.push_back(rear_left_wheel_r);
		all_bodies.push_back(bottom_slider_r);
		all_bodies.push_back(top_slider_r);
		all_bodies.push_back(platform_r);

		p.disallowCollisionSet(all_bodies);

		//p.setPistonTargetVelocity(scissor_slider, slider_force, 0);
		//printf("current_pos: %f\n", p.getPistonPosition(scissor_slider));

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(15, 3, 0);
		mthz::Quaternion orient(PI / 2.0, mthz::Vec3(0, 1, 0));
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		p.setGravity(mthz::Vec3(0, -6.0, 0));

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
			if (rndr::getKeyDown(GLFW_KEY_I)) {
				double current_dist = p.getDistanceConstraintTargetDistance(scissor_heght_right);
				double target_height = std::min<double>(height_max, current_dist + height_delta_vel * fElapsedTime);

				p.setDistanceConstraintTargetDistance(scissor_heght_right, target_height);
				p.setDistanceConstraintTargetDistance(scissor_heght_left, target_height);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				double current_dist = p.getDistanceConstraintTargetDistance(scissor_heght_right);
				double target_height = std::max<double>(height_min, current_dist - height_delta_vel * fElapsedTime);

				p.setDistanceConstraintTargetDistance(scissor_heght_right, target_height);
				p.setDistanceConstraintTargetDistance(scissor_heght_left, target_height);
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

			double slow_factor = 1;

			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > slow_factor * timestep) {
				phyz_time -= slow_factor * timestep;
				p.timeStep();
			}

			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = pos;
			mthz::Quaternion cam_orient = orient;

			mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

			double aspect_ratio = (double)properties.window_height / properties.window_width;
			//shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(mthz::Vec3(0, 0, 0), cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 550.0, 2.0, 2.0 * aspect_ratio, 60.0));
			shader.setUniform3f("u_ambient_light", 0.4, 0.4, 0.4);
			shader.setUniform3f("u_pointlight_pos", trnsfm_light_pos.x, trnsfm_light_pos.y, trnsfm_light_pos.z);
			shader.setUniform3f("u_pointlight_col", 0.6, 0.6, 0.6);
			shader.setUniform1i("u_Asleep", false);

			for (const PhysBod& b : bodies) {

				Mesh transformed_mesh = getTransformed(b.mesh, b.r->getPos(), b.r->getOrientation(), cam_pos, cam_orient, b.r->getAsleep(), color{ 1.0f, 0.0f, 0.0f });

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), transformed_mesh.vertices.size(), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);
		}
	}
};