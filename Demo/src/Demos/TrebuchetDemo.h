#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class TrebuchetDemo : public DemoScene {
public:
	TrebuchetDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~TrebuchetDemo() override {

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

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "Wrecking Ball Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(false);
		p.setPGSIterations(45, 35);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;

		//************************
		//*******BASE PLATE*******
		//************************
		double s = 100;
		phyz::Geometry geom2 = phyz::Geometry::box(mthz::Vec3(-s / 2, -2, -s / 2), s, 2, s);
		Mesh m2 = fromGeometry(geom2);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, true);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		bodies.push_back({ m2, r2 });

		//************************
		//****TREBUCHET FRAME*****
		//************************
		mthz::Vec3 trebuchet_pos = mthz::Vec3(0, 0, 0);
		double arm_channel_width = 0.5;
		double rail_width = 0.25;
		double rail_length = 5;
		double rail_height = 2;
		double drop_channel_width = 0.21;
		double drop_channel_height = 5;
		double drop_channel_support_width = 0.33;

		mthz::Vec3 channel_beam1_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0 + rail_width, 0, -drop_channel_width / 2.0 - drop_channel_support_width);
		phyz::Geometry channel_beam1 = phyz::Geometry::box(channel_beam1_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		mthz::Vec3 channel_beam2_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0 + rail_width, 0, drop_channel_width / 2.0);
		phyz::Geometry channel_beam2 = phyz::Geometry::box(channel_beam2_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		phyz::Geometry drop_channel_cap1 = phyz::Geometry::box(channel_beam1_pos + mthz::Vec3(0, drop_channel_height, 0), drop_channel_support_width, drop_channel_support_width, 2 * drop_channel_support_width + drop_channel_width);

		mthz::Vec3 channel_beam3_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width - drop_channel_support_width, 0, -drop_channel_width / 2.0 - drop_channel_support_width);
		phyz::Geometry channel_beam3 = phyz::Geometry::box(channel_beam3_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		mthz::Vec3 channel_beam4_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width - drop_channel_support_width, 0, drop_channel_width / 2.0);
		phyz::Geometry channel_beam4 = phyz::Geometry::box(channel_beam4_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		phyz::Geometry drop_channel_cap2 = phyz::Geometry::box(channel_beam3_pos + mthz::Vec3(0, drop_channel_height, 0), drop_channel_support_width, drop_channel_support_width, 2 * drop_channel_support_width + drop_channel_width);


		mthz::Vec3 rail1_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0, rail_height - rail_width, -rail_length / 2.0);
		phyz::Geometry rail1 = phyz::Geometry::box(rail1_pos, rail_width, rail_width, (rail_length - drop_channel_width) / 2.0);

		mthz::Vec3 rail2_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0, rail_height - rail_width, drop_channel_width / 2.0);
		phyz::Geometry rail2 = phyz::Geometry::box(rail2_pos, rail_width, rail_width, (rail_length - drop_channel_width) / 2.0);

		mthz::Vec3 rail3_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width, rail_height - rail_width, -rail_length / 2.0);
		phyz::Geometry rail3 = phyz::Geometry::box(rail3_pos, rail_width, rail_width, (rail_length - drop_channel_width) / 2.0);

		mthz::Vec3 rail4_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width, rail_height - rail_width, drop_channel_width / 2.0);
		phyz::Geometry rail4 = phyz::Geometry::box(rail4_pos, rail_width, rail_width, (rail_length - drop_channel_width) / 2.0);


		double weight_width = 0.6;
		double weight_thickness = 0.5;
		double weight_gap = 0.01;
		double weight_axle_radius = 0.1;
		double weight_height = drop_channel_height - weight_axle_radius;
		double weight_axle_excess = 0.05;

		double weight_dist = arm_channel_width/2.0 + (rail_width + drop_channel_support_width + weight_gap);
		double weight_axle_length = 2 * (weight_dist + weight_thickness + weight_axle_excess);
		mthz::Vec3 weight_center = trebuchet_pos + mthz::Vec3(0, weight_height, 0);
		phyz::Geometry weight_axle = phyz::Geometry::cylinder(weight_center - mthz::Vec3(0, weight_axle_length / 2.0, 0), weight_axle_radius, weight_axle_length)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), weight_center);

		phyz::Geometry weight1 = phyz::Geometry::box(weight_center + mthz::Vec3(weight_dist, -weight_width / 2.0, -weight_width / 2.0), weight_thickness, weight_width, weight_width);
		phyz::Geometry weight2 = phyz::Geometry::box(weight_center + mthz::Vec3(-weight_dist - weight_thickness, -weight_width / 2.0, -weight_width / 2.0), weight_thickness, weight_width, weight_width);


		double release_pin_radius = 0.06;
		double release_pin_gap = 0.01;
		double release_pin_excess = 0.05;

		double release_pin_height = weight_height - weight_axle_radius - release_pin_radius - release_pin_gap;
		double release_pin_length = drop_channel_width + 2 * drop_channel_support_width + 2 * release_pin_excess;

		mthz::Vec3 release_pin1_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0 + rail_width + drop_channel_support_width / 2.0, release_pin_height, -drop_channel_width / 2.0 - drop_channel_support_width - release_pin_excess);
		phyz::Geometry release_pin1 = phyz::Geometry::cylinder(release_pin1_pos, release_pin_radius, release_pin_length)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), release_pin1_pos);

		mthz::Vec3 release_pin2_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width - drop_channel_support_width / 2.0, release_pin_height, -drop_channel_width / 2.0 - drop_channel_support_width - release_pin_excess);
		phyz::Geometry release_pin2 = phyz::Geometry::cylinder(release_pin2_pos, release_pin_radius, release_pin_length)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), release_pin2_pos);


		double trebuchet_arm_length = 4;
		double trebuchet_arm_height = 0.5;
		double trebuchet_arm_thickness = 0.3;

		mthz::Vec3 trebuchet_arm_pos = weight_center + mthz::Vec3(-trebuchet_arm_thickness / 2.0, -trebuchet_arm_height / 2.0, -trebuchet_arm_length + trebuchet_arm_height / 2.0);
		phyz::Geometry trebuchet_main_arm = phyz::Geometry::box(trebuchet_arm_pos, trebuchet_arm_thickness, trebuchet_arm_height, trebuchet_arm_length);


		phyz::Geometry trebuchet_frame = { channel_beam1, channel_beam2, channel_beam3, channel_beam4, drop_channel_cap1, drop_channel_cap2, rail1, rail2, rail3, rail4 };
		phyz::RigidBody* trebuchet_frame_r = p.createRigidBody(trebuchet_frame, true);

		phyz::Geometry release_pins = { release_pin1, release_pin2 };
		phyz::RigidBody* release_pins_r = p.createRigidBody(release_pins);

		phyz::Geometry weight = { weight_axle, weight1, weight2 };
		phyz::RigidBody* weight_r = p.createRigidBody(weight);

		phyz::Geometry trebuchet_arm = { trebuchet_main_arm };
		phyz::RigidBody* trebuchet_arm_r = p.createRigidBody(trebuchet_arm, true);

		bodies.push_back({ fromGeometry(trebuchet_frame), trebuchet_frame_r });
		bodies.push_back({ fromGeometry(weight), weight_r });
		bodies.push_back({ fromGeometry(release_pins), release_pins_r });
		bodies.push_back({ fromGeometry(trebuchet_arm), trebuchet_arm_r });

		phyz::ConstraintID release = p.addSliderConstraint(trebuchet_frame_r, release_pins_r, release_pins_r->getCOM(), mthz::Vec3(0, 0, -1), 350, 350, -2 * release_pin_excess - drop_channel_width - drop_channel_support_width, 0);
		p.setPiston(release, 10, 1);

		p.addHingeConstraint(trebuchet_arm_r, weight_r, weight_center, mthz::Vec3(1, 0, 0));

		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 10, 10);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 90.0;
		p.setStep_time(timestep);
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

			if (rndr::getKeyDown(GLFW_KEY_F)) {
				p.setPiston(release, 10, -1);
			}
			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				for (PhysBod b : bodies) {
					delete b.mesh.ib;
					delete b.mesh.va;
				}
				manager->deselectCurrentScene();
				return;
			}

			double slow_factor = 1;

			phyz_time += fElapsedTime;
			//phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > slow_factor * timestep) {
				phyz_time -= slow_factor * timestep;
				p.timeStep();
			}


			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

			for (const PhysBod& b : bodies) {
				mthz::Vec3 cam_pos = pos;
				mthz::Quaternion cam_orient = orient;

				mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
				mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

				shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(cam_pos, cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
				shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0));
				shader.setUniform3f("u_ambient_light", 0.4, 0.4, 0.4);
				shader.setUniform3f("u_pointlight_pos", trnsfm_light_pos.x, trnsfm_light_pos.y, trnsfm_light_pos.z);
				shader.setUniform3f("u_pointlight_col", 0.6, 0.6, 0.6);
				shader.setUniform1i("u_Asleep", b.r->getAsleep());
				rndr::draw(*b.mesh.va, *b.mesh.ib, shader);

			}
		}
	}
};