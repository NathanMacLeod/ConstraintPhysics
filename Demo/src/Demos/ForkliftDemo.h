#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class ForkliftDemo : public DemoScene {
public:
	ForkliftDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~ForkliftDemo() override {

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
		//*******LEAP FROG********
		//************************

		mthz::Vec3 frog_pos(0, 5, 0);

		double frog_length = 2;
		double frog_width = 1;
		double frog_thickness = 0.33;
		phyz::Geometry frog_base = phyz::Geometry::box(frog_pos, frog_length, frog_thickness, frog_width);

		double leg_width = 0.15;
		double leg_height = 0.4;
		double gap = 0.6;

		phyz::Geometry leg1 = phyz::Geometry::box(frog_pos + mthz::Vec3(0, - leg_height, 0), leg_width, leg_height, leg_width);
		phyz::Geometry leg2 = phyz::Geometry::box(frog_pos + mthz::Vec3(frog_length - leg_width, - leg_height, 0), leg_width, leg_height, leg_width);
		phyz::Geometry leg3 = phyz::Geometry::box(frog_pos + mthz::Vec3(frog_length - leg_width, - leg_height, frog_width - leg_width), leg_width, leg_height, leg_width);
		phyz::Geometry leg4 = phyz::Geometry::box(frog_pos + mthz::Vec3(0, -leg_height, frog_width - leg_width), leg_width, leg_height, leg_width);

		phyz::RigidBody* frog_base_r = p.createRigidBody(frog_base);
		phyz::RigidBody* leg1_r = p.createRigidBody(leg1);
		phyz::RigidBody* leg2_r = p.createRigidBody(leg2);
		phyz::RigidBody* leg3_r = p.createRigidBody(leg3);
		phyz::RigidBody* leg4_r = p.createRigidBody(leg4);

		phyz::ConstraintID leg1_c = p.addSliderConstraint(frog_base_r, leg1_r, frog_pos, mthz::Vec3(0, -1, 0), 350, 350, std::numeric_limits<double>::infinity(), 0);
		p.addSliderConstraint(frog_base_r, leg2_r, frog_pos, mthz::Vec3(0, -1, 0), 350, 350, std::numeric_limits<double>::infinity(), 0);
		p.addSliderConstraint(frog_base_r, leg3_r, frog_pos, mthz::Vec3(0, -1, 0), 350, 350, std::numeric_limits<double>::infinity(), 0);
		phyz::ConstraintID leg4_c = p.addSliderConstraint(frog_base_r, leg4_r, frog_pos, mthz::Vec3(0, -1, 0), 350, 350, std::numeric_limits<double>::infinity(), 0);

		p.addSpring(frog_base_r, leg1_r, leg1_r->getCOM() + mthz::Vec3(0, leg_height, 0), leg1_r->getCOM() + mthz::Vec3(0, leg_height / 2.0, 0), 0.15, 15.0, gap);
		p.addSpring(frog_base_r, leg2_r, leg2_r->getCOM() + mthz::Vec3(0, leg_height, 0), leg2_r->getCOM() + mthz::Vec3(0, leg_height / 2.0, 0), 0.15, 15.0, gap);
		p.addSpring(frog_base_r, leg3_r, leg3_r->getCOM() + mthz::Vec3(0, leg_height, 0), leg3_r->getCOM() + mthz::Vec3(0, leg_height / 2.0, 0), 0.15, 15.0, gap);
		p.addSpring(frog_base_r, leg4_r, leg4_r->getCOM() + mthz::Vec3(0, leg_height, 0), leg4_r->getCOM() + mthz::Vec3(0, leg_height / 2.0, 0), 0.15, 15.0, gap);

		bodies.push_back({ fromGeometry(frog_base), frog_base_r });
		bodies.push_back({ fromGeometry(leg1), leg1_r });
		bodies.push_back({ fromGeometry(leg2), leg2_r });
		bodies.push_back({ fromGeometry(leg3), leg3_r });
		bodies.push_back({ fromGeometry(leg4), leg4_r });

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

			if (rndr::getKeyDown(GLFW_KEY_I)) {
				p.setPiston(leg1_c, 10 * 90, 1);
				p.setPiston(leg4_c, 10 * 90, 1);
			}
			else {
				p.setPiston(leg1_c, 0, 0);
				p.setPiston(leg4_c, 0, 0);
			}
			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				for (PhysBod b : bodies) {
					delete b.mesh.ib;
					delete b.mesh.va;
				}
				manager->deselectCurrentScene();
				return;
			}


			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > timestep) {
				phyz_time -= timestep;
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