#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class NewtonsCradleDemo : public DemoScene {
public:
	NewtonsCradleDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~NewtonsCradleDemo() override {

	}

	std::map<std::string, std::string> askParameters() override {
		std::map<std::string, std::string> out;

		out["pendulum count"] = askCustomParameterValue(
			"Type the number of pendulums to create (default 5): ",
			[](std::string s) -> bool {
				for (char c : s) {
					if (c < '0' || c > '9') {
						return false;
					}
				}

				return true;
			},
			"Must be a positive integer: "
		);

		if (out["pendulum count"] == "") out["pendulum count"] = "5";

		return out;
	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"R", "Reset"},
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
		p.setGlobalConstraintForceMixing(0.01);
		//p.setPrintPerformanceData(true);
		p.setStep_time(1.0 / 200);
		//p.setWarmStartDisabled(true);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;
		
		int pendulum_count = atoi(parameters["pendulum count"].c_str());
		double ball_radius = 0.5;
		double pendulum_length = 5;
		double pendulum_radius = 0.1;
		double gap_multiplier = 1.1;
		mthz::Vec3 newton_pos(-ball_radius * pendulum_count * gap_multiplier / 2, 0, -20);

		

		phyz::RigidBody* base = p.createRigidBody(phyz::ConvexUnionGeometry::box(mthz::Vec3(), 1, 1, 1), phyz::RigidBody::FIXED);
		base->setNoCollision(true);

		phyz::ConvexUnionGeometry pendulum = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -pendulum_length, 0), pendulum_radius, pendulum_length);
		phyz::ConvexUnionGeometry ball = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(0, -pendulum_length, 0), ball_radius, phyz::Material{ phyz::CFM{phyz::USE_GLOBAL}, 1.0, 1.0, 0.01, 0.2 });
		phyz::ConvexUnionGeometry pendulum_w_ball = { pendulum, ball };
		

		for (int i = 0; i < pendulum_count; i++) {
			mthz::Vec3 pendulum_pos = newton_pos + mthz::Vec3(2 * i * ball_radius * gap_multiplier, 0, 0);
			phyz::ConvexUnionGeometry pendulum_geom = pendulum_w_ball.getTranslated(pendulum_pos);

			if (i == 0) {
				pendulum_geom = pendulum_geom.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1)), pendulum_pos);
			}

			phyz::RigidBody* pendulum_r = p.createRigidBody(pendulum_geom);
			bodies.push_back({ fromGeometry(pendulum_geom), pendulum_r });
			p.addHingeConstraint(base, pendulum_r, pendulum_pos, mthz::Vec3(0, 0, 1));
		}

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 0, 0);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 90.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -120.0, 0));

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


			/*if (rndr::getKeyPressed(GLFW_KEY_T)) {
				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

				if (hit_info.did_hit) {
					hit_info.hit_object->applyImpulse(camera_dir * 5, hit_info.hit_position);
				}
			}*/

			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				for (PhysBod& p : bodies) {
					phyz::RigidBody* r = p.r;
					r->setOrientation(mthz::Quaternion());
					r->setToPosition(mthz::Vec3());
					r->setAngVel(mthz::Vec3());
					r->setVel(mthz::Vec3());
				}
			}
			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
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
};