#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class GyroscopeDemo : public DemoScene {
public:
	GyroscopeDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~GyroscopeDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"B", "Shoot a block in the direction of the camera"},
			ControlDescription{"I", "Kill the rotation"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "Angular Momentum Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(true);
		p.setPGSIterations(600, 600);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;

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
		//******CONTRAPTION*******
		//************************

		mthz::Vec3 gyroscope_center(0, 5, 0);

		double outer_ring_width = 0.3;
		double outer_ring_thickness = 0.1;
		double outer_ring_radius = 1;
		int ring_detail = 15;
		phyz::Geometry outer_ring = phyz::Geometry::ring(gyroscope_center + mthz::Vec3(0, -outer_ring_width / 2.0, 0), outer_ring_radius - outer_ring_thickness, outer_ring_radius, outer_ring_width, ring_detail);

		double mid_ring_width = 0.3;
		double mid_ring_thickness = 0.1;
		double mid_ring_radius = outer_ring_radius - 0.02;
		phyz::Geometry mid_ring = phyz::Geometry::ring(gyroscope_center + mthz::Vec3(0, -mid_ring_width / 2.0, 0), mid_ring_radius - mid_ring_thickness, mid_ring_radius, mid_ring_width, ring_detail)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), gyroscope_center);
			
		double axle_radius = 0.075;
		double axle_length = 3.0;
		double axle_excess = 0.1;

		mthz::Vec3 axle_pos = gyroscope_center + mthz::Vec3(0, -axle_length + mid_ring_radius + axle_excess, 0);
		phyz::Geometry axle = phyz::Geometry::cylinder(axle_pos, axle_radius, axle_length);

		double ball_radius = 0.125;
		double ball_seperation = 0.02;

		phyz::Geometry ball = phyz::Geometry::sphere(axle_pos + mthz::Vec3(0, -ball_seperation, 0), ball_radius);

		double spinner_radius = 0.8;
		double spinner_thickness = 0.5;

		phyz::Geometry spinner = phyz::Geometry::cylinder(gyroscope_center + mthz::Vec3(0, -spinner_thickness / 2.0, 0), spinner_radius, spinner_thickness, 10, phyz::Material::modified_density(1.0));

		phyz::Geometry gyroscope_body = { outer_ring, mid_ring, axle, ball};
		
		
		phyz::RigidBody* body_r = p.createRigidBody(gyroscope_body);
		phyz::RigidBody* spinner_r = p.createRigidBody(spinner);

		p.addHingeConstraint(body_r, spinner_r, gyroscope_center, mthz::Vec3(0, 1, 0), 2000, 2000);

		bodies.push_back({ fromGeometry(gyroscope_body), body_r });
		bodies.push_back({ fromGeometry(spinner), spinner_r });

		p.addBallSocketConstraint(body_r, r2, axle_pos + mthz::Vec3(0, -ball_seperation, 0));

		spinner_r->setAngVel(mthz::Vec3(0, 60, 0));

		//body_r->setAngVel(mthz::Vec3(0, 50, 0));

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 5, 10);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 600.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -6.0, 0));
		p.setAngleVelUpdateTickCount(30);

		bool b_down = false;

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
				spinner_r->setAngVel(body_r->getAngVel());
			}

			if (rndr::getKeyPressed(GLFW_KEY_B)) {
				double block_size = 0.3;
				double block_speed = 15;

				phyz::Geometry block = phyz::Geometry::box(pos + mthz::Vec3(-block_size / 2.0, -block_size / 2.0, -block_size / 2.0), block_size, block_size, block_size);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				block_r->setVel(camera_dir * block_speed);

				bodies.push_back({ fromGeometry(block), block_r });
			}


			if (rndr::getKeyDown(GLFW_KEY_J)) {
				//p.setMotorTargetVelocity(rotate_motor, rotate_torque, rotate_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_L)) {
				//p.setMotorTargetVelocity(rotate_motor, rotate_torque, -rotate_speed);
			}
			else {
				//p.setMotorTargetVelocity(rotate_motor, rotate_torque, 0);
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

			double aspect_ratio = (double)properties.window_height / properties.window_width;
			//shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(mthz::Vec3(0, 0, 0), cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0 * aspect_ratio, 120.0));
			shader.setUniform3f("u_ambient_light", 0.4, 0.4, 0.4);
			shader.setUniform3f("u_pointlight_pos", trnsfm_light_pos.x, trnsfm_light_pos.y, trnsfm_light_pos.z);
			shader.setUniform3f("u_pointlight_col", 0.6, 0.6, 0.6);
			shader.setUniform1i("u_Asleep", false);

			for (const PhysBod& b : bodies) {

				Mesh transformed_mesh = getTransformed(b.mesh, b.r->getPos(), b.r->getOrientation(), cam_pos, cam_orient, b.r->getAsleep(), color{ 1.0f, 0.0f, 0.0f });

				if (batch_array.remainingCapacity() <= transformed_mesh.vertices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), transformed_mesh.vertices.size(), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);
		}
	}
};