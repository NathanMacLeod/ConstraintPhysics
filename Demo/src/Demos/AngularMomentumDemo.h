#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class AngularMomentumDemo : public DemoScene {
public:
	AngularMomentumDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~AngularMomentumDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"I. K", "Spin flywheel"},
			ControlDescription{"J, L", "Rotate flywheel"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "Angular Momentum Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(false);
		p.setPGSIterations(45, 35);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;

		//************************
		//*******BASE PLATE*******
		//************************
		double s = 100;
		phyz::ConvexUnionGeometry geom2 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-s / 2, -2, -s / 2), s, 2, s);
		Mesh m2 = fromGeometry(geom2);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, phyz::RigidBody::FIXED);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		bodies.push_back({ m2, r2 });

		//************************
		//******CONTRAPTION*******
		//************************

		mthz::Vec3 base_position(0, 0, 0);
		double base_radius = 1.5;
		double base_height = 0.25;

		phyz::ConvexUnionGeometry base_plate = phyz::ConvexUnionGeometry::cylinder(base_position, base_radius, base_height);

		mthz::Vec3 column_position = base_position + mthz::Vec3(0, base_height, 0);
		double column_radius = 0.35;
		double column_height = 1.5;

		phyz::ConvexUnionGeometry column = phyz::ConvexUnionGeometry::cylinder(column_position, column_radius, column_height);
		
		mthz::Vec3 spinner_base_position = column_position + mthz::Vec3(0, column_height, 0);
		double spinner_base_radius = column_radius * 1.2;
		double spinner_base_length = column_radius * 2.1;

		phyz::ConvexUnionGeometry spinner_base = phyz::ConvexUnionGeometry::cylinder(spinner_base_position + mthz::Vec3(0, -spinner_base_length / 2.0, 0), spinner_base_radius, spinner_base_length)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner_base_position);

		mthz::Vec3 spinner_plate_position = spinner_base_position + mthz::Vec3(0, 0, spinner_base_length / 2.0);
		double spinner_plate_radius = spinner_base_radius - 0.05;
		double spinner_plate_thickness = 0.1;

		phyz::ConvexUnionGeometry spinner_plate = phyz::ConvexUnionGeometry::cylinder(spinner_plate_position, spinner_plate_radius, spinner_plate_thickness)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner_plate_position);

		double flywheel_thickness = 0.25;
		double arm_thickness = 0.1;
		double arm_width = 0.2;
		double arm_length = 0.85;

		mthz::Vec3 arm1_pos = spinner_plate_position + mthz::Vec3(-arm_width / 2.0, flywheel_thickness / 2.0, spinner_plate_thickness);
		phyz::ConvexUnionGeometry arm1 = phyz::ConvexUnionGeometry::box(arm1_pos, arm_width, arm_thickness, arm_length);

		mthz::Vec3 arm2_pos = spinner_plate_position + mthz::Vec3(-arm_width / 2.0, -flywheel_thickness / 2.0 - arm_thickness, spinner_plate_thickness);
		phyz::ConvexUnionGeometry arm2 = phyz::ConvexUnionGeometry::box(arm2_pos, arm_width, arm_thickness, arm_length);

		mthz::Vec3 flywheel_position = spinner_plate_position + mthz::Vec3(0, 0, spinner_plate_thickness + arm_length - arm_thickness / 2.0);
		double flywheel_radius = 0.65;
		phyz::ConvexUnionGeometry flywheel = phyz::ConvexUnionGeometry::cylinder(flywheel_position + mthz::Vec3(0, -flywheel_thickness/2.0, 0), flywheel_radius, flywheel_thickness);

		phyz::ConvexUnionGeometry base = { base_plate, column, spinner_base };
		phyz::ConvexUnionGeometry arm = { spinner_plate, arm1, arm2 };

		phyz::RigidBody* base_r = p.createRigidBody(base);
		p.addHingeConstraint(base_r, r2, base_position, mthz::Vec3(0, 1, 0));

		phyz::RigidBody* arm_r = p.createRigidBody(arm);
		phyz::ConstraintID rotate_motor = p.addMotorConstraint(
			p.addHingeConstraint(base_r, arm_r, spinner_plate_position, mthz::Vec3(0, 0, 1))
		);

		phyz::RigidBody* flywheel_r = p.createRigidBody(flywheel);
		phyz::ConstraintID spin_motor = p.addMotorConstraint(
			p.addHingeConstraint(arm_r, flywheel_r, flywheel_position, mthz::Vec3(0, 1, 0))
		);

		double rotate_speed = 1;
		double rotate_torque = 10;

		double spin_speed = 25;
		double spin_torque = 0.5;

		bodies.push_back({ fromGeometry(base), base_r });
		bodies.push_back({ fromGeometry(arm), arm_r });
		bodies.push_back({ fromGeometry(flywheel), flywheel_r });

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 2, 10);
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
				p.setMotorTargetVelocity(spin_motor, spin_torque, -spin_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				p.setMotorTargetVelocity(spin_motor, spin_torque, spin_speed);
			}
			else {
				p.setMotorOff(spin_motor);
			}


			if (rndr::getKeyDown(GLFW_KEY_J)) {
				p.setMotorTargetVelocity(rotate_motor, rotate_torque, rotate_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_L)) {
				p.setMotorTargetVelocity(rotate_motor, rotate_torque, -rotate_speed);
			}
			else {
				p.setMotorTargetVelocity(rotate_motor, rotate_torque, 0);
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
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 50.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
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