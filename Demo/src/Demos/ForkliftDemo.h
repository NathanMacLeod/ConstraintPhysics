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
		//*******Fork Lift********
		//************************
		//reference: https://youtu.be/mq18jajrx68?t=13

		mthz::Vec3 forklift_position = mthz::Vec3(0, 2, 0);
		mthz::Vec3 forward_dir(0, 0, 1);

		phyz::Material chasis_material = phyz::Material::modified_density(3.0);
		double chasis_base_width = 2.25;
		double chasis_base_length = 5;
		double chasis_base_height = 1;
		mthz::Vec3 chasis_base_pos = forklift_position + mthz::Vec3(-chasis_base_length / 2.0, -chasis_base_width / 2.0, -chasis_base_height / 2.0);
		phyz::Geometry chasis_base = phyz::Geometry::box(chasis_base_pos, chasis_base_width, chasis_base_height, chasis_base_length, chasis_material);

		double chasis_layer2_height = 0.6;
		double layer2_front_length = 0.5;
		double layer2_rear_length = 3.25;
		mthz::Vec3 layer2_front_block_position = chasis_base_pos + mthz::Vec3(0, chasis_base_height, chasis_base_length - layer2_front_length);
		phyz::Geometry layer2_front_block = phyz::Geometry::box(layer2_front_block_position, chasis_base_width, chasis_layer2_height, layer2_front_length);

		mthz::Vec3 layer2_rear_block_position = chasis_base_pos + mthz::Vec3(0, chasis_base_height, 0);
		phyz::Geometry layer2_rear_block = phyz::Geometry::box(layer2_rear_block_position, chasis_base_width, chasis_layer2_height, layer2_rear_length);

		double chasis_layer3_height = 0.4;
		double chasis_layer3_length = 2.5;
		mthz::Vec3 layer3_block_position = layer2_rear_block_position + mthz::Vec3(0, chasis_layer2_height, 0);
		phyz::Geometry layer3_block = phyz::Geometry::box(layer3_block_position, chasis_base_width, chasis_layer3_height, chasis_layer3_length);

		double roof_rear_support_length = 0.25;
		double roof_rear_support_width = 0.25;
		double roof_rear_support_height = 2.25;
		mthz::Vec3 roof_rear_support1_position = layer3_block_position + mthz::Vec3(0, chasis_layer3_height, chasis_layer3_length - roof_rear_support_length);
		phyz::Geometry roof_rear_support1 = phyz::Geometry::box(roof_rear_support1_position, roof_rear_support_width, roof_rear_support_height, roof_rear_support_length);

		mthz::Vec3 roof_rear_support2_position = layer3_block_position + mthz::Vec3(chasis_base_width - roof_rear_support_width, chasis_layer3_height, chasis_layer3_length - roof_rear_support_length);
		phyz::Geometry roof_rear_support2 = phyz::Geometry::box(roof_rear_support2_position, roof_rear_support_width, roof_rear_support_height, roof_rear_support_length);

		double roof_thickness = 0.2;
		double roof_length = 2.25;

		mthz::Vec3 roof_position = roof_rear_support1_position + mthz::Vec3(0, roof_rear_support_height, 0);
		phyz::Geometry roof = phyz::Geometry::box(roof_position, chasis_base_width, roof_thickness, roof_length);

		double front_support_gap = 0.01;
		double front_support_width = 0.25;
		double front_support_verticle_dist = roof_position.y + roof_thickness - layer2_front_block_position.y - chasis_layer2_height - front_support_gap;
		double front_support_horz_dist = chasis_base_pos.z + chasis_base_length - roof_position.z - roof_length;

		double front_support_length = sqrt(front_support_verticle_dist * front_support_verticle_dist + front_support_horz_dist * front_support_horz_dist);
		double front_support_angle = PI / 2.0 - atan(front_support_verticle_dist / front_support_horz_dist);

		mthz::Vec3 front_support1_position = layer2_front_block_position + mthz::Vec3(front_support_gap, chasis_layer2_height, layer2_front_length);
		phyz::Geometry front_support1 = phyz::Geometry::box(front_support1_position + mthz::Vec3(0, 0, -front_support_width), front_support_width, front_support_length, front_support_width)
			.getRotated(mthz::Quaternion(-front_support_angle, mthz::Vec3(1, 0, 0)), front_support1_position); 

		mthz::Vec3 front_support2_position = layer2_front_block_position + mthz::Vec3(chasis_base_width - front_support_width - front_support_gap, chasis_layer2_height, layer2_front_length);
		phyz::Geometry front_support2 = phyz::Geometry::box(front_support2_position + mthz::Vec3(0, 0, -front_support_width), front_support_width, front_support_length, front_support_width)
			.getRotated(mthz::Quaternion(-front_support_angle, mthz::Vec3(1, 0, 0)), front_support2_position);


		double front_wheel_radius = 0.9;
		double wheel_width = 0.55;
		double front_wheel_attach_height = 0.25;
		double rear_wheel_radius = 0.8;
		double rear_wheel_attach_height = front_wheel_attach_height - front_wheel_radius + rear_wheel_radius;
		double rear_wheel_attach_dist = 0.5;
		double front_wheel_attach_dist = chasis_base_length - 0.2;
		double wheel_attach_gap = 0.1;
		int wheel_detail = 60;

		double suspension_block_size = 1.0;

		mthz::Vec3 front_wheel1_position = chasis_base_pos + mthz::Vec3(-wheel_attach_gap, front_wheel_attach_height, front_wheel_attach_dist);
		phyz::Geometry front_wheel1 = phyz::Geometry::cylinder(front_wheel1_position, front_wheel_radius, wheel_width, wheel_detail, phyz::Material::super_friction())
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), front_wheel1_position);

		phyz::Geometry suspension_block1 = phyz::Geometry::box(front_wheel1_position + mthz::Vec3(0, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);

		mthz::Vec3 front_wheel2_position = chasis_base_pos + mthz::Vec3(chasis_base_width + wheel_attach_gap, front_wheel_attach_height, front_wheel_attach_dist);
		phyz::Geometry front_wheel2 = phyz::Geometry::cylinder(front_wheel2_position, front_wheel_radius, wheel_width, wheel_detail, phyz::Material::super_friction())
			.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1)), front_wheel2_position);

		phyz::Geometry suspension_block2 = phyz::Geometry::box(front_wheel2_position + mthz::Vec3(-suspension_block_size, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);
		
		mthz::Vec3 rear_wheel1_position = chasis_base_pos + mthz::Vec3(-wheel_attach_gap, rear_wheel_attach_height, rear_wheel_attach_dist);
		phyz::Geometry rear_wheel1 = phyz::Geometry::cylinder(rear_wheel1_position, rear_wheel_radius, wheel_width, wheel_detail, phyz::Material::super_friction())
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), rear_wheel1_position);

		phyz::Geometry steering_block1 = phyz::Geometry::box(rear_wheel1_position + mthz::Vec3(0, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);

		mthz::Vec3 rear_wheel2_position = chasis_base_pos + mthz::Vec3(chasis_base_width + wheel_attach_gap, rear_wheel_attach_height, rear_wheel_attach_dist);
		phyz::Geometry rear_wheel2 = phyz::Geometry::cylinder(rear_wheel2_position, rear_wheel_radius, wheel_width, wheel_detail, phyz::Material::super_friction())
			.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1)), rear_wheel2_position);

		phyz::Geometry steering_block2 = phyz::Geometry::box(rear_wheel2_position + mthz::Vec3(-suspension_block_size, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);

		double mast_width = 0.4;
		double mast_height = 5;
		double mast_thickness = 0.35;
		double mast_gap = 0.25;
		double mast_separation = 0.8;

		mthz::Vec3 mast_position = chasis_base_pos + mthz::Vec3(chasis_base_width / 2.0, 0, chasis_base_length + mast_gap);

		mthz::Vec3 mast1_position = mast_position + mthz::Vec3(-mast_separation / 2.0 - mast_width, 0, 0);
		phyz::Geometry mast1 = phyz::Geometry::box(mast1_position, mast_width, mast_height, mast_thickness);

		mthz::Vec3 mast2_position = mast_position + mthz::Vec3(mast_separation / 2.0, 0, 0);
		phyz::Geometry mast2 = phyz::Geometry::box(mast2_position, mast_width, mast_height, mast_thickness);


		phyz::Geometry chasis = { chasis_base, layer2_front_block, layer2_rear_block, layer3_block, roof_rear_support1, roof_rear_support2, front_support1, front_support2, roof};
		phyz::RigidBody* chasis_r = p.createRigidBody(chasis, true);

		phyz::RigidBody* suspension_block1_r = p.createRigidBody(suspension_block1);
		phyz::RigidBody* suspension_block2_r = p.createRigidBody(suspension_block2);

		phyz::RigidBody* front_wheel1_r = p.createRigidBody(front_wheel1);
		phyz::RigidBody* front_wheel2_r = p.createRigidBody(front_wheel2);

		phyz::RigidBody* steering_block1_r = p.createRigidBody(steering_block1);
		phyz::RigidBody* steering_block2_r = p.createRigidBody(steering_block2);

		phyz::RigidBody* rear_wheel1_r = p.createRigidBody(rear_wheel1);
		phyz::RigidBody* rear_wheel2_r = p.createRigidBody(rear_wheel2);

		phyz::Geometry mast = { mast1, mast2 };
		phyz::RigidBody* mast_r = p.createRigidBody(mast);

		double mast_rotation = 0.06;
		double mast_torque = 300;
		phyz::ConstraintID mast_motor = p.addHingeConstraint(chasis_r, mast_r, mast_position, mthz::Vec3(1, 0, 0), 350, 350, 0, mast_rotation);

		double break_torque = 150;
		double steer_torque = 65;
		double steer_angle = 0.45;
		double power_torque = 30;
		double forward_speed = 30;
		double reverse_speed = 10;
		double hard_suspension_limit_dist = std::numeric_limits<double>::infinity();
		double spring_dist = 1.0;
		double spring_stiffness = 600;
		double spring_damping = 25.5;

		p.addSliderConstraint(chasis_r, suspension_block1_r, front_wheel1_position, mthz::Vec3(0, -1, 0), 350, 350, -hard_suspension_limit_dist, hard_suspension_limit_dist);
		p.addSliderConstraint(chasis_r, suspension_block2_r, front_wheel2_position, mthz::Vec3(0, -1, 0), 350, 350, -hard_suspension_limit_dist, hard_suspension_limit_dist);

		phyz::ConstraintID steer_motor1 = p.addSlidingHingeConstraint(chasis_r, steering_block1_r, rear_wheel1_position, mthz::Vec3(0, -1, 0), 350, 350, -hard_suspension_limit_dist, hard_suspension_limit_dist);
		phyz::ConstraintID steer_motor2 = p.addSlidingHingeConstraint(chasis_r, steering_block2_r, rear_wheel2_position, mthz::Vec3(0, -1, 0), 350, 350, -hard_suspension_limit_dist, hard_suspension_limit_dist);

		p.addSpring(chasis_r, suspension_block1_r, front_wheel1_position + mthz::Vec3(0, spring_dist, 0), front_wheel1_position, spring_damping, spring_stiffness);
		p.addSpring(chasis_r, suspension_block2_r, front_wheel2_position + mthz::Vec3(0, spring_dist, 0), front_wheel2_position, spring_damping, spring_stiffness);
		p.addSpring(chasis_r, steering_block1_r, rear_wheel1_position + mthz::Vec3(0, spring_dist, 0), rear_wheel1_position, spring_damping, spring_stiffness);
		p.addSpring(chasis_r, steering_block2_r, rear_wheel2_position + mthz::Vec3(0, spring_dist, 0), rear_wheel2_position, spring_damping, spring_stiffness);

		phyz::ConstraintID power_motor1 = p.addHingeConstraint(front_wheel1_r, suspension_block1_r, front_wheel1_position, mthz::Vec3(1, 0, 0));
		phyz::ConstraintID power_motor2 = p.addHingeConstraint(front_wheel2_r, suspension_block2_r, front_wheel2_position, mthz::Vec3(1, 0, 0));

		p.addHingeConstraint(steering_block1_r, rear_wheel1_r, rear_wheel1_position, mthz::Vec3(-1, 0, 0));
		p.addHingeConstraint(steering_block2_r, rear_wheel2_r, rear_wheel2_position, mthz::Vec3(1, 0, 0));
		
		p.disallowCollision(rear_wheel1_r, chasis_r);
		p.disallowCollision(rear_wheel2_r, chasis_r);

		suspension_block1_r->setNoCollision(true);
		suspension_block2_r->setNoCollision(true);
		steering_block1_r->setNoCollision(true);
		steering_block2_r->setNoCollision(true);

		bodies.push_back({ fromGeometry(chasis), chasis_r });
		bodies.push_back({ fromGeometry(front_wheel1), front_wheel1_r });
		bodies.push_back({ fromGeometry(front_wheel2), front_wheel2_r });
		bodies.push_back({ fromGeometry(rear_wheel1), rear_wheel1_r });
		bodies.push_back({ fromGeometry(rear_wheel2), rear_wheel2_r });
		/*bodies.push_back({ fromGeometry(steering_block1), steering_block1_r });
		bodies.push_back({ fromGeometry(steering_block2), steering_block2_r });
		bodies.push_back({ fromGeometry(suspension_block1), suspension_block1_r });
		bodies.push_back({ fromGeometry(suspension_block2), suspension_block2_r });*/
		bodies.push_back({ fromGeometry(mast), mast_r });

		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(15, 3, 0);
		mthz::Quaternion orient(PI / 2.0, mthz::Vec3(0, 1, 0));
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

			mthz::Vec3 oriented_forward = chasis_r->getOrientation().applyRotation(forward_dir);

			if (rndr::getKeyDown(GLFW_KEY_I)) {
				bool breaking = oriented_forward.dot(chasis_r->getVel()) < 0;

				p.setMotorTargetVelocity(power_motor1, breaking? break_torque : power_torque, forward_speed);
				p.setMotorTargetVelocity(power_motor2, breaking ? break_torque : power_torque, forward_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				bool breaking = oriented_forward.dot(chasis_r->getVel()) > 0;

				p.setMotorTargetVelocity(power_motor1, breaking ? break_torque : power_torque, -reverse_speed);
				p.setMotorTargetVelocity(power_motor2, breaking ? break_torque : power_torque, -reverse_speed);
			}
			else {
				p.setMotorTargetVelocity(power_motor1, power_torque, 0);
				p.setMotorTargetVelocity(power_motor2, power_torque, 0);
			}

			if (rndr::getKeyDown(GLFW_KEY_H)) {
				p.setMotorTargetVelocity(mast_motor, mast_torque, 0.15);
			}
			else if (rndr::getKeyDown(GLFW_KEY_F)) {
				p.setMotorTargetVelocity(mast_motor, mast_torque, -0.15);
			}
			else {
				p.setMotorTargetVelocity(mast_motor, mast_torque, 0);
			}

			if (rndr::getKeyDown(GLFW_KEY_J)) {
				p.setMotorTargetPosition(steer_motor1, steer_torque, -steer_angle);
				p.setMotorTargetPosition(steer_motor2, steer_torque, -steer_angle);
			}
			else if (rndr::getKeyDown(GLFW_KEY_L)) {
				p.setMotorTargetPosition(steer_motor1, steer_torque, steer_angle);
				p.setMotorTargetPosition(steer_motor2, steer_torque, steer_angle);
			}
			else {
				p.setMotorTargetPosition(steer_motor1, steer_torque, 0);
				p.setMotorTargetPosition(steer_motor2, steer_torque, 0);
			}

			//printf("%f, %f, %f\n", chasis_r->getCOM().x, chasis_r->getCOM().y, chasis_r->getCOM().z);

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
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
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