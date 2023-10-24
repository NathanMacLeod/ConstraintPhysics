#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class WreckingBallDemo : public DemoScene {
public:
	WreckingBallDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~WreckingBallDemo() override {

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
		p.setSleepingEnabled(true);
		p.setPGSIterations(45, 35);
		p.setGlobalConstraintForceMixing(0.1);

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

		//*********************
		//****WRECKING BALL****
		//*********************
		mthz::Vec3 crane_tower_pos(0, 0, 4);
		double crane_tower_height = 4;
		double crane_tower_width = 2;
		double leg_width = 0.33;

		phyz::ConvexUnionGeometry leg1 = phyz::ConvexUnionGeometry::box(crane_tower_pos + mthz::Vec3(-crane_tower_width / 2.0, 0, -crane_tower_width / 2.0), leg_width, crane_tower_height, leg_width);
		phyz::ConvexUnionGeometry leg2 = phyz::ConvexUnionGeometry::box(crane_tower_pos + mthz::Vec3(crane_tower_width / 2.0 - leg_width, 0, -crane_tower_width / 2.0), leg_width, crane_tower_height, leg_width);
		phyz::ConvexUnionGeometry leg3 = phyz::ConvexUnionGeometry::box(crane_tower_pos + mthz::Vec3(crane_tower_width / 2.0 - leg_width, 0, crane_tower_width / 2.0 - leg_width), leg_width, crane_tower_height, leg_width);
		phyz::ConvexUnionGeometry leg4 = phyz::ConvexUnionGeometry::box(crane_tower_pos + mthz::Vec3(-crane_tower_width / 2.0, 0, crane_tower_width / 2.0 - leg_width), leg_width, crane_tower_height, leg_width);
		phyz::ConvexUnionGeometry tower_base = phyz::ConvexUnionGeometry::box(crane_tower_pos + mthz::Vec3(-crane_tower_width / 2.0, crane_tower_height, -crane_tower_width / 2.0), crane_tower_width, leg_width, crane_tower_width);
		double cabin_height = crane_tower_width * 0.67;
		phyz::ConvexUnionGeometry cabin = phyz::ConvexUnionGeometry::box(crane_tower_pos + mthz::Vec3(-crane_tower_width, crane_tower_height + leg_width, -crane_tower_width / 2.0), crane_tower_width * 1.5, cabin_height, crane_tower_width, phyz::Material::modified_density(2));

		double crane_length = crane_tower_height;
		double crane_height = crane_length / 8.0;
		double crane_width = crane_height / 3.0;
		mthz::Vec3 crane_pos = crane_tower_pos + mthz::Vec3(crane_tower_width/2.0, crane_tower_height + leg_width + cabin_height/3.0, 0);
		phyz::ConvexUnionGeometry crane = phyz::ConvexUnionGeometry::box(crane_pos + mthz::Vec3(-crane_height / 2.0, -crane_height / 2.0, -crane_width/2.0), crane_length + crane_height / 2.0, crane_height, crane_width, phyz::Material::modified_density(2));
		
		bodies.push_back({ fromGeometry(leg1), p.createRigidBody(leg1, phyz::RigidBody::FIXED) });
		bodies.push_back({ fromGeometry(leg2), p.createRigidBody(leg2, phyz::RigidBody::FIXED) });
		bodies.push_back({ fromGeometry(leg3), p.createRigidBody(leg3, phyz::RigidBody::FIXED) });
		bodies.push_back({ fromGeometry(leg4), p.createRigidBody(leg4, phyz::RigidBody::FIXED) });

		phyz::RigidBody* base_r = p.createRigidBody(tower_base, phyz::RigidBody::FIXED);
		phyz::RigidBody* cabin_r = p.createRigidBody(cabin);
		phyz::RigidBody* crane_r = p.createRigidBody(crane);
		bodies.push_back({ fromGeometry(tower_base), base_r });
		bodies.push_back({ fromGeometry(cabin), cabin_r });
		bodies.push_back({ fromGeometry(crane), crane_r });
		
		double chain_width = crane_width;
		double chain_height = chain_width * 4;
		phyz::ConvexUnionGeometry chain = phyz::ConvexUnionGeometry::box(mthz::Vec3(-chain_width / 2.0, 0, -chain_width / 2.0), chain_width, -chain_height, chain_width, phyz::Material::modified_density(2));
		int n_chain = 6;
		mthz::Vec3 chain_start_pos = crane_pos + mthz::Vec3(crane_length, -crane_height / 2.0, 0);
		phyz::RigidBody* previous_chain = nullptr;

		for (int i = 0; i < n_chain; i++) {
			mthz::Vec3 this_chain_pos = chain_start_pos - mthz::Vec3(0, i * chain_height, 0);
			phyz::ConvexUnionGeometry this_chain = chain.getTranslated(this_chain_pos);
			phyz::RigidBody* this_chain_r = p.createRigidBody(this_chain);

			if (i == 0) {
				p.addBallSocketConstraint(crane_r, this_chain_r, this_chain_pos);
			}
			else {
				p.addBallSocketConstraint(previous_chain, this_chain_r, this_chain_pos);
			}

			bodies.push_back({ fromGeometry(this_chain), this_chain_r });
			previous_chain = this_chain_r;
		}

		mthz::Vec3 final_chain_pos = chain_start_pos + mthz::Vec3(0, -n_chain * chain_height, 0);
		double ball_size = 0.65;
		phyz::ConvexUnionGeometry ball = phyz::ConvexUnionGeometry::psuedoSphere(final_chain_pos + mthz::Vec3(0, -ball_size, 0), ball_size, 15, 20, phyz::Material::modified_density(1));//phyz::ConvexUnionGeometry::sphere(final_chain_pos + mthz::Vec3(0, -ball_size, 0), ball_size, phyz::Material::modified_density(1));
		phyz::RigidBody* ball_r = p.createRigidBody(ball);
		p.addBallSocketConstraint(ball_r, previous_chain, final_chain_pos);
		bodies.push_back({ fromGeometry(ball), ball_r });

		double rotate_torque = 0.2 * 90;
		phyz::ConstraintID rotate_motor = p.addHingeConstraint(base_r, cabin_r, crane_tower_pos + mthz::Vec3(0, crane_tower_height + crane_tower_width, 0), mthz::Vec3(0, 1, 0));
		double lift_torque = 60 * 90;
		phyz::ConstraintID lift_motor = p.addHingeConstraint(cabin_r, crane_r, crane_pos, mthz::Vec3(0, 0, 1), -PI / 2.0, PI / 4.0);

		//*************
		//****TOWER****
		//*************
		mthz::Vec3 tower_pos(8, 0, 0);
		double tower_width = 4;
		double tower_story_height = 1.25;
		double floor_height = 0.25;
		double pillar_width = 0.3;
		int n_stories = 9;
		phyz::ConvexUnionGeometry pillar = phyz::ConvexUnionGeometry::box(mthz::Vec3(), pillar_width, tower_story_height, pillar_width);
		phyz::ConvexUnionGeometry floor_plate = phyz::ConvexUnionGeometry::box(mthz::Vec3(), tower_width / 2.0, floor_height, tower_width / 2.0);

		std::vector<phyz::RigidBody*> tower_bodies;

		for (int i = 0; i < n_stories; i++) {
			mthz::Vec3 story_pos = tower_pos + mthz::Vec3(0, i * (floor_height + tower_story_height), 0);
			phyz::ConvexUnionGeometry pillar1 = pillar.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, 0, - tower_width / 2.0));
			phyz::ConvexUnionGeometry pillar2 = pillar.getTranslated(story_pos + mthz::Vec3(-pillar_width / 2.0, 0, -tower_width / 2.0));
			phyz::ConvexUnionGeometry pillar3 = pillar.getTranslated(story_pos + mthz::Vec3(tower_width / 2.0 - pillar_width, 0, -tower_width / 2.0));
			phyz::ConvexUnionGeometry pillar4 = pillar.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, 0, -pillar_width / 2.0));
			phyz::ConvexUnionGeometry pillar5 = pillar.getTranslated(story_pos + mthz::Vec3(-pillar_width / 2.0, 0, -pillar_width / 2.0));
			phyz::ConvexUnionGeometry pillar6 = pillar.getTranslated(story_pos + mthz::Vec3(tower_width / 2.0 - pillar_width, 0, -pillar_width / 2.0));
			phyz::ConvexUnionGeometry pillar7 = pillar.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, 0, tower_width / 2.0 - pillar_width));
			phyz::ConvexUnionGeometry pillar8 = pillar.getTranslated(story_pos + mthz::Vec3(-pillar_width / 2.0, 0, tower_width / 2.0 - pillar_width));
			phyz::ConvexUnionGeometry pillar9 = pillar.getTranslated(story_pos + mthz::Vec3(tower_width / 2.0 - pillar_width, 0, tower_width / 2.0 - pillar_width));

			phyz::ConvexUnionGeometry floor_plate1 = floor_plate.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, tower_story_height, -tower_width / 2.0));
			phyz::ConvexUnionGeometry floor_plate2 = floor_plate.getTranslated(story_pos + mthz::Vec3(0, tower_story_height, -tower_width / 2.0));
			phyz::ConvexUnionGeometry floor_plate3 = floor_plate.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, tower_story_height, 0));
			phyz::ConvexUnionGeometry floor_plate4 = floor_plate.getTranslated(story_pos + mthz::Vec3(0, tower_story_height, 0));

			phyz::RigidBody* pillar1_r = p.createRigidBody(pillar1);
			phyz::RigidBody* pillar2_r = p.createRigidBody(pillar2);
			phyz::RigidBody* pillar3_r = p.createRigidBody(pillar3);
			phyz::RigidBody* pillar4_r = p.createRigidBody(pillar4);
			phyz::RigidBody* pillar5_r = p.createRigidBody(pillar5);
			phyz::RigidBody* pillar6_r = p.createRigidBody(pillar6);
			phyz::RigidBody* pillar7_r = p.createRigidBody(pillar7);
			phyz::RigidBody* pillar8_r = p.createRigidBody(pillar8);
			phyz::RigidBody* pillar9_r = p.createRigidBody(pillar9);

			phyz::RigidBody* floor_plate1_r = p.createRigidBody(floor_plate1);
			phyz::RigidBody* floor_plate2_r = p.createRigidBody(floor_plate2);
			phyz::RigidBody* floor_plate3_r = p.createRigidBody(floor_plate3);
			phyz::RigidBody* floor_plate4_r = p.createRigidBody(floor_plate4);

			std::vector<phyz::RigidBody*> new_bodies = {
				pillar1_r, pillar2_r, pillar3_r, pillar4_r, pillar5_r, pillar6_r, pillar7_r, pillar8_r, pillar9_r,
				floor_plate1_r, floor_plate2_r, floor_plate3_r, floor_plate4_r
			};

			tower_bodies.insert(tower_bodies.end(), new_bodies.begin(), new_bodies.end());

			bodies.push_back({ fromGeometry(pillar1), pillar1_r });
			bodies.push_back({ fromGeometry(pillar2), pillar2_r });
			bodies.push_back({ fromGeometry(pillar3), pillar3_r });
			bodies.push_back({ fromGeometry(pillar4), pillar4_r });
			bodies.push_back({ fromGeometry(pillar5), pillar5_r });
			bodies.push_back({ fromGeometry(pillar6), pillar6_r });
			bodies.push_back({ fromGeometry(pillar7), pillar7_r });
			bodies.push_back({ fromGeometry(pillar8), pillar8_r });
			bodies.push_back({ fromGeometry(pillar9), pillar9_r });

			bodies.push_back({ fromGeometry(floor_plate1), floor_plate1_r });
			bodies.push_back({ fromGeometry(floor_plate2), floor_plate2_r });
			bodies.push_back({ fromGeometry(floor_plate3), floor_plate3_r });
			bodies.push_back({ fromGeometry(floor_plate4), floor_plate4_r });
		}
		

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 2 * crane_tower_height, 10);
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

			/*if (rndr::getKeyPressed(GLFW_KEY_T)) {
				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

				if (hit_info.did_hit) {
					hit_info.hit_object->applyImpulse(camera_dir * 5, hit_info.hit_position);
				}
			}*/

			if (rndr::getKeyDown(GLFW_KEY_I)) {
				p.setMotorTargetVelocity(lift_motor, lift_torque, -1);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				p.setMotorTargetVelocity(lift_motor, lift_torque, 1);
			}
			else {
				p.setMotorTargetVelocity(lift_motor, lift_torque, 0);
			}
			if (rndr::getKeyDown(GLFW_KEY_J)) {
				p.setMotorTargetVelocity(rotate_motor, rotate_torque, -3);
			}
			else if (rndr::getKeyDown(GLFW_KEY_L)) {
				p.setMotorTargetVelocity(rotate_motor, rotate_torque, 3);
			}
			else {
				p.setMotorTargetVelocity(rotate_motor, rotate_torque, 0);
			}
			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				for (phyz::RigidBody* r : tower_bodies) {
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

			double aspect_ratio = (double)properties.window_height / properties.window_width;
			//shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(mthz::Vec3(0, 0, 0), cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0 * aspect_ratio, 60.0));
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