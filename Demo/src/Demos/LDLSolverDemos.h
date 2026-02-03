#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "Common.h"

class LDLSolverDemos : public DemoScene {
public:
	LDLSolverDemos(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~LDLSolverDemos() override {

	}

	std::map<std::string, std::string> askParameters() override {
		std::map<std::string, std::string> out;

		out["which_demo"] = pickParameterFromOptions(
			"Choose from one of the demos that demonstrate the advantages of the LDL-PGS solver: 1) Scissor lift, 2) High mass ratio, 3) Bridge: ", { "1", "2", "3" }
		);

		/*out["use_ldl"] = pickParameterFromOptions(
			"Choose to run the demo using 1) Normal PGS solver; 2) LDL-PGS solver: ", { "1", "2" }
		);*/

		// for the high mass ratio demo with LDL, ask whether to use a stable or unstable tick rate
		//if (out["which_demo"] == "2" && out["use_ldl"] == "2") {
		//	out["unstable_tickrate"] = pickParameterFromOptions("Choose to run the scene 1) high tick rate, so scene runs stable 2) low tick rate, to see the simulation fail: ", { "1", "2" });
		//}
		//else {
		//	//default to high tick rate
		//	out["unstable_tickrate"] = "1";
		//}

		return out;
	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"I. K", "Raise / Lower scissor lift, if present"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	enum LDLDemoType { SCISSOR_LIFT, HIGH_MASS_RATIO, BRIDGE };

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "LDL Solver Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		//p.setSleepingEnabled(false);
		p.setPGSIterations(1, 1, 1);
		p.setSubstepCount(16);
		p.setGlobalConstraintForceMixing(0.0000001);

		/*if (parameters["use_ldl"] == "2") {
			p.setPGSIterations(5, 2, 1);
		}
		else {
			p.setPGSIterations(5, 2, 0);
		}
		p.setGlobalConstraintForceMixing(0.000025);*/


		LDLDemoType demo_type;
		std::string demo_response = parameters["which_demo"];
		if      (demo_response == "1") demo_type = SCISSOR_LIFT;
		else if (demo_response == "2") demo_type = HIGH_MASS_RATIO;
		else if (demo_response == "3") demo_type = BRIDGE;

		double timestep = 1 / 60.0;
		/*if (parameters["unstable_tickrate"] == "2") {
			timestep = 1 / 120.0;
		}*/
		p.setStep_time(timestep);

		bool lock_cam = false;
		mthz::Vec3 pos(15, 3, 0);
		mthz::Quaternion orient(PI / 2.0, mthz::Vec3(0, 1, 0));

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


		ScissorLiftConstruct scissor;
		if (demo_type == SCISSOR_LIFT) {
			//************************
			//******Scissor Lift******
			//************************
			mthz::Vec3 scissor_lift_pos = mthz::Vec3(0, 2, 0);

			scissor = create_scissor_lift(&p, &bodies, scissor_lift_pos);
		}
		else if (demo_type == HIGH_MASS_RATIO) {
			pos.x += 160;
			//pos.y += 70;
			pos.y = 120;

			mthz::Vec3 chain_pos(0, 230, 0);
			double chain_width = 0.31;
			double chain_height = 1;

			double attach_box_size = 1;
			phyz::ConvexUnionGeometry attach_box = phyz::ConvexUnionGeometry::box(chain_pos - mthz::Vec3(1, 1, 1) * attach_box_size / 2, attach_box_size, attach_box_size, attach_box_size);
			phyz::ConvexUnionGeometry chain = phyz::ConvexUnionGeometry::box(mthz::Vec3(-chain_width / 2.0, 0, -chain_width / 2.0), chain_width, -chain_height, chain_width, phyz::Material::modified_density(2));

			phyz::RigidBody* attach_box_r = p.createRigidBody(attach_box, phyz::RigidBody::FIXED);
			int n_chain = 120;
			phyz::RigidBody* previous_chain = nullptr;

			for (int i = 0; i < n_chain; i++) {
				mthz::Vec3 this_chain_pos = chain_pos - mthz::Vec3(0, i * chain_height, 0);
				phyz::ConvexUnionGeometry this_chain = chain.getTranslated(this_chain_pos);

				phyz::RigidBody* this_chain_r = p.createRigidBody(this_chain);

				if (i == 0) {
					p.addBallSocketConstraint(attach_box_r, this_chain_r, this_chain_pos);
					
				}
				else {
					p.addBallSocketConstraint(previous_chain, this_chain_r, this_chain_pos);
				}

				bodies.push_back({ fromGeometry(this_chain), this_chain_r });
				previous_chain = this_chain_r;
			}

			mthz::Vec3 final_chain_pos = chain_pos + mthz::Vec3(0, -n_chain * chain_height, 0);
			double ball_radius = 7;
			mthz::Vec3 ball_pos = final_chain_pos + mthz::Vec3(0, -ball_radius, 0);
			phyz::ConvexUnionGeometry ball = phyz::ConvexUnionGeometry::sphere(ball_pos, ball_radius, phyz::Material::modified_density(2));
			phyz::RigidBody* ball_r = p.createRigidBody(ball);

			bodies.push_back({ fromGeometry(attach_box), attach_box_r });

			ball_r->setVel(mthz::Vec3(0, 0, 7));

			p.addBallSocketConstraint(ball_r, previous_chain, final_chain_pos);
			bodies.push_back({ fromGeometry(ball), ball_r });

			//works well with a low CFM
			//p.setHolonomicSolverCFM(0);
		}
		else {
			pos.y += 30;
			pos.x += 20;

			double bridge_panel_width = 0.5;
			double bridge_panel_length = 4.5;
			double bridge_panel_thickness = 0.2;
			double bridge_panel_gap = 0.05;

			int n_bridge_panels = 60;
			double bridge_width = n_bridge_panels * (bridge_panel_width + bridge_panel_gap) - bridge_panel_gap;
			mthz::Vec3 bridge_pos(0, 30, 0);

			phyz::RigidBody* previous_panel = nullptr;
			for (int i = 0; i < n_bridge_panels; i++) {
				mthz::Vec3 panel_pos = bridge_pos + mthz::Vec3(0, 0, (bridge_panel_width + bridge_panel_gap) * i - bridge_width / 2);
				phyz::ConvexUnionGeometry panel = phyz::ConvexUnionGeometry::box(panel_pos, bridge_panel_length, bridge_panel_thickness, bridge_panel_width);

				phyz::RigidBody::MovementType panel_movement = i == 0 || i + 1 == n_bridge_panels ? phyz::RigidBody::FIXED : phyz::RigidBody::DYNAMIC;
				phyz::RigidBody* panel_r = p.createRigidBody(panel, panel_movement);

				if (i != 0) {
					p.addHingeConstraint(panel_r, previous_panel, panel_pos + mthz::Vec3(bridge_panel_length / 2, 0, -bridge_panel_gap / 2), mthz::Vec3(1, 0, 0), 1000, 1000);
				}

				previous_panel = panel_r;
				bodies.push_back({ fromGeometry(panel), panel_r });
			}
		}

		//p.setPistonTargetVelocity(scissor_slider, slider_force, 0);
		//printf("current_pos: %f\n", p.getPistonPosition(scissor_slider));

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		p.setGravity(mthz::Vec3(0, -6.0, 0));

		int tick_count = 0;

		bool paused = false;

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
			if (demo_type == SCISSOR_LIFT && rndr::getKeyDown(GLFW_KEY_I)) {
				set_scissor_lift_movement_input(&p, &scissor, ScissorLiftConstruct::MovementState::RAISING);
			}
			else if (demo_type == SCISSOR_LIFT && rndr::getKeyDown(GLFW_KEY_K)) {
				set_scissor_lift_movement_input(&p, &scissor, ScissorLiftConstruct::MovementState::LOWERING);
			}
			else if (demo_type == SCISSOR_LIFT) {
				set_scissor_lift_movement_input(&p, &scissor, ScissorLiftConstruct::MovementState::FIXED);
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

			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				/*for (PhysBod p : bodies) {
					phyz::RigidBody* b = p.r;
					if (b->getMovementType() == phyz::RigidBody::FIXED) continue;

					mthz::Vec3 p = b->getCOM();
					mthz::Quaternion o = b->getOrientation();
					mthz::Vec3 v = b->getVel();
					mthz::Vec3 a = b->getAngVel();
					printf("pos: (%f, %f, %f); orient: (%f, %f, %f, %f); vel: (%f, %f, %f), ang_vel: (%f, %f, %f)\n",
						p.x, p.y, p.z,
						o.r, o.i, o.j, o.k,
						v.x, v.y, v.z,
						a.x, a.y, a.z
					);
				}*/

				p.timeStep();
				printf("%d\n", tick_count++);
			}

			t += fElapsedTime;

			if (rndr::getKeyPressed(GLFW_KEY_Y)) {
				lock_cam = !lock_cam;

			}

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				manager->deselectCurrentScene();
				return;
			}

			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				paused = !paused;
			}

			double slow_factor = 1;

			if (!paused) {
				phyz_time += fElapsedTime;
			}
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > slow_factor * timestep) {
				phyz_time -= slow_factor * timestep;
				p.timeStep();
				tick_count++;
			}

			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = pos;
			mthz::Quaternion cam_orient = orient;

			mthz::Vec3 pointlight_pos(0.0, 225.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

			float aspect_ratio = (float)properties.window_height / properties.window_width;
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 550.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
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