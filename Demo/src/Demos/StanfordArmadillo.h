#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class StanfordArmadillo : public DemoScene {
public:
	StanfordArmadillo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~StanfordArmadillo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"3D model source", "The original 3d model is \"The Cyclone\" from mroek: https://pinshape.com/items/17577-3d-printed-the-cyclone-triple-lift-triple-track-marble-machine"},
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
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
		p.setPGSIterations(15, 5);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;

		//************************
		//*******BASE PLATE*******
		//************************
		double s = 100;
		phyz::ConvexUnionGeometry geom2 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-s / 2, -1, -s / 2), s, 2, s);
		Mesh m2 = fromGeometry(geom2);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, phyz::RigidBody::FIXED);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		r2->translateSoCOMAtPosition(mthz::Vec3(0, -5, 0));
		bodies.push_back({ m2, r2 });

		phyz::Mesh armadillo = phyz::readOBJ("resources/mesh/armadillo.obj", 3);
		phyz::MeshInput armadillo_input = phyz::generateMeshInputFromMesh(armadillo, mthz::Vec3(0, 0, 0));

		mthz::Vec3 center(0, 0, 0);

		phyz::RigidBody* armadillo_r = p.createRigidBody(armadillo_input, false);
		bodies.push_back({ fromStaticMeshInput(armadillo_input, color{ 1.0f, 0.45f, 0.35f, 0.25f, 0.75f, 0.63f, 51.2f }), armadillo_r });

		armadillo_r->setVel(mthz::Vec3(0, 0, 0.5));

		armadillo_r->setToPosition(center);
		armadillo_r->setOrientation(mthz::Quaternion(-PI / 2, mthz::Vec3(1, 0, 0)));
		
		double block_length = 1;
		double block_width = 0.5;
		
		mthz::Vec3 wall_pos(0, -4, 5);
		int wall_width = 8;
		int wall_height = 15;
		int layers = 2;

		phyz::ConvexUnionGeometry brick = phyz::ConvexUnionGeometry::box(mthz::Vec3(), block_length, block_width, block_width);

		for (int l = 0; l < layers; l++) {
			for (int i = 0; i < wall_height; i++) {
				for (int j = 0; j < wall_width; j++) {
					mthz::Vec3 pos = wall_pos + mthz::Vec3(0.5 * block_length * (i % 2) + j * block_length - wall_width * block_length / 2, i * block_width, 2 * l * block_width);
					phyz::ConvexUnionGeometry brick_translated = brick.getTranslated(pos);
					bodies.push_back({ fromGeometry(brick_translated), p.createRigidBody(brick_translated) });
				}
			}
		}

		phyz::ConvexUnionGeometry sideways_brick = phyz::ConvexUnionGeometry::box(mthz::Vec3(), block_width, block_width, 1.5 * block_length);
		for (int i = 0; i < wall_height; i++) {
			mthz::Vec3 pos = wall_pos + mthz::Vec3((i % 2 == 1? -1 : 1) * wall_width * block_length / 2, i * block_width, 0);
			phyz::ConvexUnionGeometry b = sideways_brick.getTranslated(pos);
			bodies.push_back({ fromGeometry(b), p.createRigidBody(b) });
		}


		/*phyz::ConvexUnionGeometry marble = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.36);

		std::vector<phyz::RigidBody*> marbles;
		for (mthz::Vec3 marble_pos : getMarbleStartPositions()) {
			phyz::ConvexUnionGeometry marble_p = marble.getTranslated(marble_pos);
			phyz::RigidBody* marble_r = p.createRigidBody(marble_p);
			bodies.push_back({ fromGeometry(marble_p), marble_r });
			marbles.push_back(marble_r);
		}*/


		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 2, 15);
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

			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

				if (hit_info.did_hit) hit_info.hit_object->applyImpulse(camera_dir * 2, hit_info.hit_position);
			}

			t += fElapsedTime;


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
			shader.setUniform3f("u_ambient_light", 0.7f, 0.7f, 0.7f);
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

