#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class MarbleMachineDemo : public DemoScene {
public:
	MarbleMachineDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~MarbleMachineDemo() override {

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
		p.setSleepingEnabled(false);
		p.setPGSIterations(45, 35);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;

		phyz::Mesh marble_track = phyz::readOBJ("resources/mesh/marble_track.obj", 0.1);
		phyz::Mesh marble_screw = phyz::readOBJ("resources/mesh/marble_screw.obj", 0.1);
		phyz::MeshInput marble_track_input = phyz::generateMeshInputFromMesh(marble_track, mthz::Vec3(0, 0, 0));
		phyz::MeshInput marble_screw_input = phyz::generateMeshInputFromMesh(marble_screw, mthz::Vec3(0, 0, 0));

		mthz::Vec3 center(0, -14, -5);

		phyz::RigidBody* marble_track_r = p.createRigidBody(marble_track_input, true);
		bodies.push_back({ fromStaticMeshInput(marble_track_input, color{ 0.7, 0.45, 1.0, 0.25, 0.75, 0.63, 51.2 }), marble_track_r });

		phyz::RigidBody* marble_screw_r = p.createRigidBody(marble_screw_input, false);
		bodies.push_back({ fromStaticMeshInput(marble_screw_input, color{ 0.7, 0.45, 1.0, 0.25, 0.75, 0.63, 51.2 }), marble_screw_r });

		marble_track_r->setToPosition(center);
		marble_screw_r->setToPosition(center);
		marble_screw_r->setAngVel(mthz::Vec3(0, -0.2, 0));

		phyz::ConvexUnionGeometry marble = phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.36);

		std::vector<phyz::RigidBody*> marbles;
		for (mthz::Vec3 marble_pos : getMarbleStartPositions()) {
			phyz::ConvexUnionGeometry marble_p = marble.getTranslated(marble_pos);
			phyz::RigidBody* marble_r = p.createRigidBody(marble_p);
			bodies.push_back({ fromGeometry(marble_p), marble_r });
			marbles.push_back(marble_r);
		}


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

			if (rndr::getKeyPressed(GLFW_KEY_K)) {
				double block_size = 1.0;
				double block_speed = 2.5;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry marble_p = marble.getTranslated(pos);
				phyz::RigidBody* marble_r = p.createRigidBody(marble_p);

				marble_r->setVel(camera_dir * block_speed);
				bodies.push_back({ fromGeometry(marble_p), marble_r });
				marbles.push_back(marble_r);
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

			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				int i = 0;
				for (phyz::RigidBody* m : marbles) {
					mthz::Vec3 pos = m->getCOM();
					if (pos.y > -500) {
						if (++i % 3 == 0) printf("\n");
						printf("mthz::Vec3(%f, %f, %f), ", pos.x, pos.y, pos.z);
					}
					
				}
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

			double aspect_ratio = (double)properties.window_height / properties.window_width;
			//shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(mthz::Vec3(0, 0, 0), cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 500.0, 2.0, 2.0 * aspect_ratio, 60.0));
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

	private:

		std::vector<mthz::Vec3> getMarbleStartPositions() {
			return {
				mthz::Vec3(1.303999, -4.019785, -4.140381), mthz::Vec3(-0.459504, -8.207225, -5.997420),
				mthz::Vec3(-8.124196, -13.249696, -7.495247), mthz::Vec3(-0.449732, -9.343743, -6.003576), mthz::Vec3(1.090297, -5.938157, -4.904474),
				mthz::Vec3(1.088735, -7.073151, -4.901306), mthz::Vec3(1.091132, -8.201790, -4.910521), mthz::Vec3(1.087974, -9.346271, -4.887911),
				mthz::Vec3(1.082372, -10.480049, -4.887635), mthz::Vec3(1.073658, -11.609428, -4.891280), mthz::Vec3(1.075229, -12.746774, -4.888083),
				mthz::Vec3(1.099255, -13.868448, -4.910314), mthz::Vec3(1.199083, -13.688579, -5.598808), mthz::Vec3(-1.388372, -9.541868, 0.586517),
				mthz::Vec3(1.464455, -0.637097, -4.520370), mthz::Vec3(1.089075, -1.404308, -4.906768), mthz::Vec3(1.091781, -2.538449, -4.904237),
				mthz::Vec3(-0.444599, -11.611783, -5.993208), mthz::Vec3(-0.487640, -12.720814, -5.964834), mthz::Vec3(-0.453246, -13.847560, -5.946954),
				mthz::Vec3(-1.137233, -13.687554, -5.723293), mthz::Vec3(7.690416, -12.509987, -3.627865), mthz::Vec3(-5.703899, -8.357438, -9.521683),
				mthz::Vec3(-4.825352, -7.746756, -9.602428), mthz::Vec3(-4.911751, -7.006221, -8.447625), mthz::Vec3(-0.636710, -10.468176, -4.064324),
				mthz::Vec3(-0.432641, -0.609654, -6.368016), mthz::Vec3(4.097306, -5.473803, -9.158292), mthz::Vec3(-8.046052, -10.380216, -0.869022),
				mthz::Vec3(-0.625751, -11.606125, -4.111281), mthz::Vec3(-0.048188, -13.685457, -3.646140), mthz::Vec3(-0.630337, -12.741810, -4.120024),
				mthz::Vec3(-0.631530, -13.856987, -4.030910), mthz::Vec3(-0.441263, -1.413023, -5.994707), mthz::Vec3(-0.449520, -2.545040, -6.000423),
				mthz::Vec3(-0.474928, -3.669412, -6.003366), mthz::Vec3(-0.441166, -4.812376, -5.990251), mthz::Vec3(-7.126312, -6.177667, -10.210654),
				mthz::Vec3(-0.635825, -1.403401, -4.081762), mthz::Vec3(-0.634772, -2.536068, -4.077276), mthz::Vec3(-0.628578, -3.673158, -4.104637),
				mthz::Vec3(1.087202, -3.678614, -4.887311), mthz::Vec3(1.096501, -4.804529, -4.906520), mthz::Vec3(-0.634693, -4.801653, -4.073512),
				mthz::Vec3(-0.485672, -5.935715, -6.012440), mthz::Vec3(-0.635749, -5.935141, -4.070163), mthz::Vec3(-0.463495, -7.072522, -5.994482),
				mthz::Vec3(-0.636846, -7.068678, -4.067803), mthz::Vec3(-0.636621, -8.201414, -4.063844), mthz::Vec3(-0.637510, -9.335054, -4.064276),
				mthz::Vec3(-0.442152, -10.479276, -5.993174),
			}; 
		}
};
