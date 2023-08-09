#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class DebugDemo : public DemoScene {
public:
	DebugDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~DebugDemo() override {

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

		bool lock_cam = true;

		std::vector<Mesh> static_meshes;
		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;
		
		mthz::Vec3 center = mthz::Vec3(0, -2, -10);
		int grid_count = 360;
		double grid_size = 0.25;
		phyz::Mesh dragon = phyz::readOBJ("resources/mesh/xyzrgb_dragon.obj", 0.1);
		//phyz::Mesh cow = phyz::readOBJ("resources/mesh/cow.obj", 2.0);
		phyz::MeshInput dragon_input = phyz::generateMeshInputFromMesh(dragon, center + mthz::Vec3(0, 4, 0));
		phyz::MeshInput grid = phyz::generateGridMeshInput(grid_count, grid_count, grid_size, center + mthz::Vec3(-grid_count * grid_size / 2.0, 0, -grid_count * grid_size / 2.0));//phyz::generateRadialMeshInput(center, 8, 100, 1);

		for (mthz::Vec3& v : grid.points) {
			//v.y += 0.5 * 2 * (0.5 - frand());
			//v.y += 0.0055 * (v - center).magSqrd();
			//v.y += 3 -0.5 * (v - center).mag() + 0.02 * (v - center).magSqrd();
			v.y += cos((v - center).mag() / 2.33);
		}
		for (phyz::TriIndices& t : grid.triangle_indices) {
			t.material = phyz::Material::ice();
		}

		phyz::RigidBody* gr = p.createRigidBody(grid);
		static_meshes.push_back(fromStaticMeshInput(grid, color{ 0.4, 0.2, 0.7}));

		phyz::RigidBody* r = p.createRigidBody(dragon_input);
		//r->setVel(mthz::Vec3(0.25, 0, 0));
		//bodies.push_back({ fromStaticMeshInput(dragon_input, color{ 1.0, 0.84, 0.0, 0.25, 0.75, 0.63, 51.2 }), r });
		static_meshes.push_back(fromStaticMeshInput(dragon_input, color{1.0, 0.84, 0.0, 0.25, 0.75, 0.63, 51.2 }));

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos;
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 60.0;
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

			if (rndr::getKeyPressed(GLFW_KEY_G)) {
				double block_size = 1.0;
				double block_speed = 15;

				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(pos + mthz::Vec3(-block_size / 2.0, -block_size / 2.0, -block_size / 2.0), block_size, block_size, block_size/*, phyz::Material::ice()*/);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				block_r->setVel(camera_dir * block_speed);

				bodies.push_back({ fromGeometry(block), block_r });
			}

			if (rndr::getKeyPressed(GLFW_KEY_B)) {
				double width = 1.2;
				phyz::ConvexUnionGeometry box = phyz::ConvexUnionGeometry::box(center + mthz::Vec3(-width / 2.0 , 6, -width / 2.0), width, width, width, phyz::Material::ice());
				phyz::RigidBody* box_r = p.createRigidBody(box);
				//box_r->setVel(mthz::Vec3(4, 0, 4));
				bodies.push_back({ fromGeometry(box), box_r });
			}

			if (rndr::getKeyPressed(GLFW_KEY_V)) {
				double block_size = 1.0;
				double block_speed = 15;

				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::sphere(pos, block_size / 2.0/*, phyz::Material::ice()*/);
				phyz::RigidBody* block_r = p.createRigidBody(block, phyz::RigidBody::KINEMATIC);

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				block_r->setVel(camera_dir * block_speed);

				bodies.push_back({ fromGeometry(block), block_r });
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
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 500.0, 2.0, 2.0 * aspect_ratio, 120.0));
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

			for (const Mesh& m : static_meshes) {

				Mesh transformed_mesh = getTransformed(m, mthz::Vec3(), mthz::Quaternion(), cam_pos, cam_orient, false, color{1.0f, 0.0f, 0.0f});

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