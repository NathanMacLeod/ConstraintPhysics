#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class DebugDemo : public DemoScene {
public:
	DebugDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~DebugDemo() override {

	}

	static void addBrickRing(phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, mthz::Vec3 block_dim, double radius, double n_blocks, mthz::Vec3 pos, int n_layers) {
		phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(mthz::Vec3(0.0, 0.0, -block_dim.z / 2.0), block_dim.x, block_dim.y, block_dim.z);

		double dtheta = 2 * PI / n_blocks;
		for (int i = 0; i < n_layers; i++) {
			double theta_offset = i * dtheta / 2;
			for (double theta = 0; theta < 2 * PI; theta += dtheta) {
				if (theta != 0) continue;
				mthz::Quaternion rotation(theta + theta_offset, mthz::Vec3(0, 1, 0));

				mthz::Vec3 block_pos = pos + mthz::Vec3(0.0, block_dim.y * i, 0.0) + rotation.applyRotation(mthz::Vec3(radius, 0, 0));

				phyz::ConvexUnionGeometry block_oriented = block.getRotated(rotation).getTranslated(block_pos);
				phyz::RigidBody* r = p->createRigidBody(block_oriented);
				Mesh m = { fromGeometry(block_oriented) };
				bodies->push_back(PhysBod{ m, r });
			}
		}
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
		p.setPGSIterations(1, 1);
		p.setGlobalConstraintForceMixing(0.001);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;

		//mthz::Vec3 center = mthz::Vec3(0, -2, 0);
		//int grid_count = 360;
		//double grid_size = 0.5;
		//phyz::Mesh marble_track = phyz::readOBJ("resources/mesh/marble_track.obj", 0.1);
		//phyz::Mesh marble_screw = phyz::readOBJ("resources/mesh/marble_screw.obj", 0.1);
		//phyz::MeshInput marble_track_input = phyz::generateMeshInputFromMesh(marble_track, center);
		//phyz::MeshInput marble_screw_input = phyz::generateMeshInputFromMesh(marble_screw, center);
		//phyz::MeshInput grid = phyz::generateGridMeshInput(grid_count, grid_count, grid_size, center + mthz::Vec3(-grid_count * grid_size / 2.0, 0, -grid_count * grid_size / 2.0), phyz::Material::ice());//phyz::generateRadialMeshInput(center, 8, 100, 1);


		rndr::init(properties.window_width, properties.window_height, "Car Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

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

		//*****************
		//****OBSTACLES****
		//*****************
		double radius = 4;
		mthz::Vec3 block_dim(1, 1, 2);
		addBrickRing(&p, &bodies, block_dim, radius, 12, mthz::Vec3(0, -4, -22), 1);

		/*for (mthz::Vec3& v : grid.points) {
			v.y += 0.01 * 2 * (0.5 - frand());
		}
		for (phyz::TriIndices& t : grid.triangle_indices) {
			t.material = phyz::Material::ice();
		}*/

		Mesh contact_ball_mesh = fromGeometry(phyz::ConvexUnionGeometry::merge(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.03), phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), 0.02, 0.1)), {1.0, 0, 0});

		struct Contact {
			mthz::Vec3 p;
			mthz::Vec3 n;
		};
		std::vector<Contact> all_contact_points;

		p.registerCollisionAction(phyz::CollisionTarget::all(), phyz::CollisionTarget::all(), [&](phyz::RigidBody* b1, phyz::RigidBody* b2,
			const std::vector<phyz::Manifold>& manifold) {
				for (const phyz::Manifold& m : manifold) {
					for (phyz::ContactP p : m.points) {
						all_contact_points.push_back({ p.pos, m.normal });
					}
				}
			}
		);

		mthz::Vec3 pos(0, 2, 0);

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 60.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -6.0, 0));

		bool single_step_mode = false;

		bool paused = true;

		while (rndr::render_loop(&fElapsedTime)) {

			/*for (phyz::RigidBody* r : p.getBodies()) {
				if (r->getMovementType() == phyz::RigidBody::DYNAMIC) {
					printf("position: (%f, %f, %f); orietnation: (%f, %f, %f, %f); velocity: (%f, %f, %f), angular velocity: (%f, %f, %f)\n", r->getCOM().x, r->getCOM().y, r->getCOM().z, r->getOrientation().r, r->getOrientation().i, r->getOrientation().j, r->getOrientation().k, r->getVel().x, r->getVel().y, r->getVel().z, r->getAngVel().x, r->getAngVel().y, r->getAngVel().z);
				}
			}*/

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

			if (rndr::getKeyPressed(GLFW_KEY_B)) {
				single_step_mode = !single_step_mode;
			}
			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				all_contact_points.clear();
				p.timeStep();
			}

			if (rndr::getKeyPressed(GLFW_KEY_G)) {
				double block_size = 1.0;
				double block_speed = 2.5;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::cylinder(pos, 0.3, 3);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(camera_dir * block_speed);
				bodies.push_back({ fromGeometry(block), block_r });

			}

			if (rndr::getKeyPressed(GLFW_KEY_K)) {
				double block_size = 1.0;
				double block_speed = 2.5;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::sphere(pos, 0.36);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(camera_dir * block_speed);
				bodies.push_back({ fromGeometry(block), block_r });

			}

			if (rndr::getKeyPressed(GLFW_KEY_H)) {
				double block_size = 1.0;
				double block_speed = 8.5;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(pos, 1, 1, 1, phyz::Material::ice());
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(mthz::Vec3(block_speed, 0, 0));

				bodies.push_back({ fromGeometry(block), block_r });
			}

			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

				if (hit_info.did_hit) hit_info.hit_object->applyImpulse(camera_dir * 1, hit_info.hit_position);
			}

			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				paused = !paused;
			}

			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				p.timeStep();
				//printf("%d\n", tick_count++);
			}

			t += fElapsedTime;

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				manager->deselectCurrentScene();
				return;
			}

			if (!paused) {
				phyz_time += fElapsedTime;
				phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			}
			while (!single_step_mode && phyz_time > timestep) {
				all_contact_points.clear();
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
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			for (Contact c : all_contact_points) {
				mthz::Quaternion rot;
				double d = mthz::Vec3(0, 1, 0).dot(c.n);
				if (d < -0.99999) {
					rot = mthz::Quaternion(PI, mthz::Vec3(0, 0, 1));
				}
				else if (d < 0.99999) {
					mthz::Vec3 axis = mthz::Vec3(0, 1, 0).cross(c.n).normalize();
					double ang = acos(d);
					rot = mthz::Quaternion(ang, axis);
				}

				Mesh transformed_mesh = getTransformed(contact_ball_mesh, c.p, rot, cam_pos, cam_orient, false, color{1.0f, 0.0f, 0.0f});

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);
		}
	}
};