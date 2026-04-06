#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class GobletDemo : public DemoScene {
public:
	GobletDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~GobletDemo() override {

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

		rndr::init(properties.window_width, properties.window_height, "Goblet Demo");

		phyz::PhysicsEngine p;
		if (properties.n_threads != 0) {
			p.enableMultithreading(properties.n_threads);
		}


		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;

		phyz::Mesh bunny_mesh = phyz::readOBJ("resources/mesh/bunny.obj", 35.0);
		phyz::MeshInput bunny_mesh_input = phyz::generateMeshInputFromMesh(bunny_mesh, mthz::Vec3(1, 8, 0));
		phyz::RigidBody* bunny_mesh_r = p.createRigidBody(bunny_mesh_input, false);
		bodies.push_back({ fromStaticMeshInput(bunny_mesh_input, color{ 0.4f, 1.0f, 0.8f, 0.5f, 0.5f, 0.63f, 51.2f }), bunny_mesh_r });

		bunny_mesh_r->setAngVel(mthz::Vec3(0, 0.501, 0));
		bunny_mesh_r->setOrientation(mthz::Quaternion(-0.60668597361459974948, 0.00000000000000000000, 0.79494158868391417982, 0.00000000000000000000));

		//phyz::Mesh goblet_mesh = phyz::readOBJ("resources/mesh/goblet.obj", 0.3);
		//phyz::MeshInput goblet_mesh_input = phyz::generateMeshInputFromMesh(goblet_mesh, mthz::Vec3(0, 0, 0));
		//phyz::RigidBody* goblet_mesh_r = p.createRigidBody(goblet_mesh_input);
		//bodies.push_back({ fromStaticMeshInput(goblet_mesh_input, color{ 0.8f, 1.0f, 1.0f, 0.5f, 0.5f, 0.63f, 51.2f }), goblet_mesh_r });

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		// adding rendering for contact points
		Mesh contact_ball_mesh = fromGeometry(phyz::ConvexUnionGeometry::merge(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.03), phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), 0.02, 0.1)), { 1.0, 0, 0 });

		bool color_by_manifold = false;
		struct Contact {
			mthz::Vec3 p;
			mthz::Vec3 n;
			color c;
		};
		std::vector<Contact> all_contact_points;

		p.registerCollisionAction(phyz::CollisionTarget::all(), phyz::CollisionTarget::all(), [&](phyz::RigidBody* b1, phyz::RigidBody* b2,
			const std::vector<phyz::Manifold>& manifold) {
				for (const phyz::Manifold& m : manifold) {
					for (phyz::ContactP p : m.points) {

						// generate a psuedo random color from the magicID- should make a clear visualization a contact is preserved by its magicID
						uint64_t uid = std::hash<phyz::MagicID>{}(p.magicID);
						color c = {
							((uid & 0x0000FF) >> 0) / 255.0f,
							((uid & 0x00FF00) >> 8) / 255.0f,
							((uid & 0xFF0000) >> 16) / 255.0f
						};

						all_contact_points.push_back({ p.pos, m.normal, c });
					}
				}
			}
		);

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 20, 15);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 60.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -16.0, 0));

		const int source_count = 1;
		double source_drop_rate = 1;
		double source_radius = 1.2;
		double source_y = 20;
		double ball_radius = 0.2;

		std::vector<mthz::Vec3> ball_sources;
		for (int i = 0; i < source_count; i++) {
			double theta = 2 * PI * i / source_count;
			ball_sources.push_back(mthz::Vec3(sin(theta) * source_radius, source_y, cos(theta) * source_radius));
		}

		double next_drop_timer = 1.0 / source_drop_rate;

		double delete_box_height = -50;
		double delete_box_dim = 10000;
		phyz::ConvexUnionGeometry delete_box = phyz::ConvexUnionGeometry::box(mthz::Vec3(-delete_box_dim / 2.0, delete_box_height - delete_box_dim, -delete_box_dim / 2.0), delete_box_dim, delete_box_dim, delete_box_dim);
		phyz::RigidBody* delete_box_r = p.createRigidBody(delete_box, phyz::RigidBody::FIXED);

		bool paused = false;

		while (rndr::render_loop(&fElapsedTime)) {

			if (!paused) {
				next_drop_timer -= fElapsedTime;
			}
			if (next_drop_timer < 0) {
				mthz::Quaternion orient = bunny_mesh_r->getOrientation();
				printf("%.20f, %.20f, %.20f, %.20f\n", orient.r, orient.i, orient.j, orient.k);
				next_drop_timer += 1.0 / source_drop_rate;
				
				for (mthz::Vec3 v : ball_sources) {
					phyz::ConvexUnionGeometry geom = phyz::ConvexUnionGeometry::sphere(v, ball_radius);
					//phyz::ConvexUnionGeometry geom = phyz::ConvexUnionGeometry::regDodecahedron(v, 2 * ball_radius);
					phyz::RigidBody* r = p.createRigidBody(geom);
					bodies.push_back({ fromGeometry(geom, color{1.0f, 0.4f, 0.4f}), r});

					p.registerCollisionAction(phyz::CollisionTarget::with(r), phyz::CollisionTarget::with(delete_box_r), [&, r](phyz::RigidBody* b1, phyz::RigidBody* b2, const std::vector<phyz::Manifold>& manifold) {
						p.removeRigidBody(r);

						for (int i = 0; i < bodies.size(); i++) {
							if (bodies[i].r == r) {
								bodies.erase(bodies.begin() + i);
								break;
							}
						}
					});
				}
			}

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

			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				paused = !paused;
			}

			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				mthz::Quaternion orient = bunny_mesh_r->getOrientation();
				printf("%f.20 %f.20 %f.20 %f.20\n", orient.r, orient.i, orient.j, orient.k);
				phyz_time += timestep;
				next_drop_timer -= timestep;
			}

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

			if (!paused) {
				phyz_time += fElapsedTime;
			}
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > timestep) {
				all_contact_points.clear();
				phyz_time -= timestep;
				p.timeStep();
			}


			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = pos;
			mthz::Quaternion cam_orient = orient;

			mthz::Vec3 pointlight_pos(0.0, 225.0, 0.0);
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

				Mesh transformed_mesh = getTransformed(contact_ball_mesh, c.p, rot, cam_pos, cam_orient, true, c.c);

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