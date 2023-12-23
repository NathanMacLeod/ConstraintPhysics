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
		p.setPGSIterations(45, 35);
		p.setGlobalConstraintForceMixing(0.01);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;

		mthz::Vec3 center = mthz::Vec3(0, -2, 0);
		int grid_count = 360;
		double grid_size = 0.5;
		//phyz::Mesh dragon = phyz::readOBJ("resources/mesh/xyzrgb_dragon.obj", 0.2);
		//phyz::Mesh cow = phyz::readOBJ("resources/mesh/cow.obj", 2.0);
		//phyz::MeshInput dragon_input = phyz::generateMeshInputFromMesh(dragon, center + mthz::Vec3(0, 7, 0));
		//phyz::MeshInput grid = phyz::generateGridMeshInput(grid_count, grid_count, grid_size, center + mthz::Vec3(-grid_count * grid_size / 2.0, 0, -grid_count * grid_size / 2.0));//phyz::generateRadialMeshInput(center, 8, 100, 1);

		//for (mthz::Vec3& v : grid.points) {
			//v.y += 5 * 2 * (0.5 - frand()) - 10;
			//v.y += 0.0055 * (v - center).magSqrd();
			//v.y += 3 - 0.5 * (v - center).mag() + 0.02 * (v - center).magSqrd();
		//	v.y += cos((v - center).mag() / 2.33);
		//}
		//for (phyz::TriIndices& t : grid.triangle_indices) {
			//t.material = phyz::Material::ice();
		//}

		//************************
		//*******BASE PLATE*******
		//************************
		double s = 500;
		phyz::ConvexUnionGeometry geom2 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-s / 2, -2, -s / 2), s, 2, s);
		Mesh m2 = fromGeometry(geom2);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, phyz::RigidBody::FIXED);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		bodies.push_back({ m2, r2 });

		/*{
			mthz::Vec3 pos(0, 2, 0);
			double width = 1;
			double length = 2;

			phyz::ConvexUnionGeometry box1 = phyz::ConvexUnionGeometry::box(pos, length, width, width);
			phyz::RigidBody* r1 = p.createRigidBody(box1);
			phyz::ConvexUnionGeometry box2 = phyz::ConvexUnionGeometry::box(pos + mthz::Vec3(length, 0, 0), length, width, width);
			phyz::RigidBody* r2 = p.createRigidBody(box2);
			phyz::ConvexUnionGeometry box3 = phyz::ConvexUnionGeometry::box(pos + mthz::Vec3(2 * length, 0, 0), length, width, width);
			phyz::RigidBody* r3 = p.createRigidBody(box3);
			phyz::ConvexUnionGeometry box4 = phyz::ConvexUnionGeometry::box(pos + mthz::Vec3(3 * length, 0, 0), length, width, width);
			phyz::RigidBody* r4 = p.createRigidBody(box4);

			p.addHingeConstraint(r1, r2, pos + mthz::Vec3(length, width / 2.0, width / 2.0), mthz::Vec3(0, 1, 0));
			p.addHingeConstraint(r2, r3, pos + mthz::Vec3(2 * length, width / 2.0, width / 2.0), mthz::Vec3(0, 0, 1));
			p.addHingeConstraint(r3, r4, pos + mthz::Vec3(3 * length, width / 2.0, width / 2.0), mthz::Vec3(1, 0, 0));

			bodies.push_back({ fromGeometry(box1), r1 });
			bodies.push_back({ fromGeometry(box2), r2 });
			bodies.push_back({ fromGeometry(box3), r3 });
			bodies.push_back({ fromGeometry(box4), r4 });
		}*/

		mthz::Vec3 mech_pos(0, 3, -10);
		double plate_thickness = 0.33;
		double length = 3;
		double width = 1;

		phyz::ConvexUnionGeometry plate = phyz::ConvexUnionGeometry::box(mech_pos, length + width, plate_thickness, width);
		mthz::Vec3 hinge_pos1 = mech_pos + mthz::Vec3(width / 2.0, plate_thickness, width / 2.0);
		mthz::Vec3 hinge_pos2 = mech_pos + mthz::Vec3(length + width / 2.0, plate_thickness, width / 2.0);

		double sweeper_length = length / 2.0;
		double sweeper_thickness = 0.2;
		phyz::ConvexUnionGeometry sweeper1 = phyz::ConvexUnionGeometry::box(hinge_pos1, sweeper_length, sweeper_thickness, sweeper_thickness);
		phyz::ConvexUnionGeometry sweeper2 = phyz::ConvexUnionGeometry::box(hinge_pos2 + mthz::Vec3(-sweeper_length, 0, 0), sweeper_length, sweeper_thickness, sweeper_thickness);

		phyz::RigidBody* plate_r = p.createRigidBody(plate);
		phyz::RigidBody* sweeper1_r = p.createRigidBody(sweeper1);
		phyz::RigidBody* sweeper2_r = p.createRigidBody(sweeper2);

		bodies.push_back({ fromGeometry(plate), plate_r });
		bodies.push_back({ fromGeometry(sweeper1), sweeper1_r});
		bodies.push_back({ fromGeometry(sweeper2), sweeper2_r });

		p.addHingeConstraint(plate_r, sweeper1_r, hinge_pos1, mthz::Vec3(0, 1, 0));
		phyz::ConstraintID motor = p.addHingeConstraint(plate_r, sweeper2_r, hinge_pos2, mthz::Vec3(0, 1, 0));
		p.addHingeConstraint(sweeper1_r, sweeper2_r, hinge_pos1 + mthz::Vec3(sweeper_length, 0, 0), mthz::Vec3(0, 1, 0));

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

		//phyz::RigidBody* gr = p.createRigidBody(grid);
		//bodies.push_back({ fromStaticMeshInput(grid, color{1.0, 0.84, 0.0, 0.25, 0.75, 0.63, 51.2 }), gr });

		//phyz::RigidBody* r = p.createRigidBody(dragon_input, false);
		//bodies.push_back({ fromStaticMeshInput(dragon_input, color{ 1.0, 0.84, 0.0, 0.25, 0.75, 0.63, 51.2 }), r });

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 3, -3);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 60.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, 0.0, 0));

		bool single_step_mode = false;

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

			if (rndr::getKeyDown(GLFW_KEY_P)) {
				p.setMotorConstantTorque(motor, -100);
			}
			else {
				p.setMotorOff(motor);
			}

			if (rndr::getKeyPressed(GLFW_KEY_G)) {
				double block_size = 1.0;
				double block_speed = 15;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::cylinder(pos, 0.3, 3);// .getRotated(mthz::Quaternion(PI / 4, mthz::Vec3(1, 0, 0)), pos);
				//phyz::ConvexUnionGeometry poly = phyz::ConvexUnionGeometry::polyCylinder(pos + mthz::Vec3(0, -0.5, 0), 1, 1);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(camera_dir * block_speed);

				bodies.push_back({ fromGeometry(block), block_r });
			}

			if (rndr::getKeyPressed(GLFW_KEY_H)) {
				double block_size = 1.0;
				double block_speed = 15;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(pos, 1, 1, 1);// .getRotated(mthz::Quaternion(PI / 4, mthz::Vec3(1, 0, 0)), pos);
				//phyz::ConvexUnionGeometry poly = phyz::ConvexUnionGeometry::polyCylinder(pos + mthz::Vec3(0, -0.5, 0), 1, 1);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(camera_dir * block_speed);

				bodies.push_back({ fromGeometry(block), block_r });
			}

			if (rndr::getKeyPressed(GLFW_KEY_R)) {
				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

				if (hit_info.did_hit) {
					for (int i = 0; i < bodies.size(); i++) {
						if (bodies[i].r == hit_info.hit_object) {
							bodies.erase(bodies.begin() + i);
							break;
						}
					}
					p.removeRigidBody(hit_info.hit_object);
				}
			}

			t += fElapsedTime;

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				manager->deselectCurrentScene();
				return;
			}

			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
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
				batch_array.push(transformed_mesh.vertices.data(), transformed_mesh.vertices.size(), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);
		}
	}
};