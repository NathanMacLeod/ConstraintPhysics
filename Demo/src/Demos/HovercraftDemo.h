#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class HovercraftDemo : public DemoScene {
public:
	HovercraftDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~HovercraftDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"I. K", "Move forklift forward, reverse"},
			ControlDescription{"J, L", "Turn forklift left, right"},
			ControlDescription{"T, G", "Raise, lower lift"},
			ControlDescription{"F, H", "Tilt lift forward, back"},
			ControlDescription{"Y", "Toggle first-person perspective"},
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
		p.setPGSIterations(45, 65);
		double timestep = 1 / 90.0;
		p.setStep_time(timestep);

		bool lock_cam = false;

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

		mthz::Vec3 car_dimensions(1, 0.25, 0.5);
		mthz::Vec3 car_position(0, 5, 0);
		double lift_max_dist = 0.5;
		double lift_equilibrium_dist = 0.35;

		phyz::ConvexUnionGeometry car_chasis = phyz::ConvexUnionGeometry::box(car_position - car_dimensions / 2, car_dimensions.x, car_dimensions.y, car_dimensions.z);
		phyz::RigidBody* car_r = p.createRigidBody(car_chasis);

		std::vector<phyz::RigidBody::PKey> lift_positions = {
			car_r->trackPoint(car_position + mthz::Vec3(car_dimensions.x / 2, 0, car_dimensions.z / 2)),
			car_r->trackPoint(car_position + mthz::Vec3(car_dimensions.x / 2, 0, -car_dimensions.z / 2)),
			car_r->trackPoint(car_position + mthz::Vec3(-car_dimensions.x / 2, 0, car_dimensions.z / 2)),
			car_r->trackPoint(car_position + mthz::Vec3(-car_dimensions.x / 2, 0, -car_dimensions.z / 2)),
		};

		bodies.push_back({ fromGeometry(car_chasis), car_r });

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(15, 3, 0);
		mthz::Quaternion orient(PI / 2.0, mthz::Vec3(0, 1, 0));
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		p.setGravity(mthz::Vec3(0, -6.0, 0));

		struct Contact {
			mthz::Vec3 p;
			mthz::Vec3 n;
		};
		std::vector<Contact> all_contact_points;
		Mesh contact_ball_mesh = fromGeometry(phyz::ConvexUnionGeometry::merge(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.03), phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), 0.02, 0.1)), { 1.0, 0, 0 });

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

				if (hit_info.did_hit) hit_info.hit_object->applyImpulse(camera_dir * 0.2, hit_info.hit_position);
			}

			t += fElapsedTime;

			mthz::Vec3 oriented_forward = car_r->getOrientation().applyRotation(mthz::Vec3(1, 0, 0));
			mthz::Vec3 oriented_up = car_r->getOrientation().applyRotation(mthz::Vec3(0, 1, 0));

			if (rndr::getKeyDown(GLFW_KEY_I)) {
				bool breaking = oriented_forward.dot(car_r->getVel()) < 0;

				car_r->applyImpulse(oriented_forward * 0.1 * timestep, car_r->getCOM() - 0.2 * oriented_up);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				bool breaking = oriented_forward.dot(car_r->getVel()) > 0;

				car_r->applyImpulse(-oriented_forward * 0.1 * timestep, car_r->getCOM() - 0.2 * oriented_up);
			}

			/*if (rndr::getKeyDown(GLFW_KEY_J)) {
				steer_angle = -steer_max_angle;
			}
			else if (rndr::getKeyDown(GLFW_KEY_L)) {
				steer_angle = steer_max_angle;
			}*/

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				manager->deselectCurrentScene();
				return;
			}

			double slow_factor = 1;

			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > slow_factor * timestep) {
				mthz::Vec3 lift_dir = car_r->getOrientation().applyRotation(mthz::Vec3(0, -1, 0));
				mthz::Vec3 forward_dir = car_r->getOrientation().applyRotation(mthz::Vec3(1, 0, 0));
				mthz::Vec3 side_dir = car_r->getOrientation().applyRotation(mthz::Vec3(0, 0, 1));
				all_contact_points.clear();
				for (phyz::RigidBody::PKey lift_point_key : lift_positions) {
					mthz::Vec3 lift_point = car_r->getTrackedP(lift_point_key);

					phyz::RayHitInfo ray_cast = p.raycastFirstIntersection(lift_point, lift_dir, { car_r });
					if (ray_cast.did_hit && ray_cast.hit_distance < lift_max_dist) {

						double k = 0.25 * car_r->getMass() * p.getGravity().mag();
						mthz::Vec3 rel_vel = car_r->getVelOfPoint(ray_cast.hit_position) - ray_cast.hit_object->getVelOfPoint(ray_cast.hit_position);
						double vel = rel_vel.dot(lift_dir);
						//lifting force
						double force = 0.3 * k * vel + sqrt(lift_max_dist - ray_cast.hit_distance) * k / sqrt(lift_max_dist - lift_equilibrium_dist);
						mthz::Vec3 clipped_lift = lift_dir * ray_cast.surface_normal.dot(lift_dir) * force * timestep;
						car_r->applyImpulse(clipped_lift, ray_cast.hit_position);
						//all_contact_points.push_back({ ray_cast.hit_position, -lift_dir });

						//friction
						double friction_constant = 0;
						mthz::Vec3 friction_side = rel_vel.dot(side_dir) > 0 ? -side_dir : side_dir;
						mthz::Vec3 fiction_dir = friction_side - ray_cast.surface_normal.dot(friction_side) * ray_cast.surface_normal;
						car_r->applyImpulse(fiction_dir * friction_constant * force * timestep, ray_cast.hit_position);
						all_contact_points.push_back({ ray_cast.hit_position, fiction_dir });
					}
				}

				phyz_time -= slow_factor * timestep;
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
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 550.0, 2.0, 2.0 * aspect_ratio, 60.0));
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

				Mesh transformed_mesh = getTransformed(contact_ball_mesh, c.p, rot, cam_pos, cam_orient, false, color{ 1.0f, 0.0f, 0.0f });

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