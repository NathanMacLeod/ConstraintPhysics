#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../ModelReader.h"
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

	enum DrivingInput { FORWARD, BACK, NEUTRAL };
	enum SteeringInput { LEFT, RIGHT, STRAIGHT };

	class RaycastWheel {
	public:
		RaycastWheel(
			phyz::RigidBody* body, mthz::Vec3 local_suspension_position, mthz::Vec3 suspension_down_direction, mthz::Vec3 wheel_forward_direction,
			double max_extension_distance, double suspension_strength, double damping_factor, double friction_constant, double wheel_radius
		)	: body(body), suspension_down_direction(suspension_down_direction), nosteer_wheel_forward_direction(wheel_forward_direction),
			max_suspension_dist(max_extension_distance), suspension_strength(suspension_strength), damping_factor(damping_factor), friction_constant(friction_constant), wheel_radius(wheel_radius),
			wheel_steer_angle(0)
		{
			suspension_location_key = body->trackPoint(local_suspension_position);
		}

		void setDrivingParameters(double drive_forward_strength, double braking_strength, double drive_reverse_strength) {
			this->drive_forward_strength = drive_forward_strength;
			this->braking_strength = braking_strength;
			this->drive_reverse_strength = drive_forward_strength;
		}

		void setSteeringParameters(double steer_rate) {
			this->steer_rate = steer_rate;
		}
		
		bool checkIntersectionAndApplyForces(const phyz::PhysicsEngine& p_engine, DrivingInput drive_input, double elapsed_time) {
			mthz::Vec3 lift_point = body->getTrackedP(suspension_location_key);

			mthz::Vec3 oriented_down_direction = body->getOrientation().applyRotation(suspension_down_direction);

			phyz::RayHitInfo ray_cast = p_engine.raycastFirstIntersection(lift_point, oriented_down_direction, { body });
			if (ray_cast.did_hit && ray_cast.hit_distance < max_suspension_dist) {

				mthz::Vec3 rel_vel = body->getVelOfPoint(ray_cast.hit_position) - ray_cast.hit_object->getVelOfPoint(ray_cast.hit_position);
				double vel = rel_vel.dot(oriented_down_direction);
				//lifting force
				double force = suspension_strength * std::max<double>(0, damping_factor * vel + (max_suspension_dist - ray_cast.hit_distance));
				mthz::Vec3 clipped_lift = oriented_down_direction * ray_cast.surface_normal.dot(oriented_down_direction) * force * elapsed_time;
				body->applyImpulse(clipped_lift, ray_cast.hit_position);

				//all_contact_points.push_back({ ray_cast.hit_position, -lift_dir });

				//friction
				double friction_mag = friction_constant * suspension_strength;
				mthz::Vec3 wheel_forward_dir = body->getOrientation().applyRotation(mthz::Quaternion(wheel_steer_angle, -suspension_down_direction).applyRotation(nosteer_wheel_forward_direction));
				mthz::Vec3 wheel_side_dir = oriented_down_direction.cross(wheel_forward_dir);
				mthz::Vec3 friction_dir = (wheel_side_dir - ray_cast.surface_normal * ray_cast.surface_normal.dot(wheel_side_dir)).normalize();
				mthz::Vec3 friction_force = -friction_dir * friction_dir.dot(rel_vel) * friction_mag * elapsed_time;
				body->applyImpulse(friction_force, ray_cast.hit_position);

				//for animating wheel
				render_wheel_position = lift_point + oriented_down_direction * (ray_cast.hit_distance - 2 * wheel_radius);
				return true;
			}
			else {
				render_wheel_position = lift_point + oriented_down_direction * (max_suspension_dist - 2 * wheel_radius);
				return false;
			}
		}

		void setTargetSteeringAngle(double target_angle) {
			while (target_angle > 2 * PI) target_angle -= 2 * PI;
			while (target_angle < 0) target_angle += 2 * PI;

			target_steer_angle = target_angle;
		}

		void updateWheelSteering(double elapsed_time) {
			while (wheel_steer_angle > 2 * PI) wheel_steer_angle -= 2 * PI;
			while (wheel_steer_angle < 0) wheel_steer_angle += 2 * PI;

			double max_travel_for_tick = steer_rate * elapsed_time;
			if (abs(wheel_steer_angle - target_steer_angle) <= max_travel_for_tick) {
				wheel_steer_angle = target_steer_angle;
			}
			else {

			}
		}

		mthz::Vec3 getWheelRenderLocation() const {
			return render_wheel_position;
		}

		mthz::Quaternion getWheelRenderOrientation() const {
			return body->getOrientation();
		}

	private:
		phyz::RigidBody* body;
		phyz::RigidBody::PKey suspension_location_key;
		mthz::Vec3 suspension_down_direction;
		mthz::Vec3 nosteer_wheel_forward_direction;
		mthz::Vec3 render_wheel_position;
		
		double max_suspension_dist;
		double suspension_strength;
		double damping_factor;
		double friction_constant;
		double wheel_radius;
		double wheel_rotation;
		double drive_forward_strength = 0;
		double drive_reverse_strength = 0;
		double braking_strength = 0;
		double wheel_steer_angle;
		double target_steer_angle;
		double steer_rate = 0;
	};

	class Car {
	public:
		Car(mthz::Vec3 car_position, phyz::PhysicsEngine* p) {
			double car_model_scale = 0.33;
			double lift_max_dist = 2.75 * car_model_scale;
			double lift_equilibrium_dist = 2.65 * car_model_scale;
			double grip = 0.4;
			double dampen = 0.16;

			MeshColliderOutput mc = readMeshAndColliders("resources/mesh/car.obj", car_model_scale);

			phyz::ConvexUnionGeometry car_geom = { mc.colliders["upper"], mc.colliders["lower"] };
			car_mesh = mc.meshes["car"];

			double wheel_radius = 0.6 * car_model_scale;
			double wheel_thickness = 0.45 * car_model_scale;
			wheel_mesh = fromGeometry(phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -wheel_thickness / 2.0, -wheel_radius), wheel_radius, wheel_thickness).getRotated(mthz::Quaternion(-PI / 2, mthz::Vec3(1, 0, 0))));

			//phyz::ConvexUnionGeometry car_chasis = phyz::ConvexUnionGeometry::box(car_position - car_dimensions / 2, car_dimensions.x, car_dimensions.y, car_dimensions.z);
			car_r = p->createRigidBody(car_geom);
			car_r->translate(car_position);
			car_r->setCOMType(phyz::RigidBody::CUSTOM);
			car_r->setCustomCOMLocalPosition(mthz::Vec3(0, -0.2, 0));

			wheels = {
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(2.12, 1.5, 1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p->getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(2.12, 1.5, -1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p->getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(-2.35, 1.5, 1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p->getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(-2.35, 1.5, -1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p->getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
			};

			indices_of_steering_wheels = { 2, 3 };

		}

		phyz::RigidBody* getBody() { return car_r; }

		void update(const phyz::PhysicsEngine& p, DrivingInput forward_input, SteeringInput steer_input, double elapsed_time) {
			for (RaycastWheel& wheel_info : wheels) {
				wheel_info.checkIntersectionAndApplyForces(p, NEUTRAL, elapsed_time);
			}

			double max_turn_angle = PI / 7;

			mthz::Vec3 oriented_forward = car_r->getOrientation().applyRotation(mthz::Vec3(-1, 0, 0));
			double forward_speed = abs(oriented_forward.dot(car_r->getVel()));
			double max_turn_angle_for_speed = asin(sin(max_turn_angle) / (1 + forward_speed / 6));

			double target_steer_angle;
			if (steer_input == LEFT)       target_steer_angle = -max_turn_angle_for_speed;
			else if (steer_input == RIGHT) target_steer_angle = max_turn_angle_for_speed;
			else                           target_steer_angle = 0;

			for (int indx : indices_of_steering_wheels) {
				wheels[indx].setTargetSteeringAngle(target_steer_angle);
				wheels[indx].updateWheelSteering(elapsed_time);
			}
		}

		std::vector<MeshAndPosTransform> getAlMeshesWithPosititons() {
			std::vector<MeshAndPosTransform> mesh_and_transforms;
			mesh_and_transforms.push_back({&car_mesh, car_r->getPos(), car_r->getOrientation()});
			for (const RaycastWheel& r : wheels) {
				mesh_and_transforms.push_back({ &wheel_mesh, r.getWheelRenderLocation(), r.getWheelRenderOrientation() });
			}

			return mesh_and_transforms;
		}

	private:
		phyz::RigidBody* car_r;
		std::vector<RaycastWheel> wheels;
		std::vector<unsigned int> indices_of_steering_wheels;
		Mesh car_mesh;
		Mesh wheel_mesh;
	};

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

		bool lock_cam = true;

		std::vector<PhysBod> bodies;

		mthz::Vec3 center = mthz::Vec3(0, -2, 0);
		int grid_count = 360;
		double grid_size = 0.5;
		//phyz::MeshInput grid = phyz::generateGridMeshInput(grid_count, grid_count, grid_size, center + mthz::Vec3(-grid_count * grid_size / 2.0, 0, -grid_count * grid_size / 2.0));//phyz::generateRadialMeshInput(center, 8, 100, 1);

		//for (mthz::Vec3& v : grid.points) {
		//	//v.y += 0.1 * 2 * (0.5 - frand());
		//	//v.y += 0.0055 * (v - center).magSqrd();
		//	//v.y += 3 - 0.5 * (v - center).mag() + 0.02 * (v - center).magSqrd();
		//	v.y += (10 / (10 + (v - center).mag())) * cos((v - center).mag() / 2.33);
		//}

		//phyz::RigidBody* gr = p.createRigidBody(grid);
		//bodies.push_back({ fromStaticMeshInput(grid, color{1.0, 0.84, 0.0, 0.25, 0.75, 0.63, 51.2 }), gr });

		phyz::Mesh raceway = phyz::readOBJ("resources/mesh/Royal_Raceway.obj", 0.1);
		phyz::MeshInput raceway_input = phyz::generateMeshInputFromMesh(raceway, mthz::Vec3(0, 0, 0));
		phyz::RigidBody* raceway_r = p.createRigidBody(raceway_input, false);
		bodies.push_back({ fromStaticMeshInput(raceway_input, auto_generate), raceway_r });

		//mthz::Vec3 car_dimensions(1, 0.25, 0.5);
		//mthz::Vec3 car_position(0, 5, 0);
		//double car_model_scale = 0.33;
		//double lift_max_dist = 2.75 * car_model_scale;
		//double lift_equilibrium_dist = 2.65 * car_model_scale;
		//double grip = 0.4;
		//double dampen = 0.16;
		//

		//MeshColliderOutput mc = readMeshAndColliders("resources/mesh/car.obj", car_model_scale);

		//phyz::ConvexUnionGeometry car_geom = { mc.colliders["upper"], mc.colliders["lower"] };
		//Mesh car_mesh = mc.meshes["car"];

		//double wheel_radius = 0.6 * car_model_scale;
		//double wheel_thickness = 0.45 * car_model_scale;
		//Mesh wheel = fromGeometry(phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -wheel_thickness / 2.0, -wheel_radius), wheel_radius, wheel_thickness).getRotated(mthz::Quaternion(-PI / 2, mthz::Vec3(1, 0, 0))));

		////phyz::ConvexUnionGeometry car_chasis = phyz::ConvexUnionGeometry::box(car_position - car_dimensions / 2, car_dimensions.x, car_dimensions.y, car_dimensions.z);
		//phyz::RigidBody* car_r = p.createRigidBody(car_geom);
		//car_r->translate(car_position);
		//car_r->setCOMType(phyz::RigidBody::CUSTOM);
		//car_r->setCustomCOMLocalPosition(mthz::Vec3(0, -0.2, 0));
		//double front_wheel_turn_angle = 0;
		//double max_turn_angle = PI / 8;

		//std::vector<RaycastWheel> raycast_wheels = {
		//	RaycastWheel(car_r, car_model_scale * mthz::Vec3(2.12, 1.5, 1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p.getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
		//	RaycastWheel(car_r, car_model_scale * mthz::Vec3(2.12, 1.5, -1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p.getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
		//	RaycastWheel(car_r, car_model_scale * mthz::Vec3(-2.35, 1.5, 1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p.getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
		//	RaycastWheel(car_r, car_model_scale * mthz::Vec3(-2.35, 1.5, -1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, 0.25 * p.getGravity().mag() * car_r->getMass() / 0.2, dampen, grip, wheel_radius),
		//};

		//bodies.push_back({ car_mesh, car_r });

		mthz::Vec3 car_position(0, 5, 0);
		Car car(car_position, &p);

		rndr::lockMouse();
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

		double mouse_sensitivity = 0.0015;
		rndr::MousePos mouse_position = rndr::getMousePosition();
		mthz::Quaternion mouse_aim_target;
		Camera camera;
		camera.setOffsetFromOrbit(mthz::Vec3(0, 2, 10));
		camera.setOrientation(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 1, 0)));

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

			mthz::Vec3 oriented_forward = car.getBody()->getOrientation().applyRotation(mthz::Vec3(-1, 0, 0));
			mthz::Vec3 oriented_up = car.getBody()->getOrientation().applyRotation(mthz::Vec3(0, 1, 0));

			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				car.getBody()->applyTorque(0.100 * car.getBody()->getMass() * oriented_forward);
			}

			if (rndr::getKeyDown(GLFW_KEY_I)) {
				bool breaking = oriented_forward.dot(car.getBody()->getVel()) < 0;

				car.getBody()->applyImpulse(oriented_forward * 2 * car.getBody()->getMass() * fElapsedTime, car.getBody()->getCOM() - 0.2 * oriented_up);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				bool breaking = oriented_forward.dot(car.getBody()->getVel()) > 0;

				car.getBody()->applyImpulse(-oriented_forward * 2 * car.getBody()->getMass() * fElapsedTime, car.getBody()->getCOM() - 0.2 * oriented_up);
			}

			double drag_coeff = 0.0025;
			car.getBody()->applyForce(-drag_coeff * car.getBody()->getVel() * car.getBody()->getVel().mag() * fElapsedTime);

			/*double forward_speed = abs(oriented_forward.dot(car.getBody()->getVel()));
			double max_turn_angle_for_speed = asin(sin(max_turn_angle) / (1 + forward_speed / 6));
			if (rndr::getKeyDown(GLFW_KEY_L)) {
				front_wheel_turn_angle = -max_turn_angle_for_speed;
			}
			else if (rndr::getKeyDown(GLFW_KEY_J)) {
				front_wheel_turn_angle = max_turn_angle_for_speed;
			}
			else {
				front_wheel_turn_angle = 0;
			}*/

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				manager->deselectCurrentScene();
				return;
			}

			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				lock_cam = !lock_cam;
				pos = camera.getPos();
				orient = camera.getOrientation();
			}

			//mouse camera input
			camera.setOrbitPosition(car.getBody()->getPos());
			camera.update(fElapsedTime);

			rndr::MousePos new_mouse = rndr::getMousePosition();
			double mouse_delta_x = new_mouse.x - mouse_position.x;
			double mouse_delta_y = new_mouse.y - mouse_position.y;
			mouse_position = new_mouse;

			bool ignore_mouse_aim_input = false;

			//read mouse input
			if (rndr::getKeyDown(GLFW_MOUSE_BUTTON_1)) {
				ignore_mouse_aim_input = true;
				camera.setDampenEnabled(false);
				camera.setRotateSpeed(9);

				//use mouse for free look
				camera.setTargetOrientation(camera.getTargetOrientation() * mthz::Quaternion(mouse_delta_y * mouse_sensitivity, mthz::Vec3(1, 0, 0)));
				camera.setTargetOrientation(mthz::Quaternion(-mouse_delta_x * mouse_sensitivity, mthz::Vec3(0, 1, 0)) * camera.getTargetOrientation());

			}
			else {
				camera.setDampenEnabled(true);
				camera.setRotateSpeed(4);
				//direct 
				
				mthz::Vec3 car_forward_dir = car.getBody()->getOrientation().applyRotation(mthz::Vec3(-1, 0, 0));
				mthz::Vec3 car_cam_dir = car_forward_dir;
				car_cam_dir.y = 0;
				car_cam_dir = car_cam_dir.normalize();
				double phi = 0;
				if (car_cam_dir.z != 0) {
					phi = atan(car_cam_dir.x / car_cam_dir.z);
					if (car_cam_dir.z > 0) phi += PI;
				}

				mouse_aim_target = mthz::Quaternion(phi, mthz::Vec3(0, 1, 0));
				camera.setTargetOrientation(mouse_aim_target);
			}

			double slow_factor = 1;

			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > slow_factor * timestep) {
				mthz::Vec3 lift_dir = car.getBody()->getOrientation().applyRotation(mthz::Vec3(0, -1, 0));
				mthz::Vec3 forward_dir = car.getBody()->getOrientation().applyRotation(mthz::Vec3(-1, 0, 0));
				mthz::Vec3 side_dir = car.getBody()->getOrientation().applyRotation(mthz::Vec3(0, 0, 1));
				//mthz::Vec3 turned_wheel_side_dir = car.getBody()->getOrientation().applyRotation(mthz::Quaternion(front_wheel_turn_angle, mthz::Vec3(0, 1, 0)).applyRotation(mthz::Vec3(0, 0, 1)));

				//all_contact_points.clear();
				//for (WheelInfo& wheel_info : lift_positions) {
				//	mthz::Vec3 lift_point = car_r->getTrackedP(wheel_info.location_key);

				//	phyz::RayHitInfo ray_cast = p.raycastFirstIntersection(lift_point, lift_dir, { car_r });
				//	if (ray_cast.did_hit && ray_cast.hit_distance < lift_max_dist) {

				//		double k = 0.25 * car_r->getMass() * p.getGravity().mag();
				//		mthz::Vec3 rel_vel = car_r->getVelOfPoint(ray_cast.hit_position) - ray_cast.hit_object->getVelOfPoint(ray_cast.hit_position);
				//		double vel = rel_vel.dot(lift_dir);
				//		//lifting force
				//		double force = std::max<double>(0, dampen * k * vel + (lift_max_dist - ray_cast.hit_distance) * k / (lift_max_dist - lift_equilibrium_dist));
				//		mthz::Vec3 clipped_lift = lift_dir * ray_cast.surface_normal.dot(lift_dir) * force * timestep;
				//		car_r->applyImpulse(clipped_lift, ray_cast.hit_position);

				//		//all_contact_points.push_back({ ray_cast.hit_position, -lift_dir });

				//		//friction
				//		double friction_constant = grip * k;
				//		mthz::Vec3 wheel_side_dir = wheel_info.is_turning_wheel ? turned_wheel_side_dir : side_dir;
				//		mthz::Vec3 friction_dir = (wheel_side_dir - ray_cast.surface_normal * ray_cast.surface_normal.dot(wheel_side_dir)).normalize();
				//		mthz::Vec3 friction_force = -friction_dir * friction_dir.dot(rel_vel) * friction_constant * timestep;
				//		car_r->applyImpulse(friction_force, ray_cast.hit_position);
				//		all_contact_points.push_back({ ray_cast.hit_position, friction_dir });

				//		//for animating wheel
				//		wheel_info.wheel_render_location = lift_point + lift_dir * (ray_cast.hit_distance - 2 * wheel_radius);
				//	}
				//	else {
				//		wheel_info.wheel_render_location = lift_point + lift_dir * (lift_max_dist - 2 * wheel_radius);
				//	}
				//	all_contact_points.push_back({ car_r->getCOM(), mthz::Vec3(0, -1, 0)});
				//}

				car.update(p, NEUTRAL, STRAIGHT, timestep);

				phyz_time -= slow_factor * timestep;
				p.timeStep();
			}

			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = lock_cam? camera.getPos() : pos;
			mthz::Quaternion cam_orient = lock_cam? camera.getOrientation() : orient;

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

			for (const Contact& c : all_contact_points) {
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

			for (MeshAndPosTransform m : car.getAlMeshesWithPosititons()) {

				Mesh transformed_mesh = getTransformed(*m.m, m.position, m.orientation, cam_pos, cam_orient, false, color{ 1.0f, 0.0f, 0.0f });

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() /*|| batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()*/) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), transformed_mesh.vertices.size(), transformed_mesh.indices);

			}

			//for (const RaycastWheel& w : raycast_wheels) {
			//	
			//	//mthz::Quaternion rotate_from_steering = w.is_turning_wheel ? mthz::Quaternion(front_wheel_turn_angle, mthz::Vec3(0, 1, 0)) : mthz::Quaternion();
			//	mthz::Quaternion wheel_orientation = w.getWheelRenderOrientation();


			//	Mesh transformed_mesh = getTransformed(wheel, w.getWheelRenderLocation(), wheel_orientation, cam_pos, cam_orient, false, color{1.0f, 0.0f, 0.0f});

			//	if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
			//		rndr::draw(batch_array, shader);
			//		batch_array.flush();
			//	}
			//	batch_array.push(transformed_mesh.vertices.data(), transformed_mesh.vertices.size(), transformed_mesh.indices);
			//}

			rndr::draw(batch_array, shader);
		}
	}
};