#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../ModelReader.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

static bool freeze = false;

class HovercraftDemo : public DemoScene {
public:
	HovercraftDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~HovercraftDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Drive controls"},
			ControlDescription{"Mouse: rotate the camera"},
			ControlDescription{"P", "Toggle free look"},
			ControlDescription{"I, J, K, L", "Move the camera in free look"},
			ControlDescription{"Arrow Keys", "Rotate the camera in free look"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	enum DrivingInput { FORWARD, BACK, NEUTRAL };
	enum SteeringInput { LEFT, RIGHT, STRAIGHT };

	class RaycastWheel {
	public:
		RaycastWheel(
			phyz::RigidBody* body, mthz::Vec3 local_suspension_position, mthz::Vec3 suspension_down_direction, mthz::Vec3 wheel_forward_direction,
			double max_extension_distance, double suspension_strength, double damping_factor, double max_friction_constant, double min_friction_constant, double wheel_radius, double max_extension_velocity
		) : body(body), suspension_down_direction(suspension_down_direction), nosteer_wheel_forward_direction(wheel_forward_direction),
			max_suspension_dist(max_extension_distance), suspension_strength(suspension_strength), damping_factor(damping_factor), max_friction_constant(max_friction_constant), min_friction_constant(min_friction_constant),
			wheel_radius(wheel_radius), wheel_steer_angle(0), wheel_rotation(0), wheel_rotation_speed(0), max_extension_velocity(max_extension_velocity)
		{
			suspension_location_key = body->trackPoint(local_suspension_position);
		}

		void setDriveStrength(double drive_forward_strength, double drive_reverse_strength) {
			this->drive_forward_strength = drive_forward_strength;
			this->drive_reverse_strength = drive_forward_strength;
		}

		void setBrakingStrength(double braking_strength) {
			this->braking_strength = braking_strength;
		}

		void setSteeringParameters(double steer_rate) {
			this->steer_rate = steer_rate;
		}
		
		bool checkIntersectionAndApplyForces(const phyz::PhysicsEngine& p_engine, DrivingInput drive_input, double elapsed_time) {
			mthz::Vec3 lift_point = body->getTrackedP(suspension_location_key);

			mthz::Vec3 oriented_down_direction = body->getOrientation().applyRotation(suspension_down_direction);

			phyz::RayHitInfo ray_cast = p_engine.raycastFirstIntersection(lift_point, oriented_down_direction, { body });
			if (ray_cast.did_hit && ray_cast.hit_distance < max_suspension_dist) {

				double extension_delta = ray_cast.hit_distance - current_extension;
				double max_extension_delta_for_tick = max_extension_velocity * elapsed_time;
				if (abs(extension_delta) > max_extension_delta_for_tick) {
					if (extension_delta > 0) extension_delta = max_extension_delta_for_tick;
					else					 extension_delta = -max_extension_delta_for_tick;
				}
				
				current_extension += extension_delta;

				phyz::RigidBody* touching_body = ray_cast.hit_object;

				mthz::Vec3 rel_vel = body->getVelOfPoint(ray_cast.hit_position) -ray_cast.hit_object->getVelOfPoint(ray_cast.hit_position);
				double vel = rel_vel.dot(oriented_down_direction);
				//lifting force
				double force = suspension_strength * std::max<double>(0, damping_factor * vel + (max_suspension_dist -current_extension));
				mthz::Vec3 clipped_lift = -ray_cast.surface_normal * ray_cast.surface_normal.dot(oriented_down_direction) * force * elapsed_time;
				body->applyImpulse(clipped_lift, ray_cast.hit_position);
				//touching_body->applyImpulse(-clipped_lift, ray_cast.hit_position);

				mthz::Vec3 wheel_forward_dir = body->getOrientation().applyRotation(mthz::Quaternion(wheel_steer_angle, -suspension_down_direction).applyRotation(nosteer_wheel_forward_direction));
				mthz::Vec3 wheel_side_dir = oriented_down_direction.cross(wheel_forward_dir);

				//drive force
				bool is_moving_forward = rel_vel.dot(wheel_forward_dir) > 0;
				bool is_braking = is_moving_forward && drive_input == BACK || !is_moving_forward && drive_input == FORWARD;
				double drive_force_coeff;
				if (drive_input == FORWARD)   drive_force_coeff = is_braking ? braking_strength : drive_forward_strength;
				else if (drive_input == BACK) drive_force_coeff = is_braking ? -braking_strength : -drive_reverse_strength;
				else                          drive_force_coeff = 0;

				mthz::Vec3 drive_force = wheel_forward_dir * drive_force_coeff * elapsed_time;
				body->applyImpulse(drive_force, ray_cast.hit_position);
				//touching_body->applyImpulse(-drive_force, ray_cast.hit_position);

				//friction
				double forward_vel = rel_vel.dot(wheel_forward_dir);
				double slide_angle = 0;
				if (abs(forward_vel) > 0.000001) {
					slide_angle = atan(abs(forward_vel / rel_vel.dot(wheel_side_dir)));
				}
				//printf("%f\n", slide_angle);
				double friction_constant = max_friction_constant -(max_friction_constant -min_friction_constant) * slide_angle / (PI / 2);
				double friction_mag = friction_constant * suspension_strength;
				
				mthz::Vec3 friction_dir = (wheel_side_dir -ray_cast.surface_normal * ray_cast.surface_normal.dot(wheel_side_dir)).normalize();
				mthz::Vec3 friction_force = -friction_dir * friction_dir.dot(rel_vel) * friction_mag * elapsed_time;
				body->applyImpulse(friction_force, ray_cast.hit_position);
				//touching_body->applyImpulse(-friction_force, ray_cast.hit_position);

				//for animating wheel
				wheel_rotation_speed = forward_vel / (2 * PI * wheel_radius);
				wheel_rotation += wheel_rotation_speed * elapsed_time;

				while (wheel_rotation > 2 * PI) wheel_rotation -= 2 * PI;
				while (wheel_rotation < 0)      wheel_rotation += 2 * PI;
				return true;
			}
			else {
				wheel_rotation_speed /= (1 + 0.2 * elapsed_time);
				wheel_rotation += wheel_rotation_speed * elapsed_time;
				while (wheel_rotation > 2 * PI) wheel_rotation -= 2 * PI;
				while (wheel_rotation < 0)      wheel_rotation += 2 * PI;

				current_extension = max_suspension_dist;
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
			if (abs(wheel_steer_angle -target_steer_angle) <= max_travel_for_tick) {
				wheel_steer_angle = target_steer_angle;
				return;
			}

			if (sin(target_steer_angle -wheel_steer_angle) > 0) {
				wheel_steer_angle += max_travel_for_tick;
			}
			else {
				wheel_steer_angle -= max_travel_for_tick;
			}
		}

		mthz::Vec3 getWheelRenderLocation() const {
			mthz::Vec3 lift_point = body->getExtrapolatedTrackedP(suspension_location_key);
			mthz::Vec3 oriented_down_direction = body->getExtrapolatedOrientation().applyRotation(suspension_down_direction);

			return lift_point + oriented_down_direction * (current_extension -wheel_radius);
		}

		mthz::Quaternion getWheelRenderOrientation() const {
			return body->getOrientation() * mthz::Quaternion(wheel_steer_angle, mthz::Vec3(0, 1, 0)) * mthz::Quaternion(wheel_rotation, mthz::Vec3(0, 0, 1));
		}

	private:
		phyz::RigidBody* body;
		phyz::RigidBody::PKey suspension_location_key;
		mthz::Vec3 suspension_down_direction;
		mthz::Vec3 nosteer_wheel_forward_direction;
		
		double max_suspension_dist;
		double suspension_strength;
		double damping_factor;
		double max_friction_constant;
		double min_friction_constant;
		double wheel_radius;
		double wheel_rotation;
		double wheel_rotation_speed;
		double drive_forward_strength = 0;
		double drive_reverse_strength = 0;
		double braking_strength = 0;
		double wheel_steer_angle;
		double target_steer_angle;
		double steer_rate = 0;
		double current_extension;
		double max_extension_velocity;
	};

	class Car {
	public:
		Car(mthz::Vec3 car_position, phyz::PhysicsEngine* p) {
			double car_model_scale = 0.33;
			double lift_max_dist = 1.5 * car_model_scale;
			double lift_equilibrium_dist = 2.65 * car_model_scale;
			double max_grip = 0.3;
			double min_grip = 0.05;
			double dampen = 0.16;
			double stiffness_modifier = 30;
			double wheel_height = 0.35;

			MeshColliderOutput mc = readMeshAndColliders("resources/mesh/car.obj", car_model_scale);

			phyz::ConvexUnionGeometry car_geom = { mc.colliders["upper"], mc.colliders["lower"] };
			car_mesh = mc.meshes["car"];

			double wheel_radius = 0.6 * car_model_scale;
			double wheel_thickness = 0.45 * car_model_scale;
			wheel_mesh = fromGeometry(phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(0, -wheel_thickness / 2.0, 0), wheel_radius, wheel_thickness).getRotated(mthz::Quaternion(-PI / 2, mthz::Vec3(1, 0, 0))));

			//phyz::ConvexUnionGeometry car_chasis = phyz::ConvexUnionGeometry::box(car_position -car_dimensions / 2, car_dimensions.x, car_dimensions.y, car_dimensions.z);
			car_r = p->createRigidBody(car_geom);
			car_r->translate(car_position);
			car_r->setCOMType(phyz::RigidBody::CUSTOM);
			car_r->setCustomCOMLocalPosition(mthz::Vec3(0, -0.25, 0));

			wheels = {
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(2.12, wheel_height, 1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, stiffness_modifier * 0.25 * p->getGravity().mag() * car_r->getMass(), dampen, max_grip, min_grip, wheel_radius, 1.0),
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(2.12, wheel_height, -1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, stiffness_modifier * 0.25 * p->getGravity().mag() * car_r->getMass(), dampen, max_grip, min_grip, wheel_radius, 1.0),
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(-2.35, wheel_height, 1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, stiffness_modifier * 0.25 * p->getGravity().mag() * car_r->getMass(), dampen, max_grip, min_grip, wheel_radius, 1.0),
				RaycastWheel(car_r, car_model_scale * mthz::Vec3(-2.35, wheel_height, -1.25), mthz::Vec3(0, -1, 0), mthz::Vec3(-1, 0, 0), lift_max_dist, stiffness_modifier * 0.25 * p->getGravity().mag() * car_r->getMass(), dampen, max_grip, min_grip, wheel_radius, 1.0),
			};

			indices_of_steering_wheels = { 2, 3 };

			for (RaycastWheel& r : wheels) {
				r.setBrakingStrength(3 * car_r->getMass());
			}

			for (int indx : indices_of_steering_wheels) {
				wheels[indx].setSteeringParameters(1.0);
				wheels[indx].setDriveStrength(1.2 * car_r->getMass(), 0.35 * car_r->getMass());
			}
		}

		phyz::RigidBody* getBody() { return car_r; }

		void update(const phyz::PhysicsEngine& p, DrivingInput forward_input, SteeringInput steer_input, double elapsed_time) {
			bool no_wheels_on_ground = true;
			for (RaycastWheel& wheel_info : wheels) {
				bool wheel_on_ground = wheel_info.checkIntersectionAndApplyForces(p, forward_input, elapsed_time);
				no_wheels_on_ground &= !wheel_on_ground;
			}

			double max_turn_angle = PI / 7;

			mthz::Vec3 oriented_forward = car_r->getOrientation().applyRotation(mthz::Vec3(-1, 0, 0));
			double forward_speed = abs(oriented_forward.dot(car_r->getVel()));
			double max_turn_angle_for_speed = asin(sin(max_turn_angle) / (1 + forward_speed / 5));

			double target_steer_angle;
			if (steer_input == LEFT)       target_steer_angle = max_turn_angle_for_speed;
			else if (steer_input == RIGHT) target_steer_angle = -max_turn_angle_for_speed;
			else                           target_steer_angle = 0;

			for (int indx : indices_of_steering_wheels) {
				wheels[indx].setTargetSteeringAngle(target_steer_angle);
				wheels[indx].updateWheelSteering(elapsed_time);
			}

			mthz::Vec3 pitch_axis = car_r->getOrientation().applyRotation(mthz::Vec3(0, 0, 1));
			mthz::Vec3 yaw_axis = car_r->getOrientation().applyRotation(mthz::Vec3(0, 1, 0));
			double pitch_torque = 0.5 * car_r->getMass();
			double yaw_torque = 0.5 * car_r->getMass();
			if (no_wheels_on_ground) {
				if (forward_input == FORWARD)    car_r->applyTorque(pitch_axis * pitch_torque * elapsed_time);
				else if (forward_input == BACK)  car_r->applyTorque(-pitch_axis * pitch_torque * elapsed_time);
				if (steer_input == LEFT)       car_r->applyTorque(yaw_axis * pitch_torque * elapsed_time);
				else if (steer_input == RIGHT) car_r->applyTorque(-yaw_axis * pitch_torque * elapsed_time);
			}

			double drag_coeff = 0.0025;
			car_r->applyForce(-drag_coeff * car_r->getVel() * car_r->getVel().mag() * elapsed_time);
		}

		std::vector<MeshAndPosTransform> getAlMeshesWithPosititons() {
			std::vector<MeshAndPosTransform> mesh_and_transforms;
			mesh_and_transforms.push_back({&car_mesh, car_r->getExtrapolatedPos(), car_r->getExtrapolatedOrientation()});
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

	void createCow(const phyz::ConvexUnionGeometry& cow_geom, const Mesh& cow_mesh, phyz::PhysicsEngine* p, std::vector<PhysBod>* bodies, mthz::Vec3 pos) {
		phyz::RigidBody* cow_r = p->createRigidBody(cow_geom);
		cow_r->rotate(mthz::Quaternion(frand() * 2 * PI, mthz::Vec3(0, 1, 0)));
		cow_r->setToPosition(pos + mthz::Vec3(0, 1, 0));
		bodies->push_back({ cow_mesh, cow_r});
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

		bool lock_cam = true;

		std::vector<PhysBod> bodies;

		mthz::Vec3 center = mthz::Vec3(0, -2, 0);
		int grid_count = 360;
		double grid_size = 0.5;

		phyz::Mesh raceway = phyz::readOBJ("resources/mesh/Royal_Raceway.obj", 0.1);
		phyz::MeshInput raceway_input = phyz::generateMeshInputFromMesh(raceway, mthz::Vec3(0, 0, 0));
		phyz::RigidBody* raceway_r = p.createRigidBody(raceway_input, false);
		bodies.push_back({ fromStaticMeshInput(raceway_input, auto_generate), raceway_r });

		MeshColliderOutput cow_mc = readMeshAndColliders("resources/mesh/cow_col.obj", 0.15);
		phyz::ConvexUnionGeometry cow_geom = {
			cow_mc.colliders["back_left"], cow_mc.colliders["back_right"], cow_mc.colliders["body"], cow_mc.colliders["ear_left"], cow_mc.colliders["ear_right"],
			cow_mc.colliders["front_left"], cow_mc.colliders["front_right"], cow_mc.colliders["head"], cow_mc.colliders["horn_base"], cow_mc.colliders["horn_tip_left"],
			cow_mc.colliders["horn_tip_right"], cow_mc.colliders["neck"], cow_mc.colliders["tail"]
		};
		Mesh cow_mesh = cow_mc.meshes["cow"];
		std::vector<mthz::Vec3> cow_positions = {
			mthz::Vec3(-13.693719, 0.336069, -166.137502),
			mthz::Vec3(-21.444286, 0.349473, -172.047570),
			mthz::Vec3(-23.731571, 0.350590, -186.608710),
			mthz::Vec3(-157.865747, 0.370183, -164.332497),
			mthz::Vec3(-167.120554, 0.407008, -170.725379),
			mthz::Vec3(-179.745823, 0.331051, -187.114547),
			mthz::Vec3(-186.666525, 0.350841, -190.334247),
			mthz::Vec3(-190.415045, 0.322733, -195.677583),
			mthz::Vec3(-190.015749, 0.318974, -200.923400),
			mthz::Vec3(-154.233430, 7.671412, -221.625220),
			mthz::Vec3(-148.018882, 8.194750, -227.043780),
			mthz::Vec3(-129.504210, 1.793921, -275.194313),
			mthz::Vec3(-123.790877, 1.895054, -275.739482),
			mthz::Vec3(-28.500858, 4.965436, -241.041626),
			mthz::Vec3(-26.158206, 4.777654, -237.350813),
			mthz::Vec3(-22.103583, 4.235274, -238.204941),
			mthz::Vec3(35.175545, 1.572493, -247.131924),
			mthz::Vec3(41.178965, 1.255116, -247.255206),
			mthz::Vec3(43.921424, 1.187011, -248.712597),
			mthz::Vec3(44.537066, 1.322759, -252.487821),
			mthz::Vec3(47.866447, 1.105480, -274.798988),
			mthz::Vec3(51.026149, 0.855093, -282.531642),
			mthz::Vec3(56.557012, 0.653733, -288.878993),
			mthz::Vec3(61.489889, 0.684995, -284.380334),
			mthz::Vec3(125.637475, 0.336231, -281.817086),
			mthz::Vec3(130.596532, 0.336566, -283.159782),
			mthz::Vec3(133.223904, 0.350697, -252.514680),
			mthz::Vec3(130.091752, 0.349289, -247.432180),
			mthz::Vec3(124.643171, 0.349019, -247.529755),
			mthz::Vec3(118.550103, 0.351011, -250.243275),
			mthz::Vec3(114.202593, 0.350958, -246.916200),
			mthz::Vec3(-163.178636, 0.345247, -80.432354),
			mthz::Vec3(-163.990327, 0.358650, -80.880536),
			mthz::Vec3(-165.247348, 0.343767, -80.982813),
			mthz::Vec3(-166.696686, 0.331126, -80.661729),
			mthz::Vec3(-167.646695, 0.399605, -79.871255),
			mthz::Vec3(-167.934599, 0.346722, -78.656303),
			mthz::Vec3(-167.681456, 0.397969, -77.215300),
			mthz::Vec3(-167.668500, 0.345016, -75.719205),
			mthz::Vec3(-167.981791, 0.430717, -74.106130),
			mthz::Vec3(-168.802161, 0.337444, -73.115156),
			mthz::Vec3(-169.970176, 0.499741, -73.080229),
			mthz::Vec3(-171.186096, 0.323435, -73.419088),
			mthz::Vec3(-171.637945, 0.364699, -74.573522),
			mthz::Vec3(-172.329467, 0.346142, -75.785403),
			mthz::Vec3(-173.121460, 0.443006, -76.765322),
			mthz::Vec3(-174.252151, 0.339674, -77.302021),
			mthz::Vec3(-175.452851, 0.453901, -76.947569),
			mthz::Vec3(-176.334742, 0.340181, -76.073848),
			mthz::Vec3(-176.421914, 0.492926, -74.923350),
			mthz::Vec3(-176.295508, 0.318068, -73.707619),
			mthz::Vec3(-175.523774, 0.411020, -72.805413),
			mthz::Vec3(-174.785244, 0.334042, -71.690861),
			mthz::Vec3(-174.272839, 0.427367, -70.215348),
			mthz::Vec3(-248.832413, 0.463198, -38.276914),
			mthz::Vec3(-250.104193, 0.691974, -36.335698),
			mthz::Vec3(-250.640832, 0.751088, -33.569884),
			mthz::Vec3(-310.119489, 2.163270, -45.036074),
			mthz::Vec3(-314.738313, 2.189016, -42.538793),
			mthz::Vec3(-215.911921, 0.856620, 25.298080),
			mthz::Vec3(-211.301625, 1.134776, 24.806675),
		};
		for (mthz::Vec3 cow_pos : cow_positions) createCow(cow_geom, cow_mesh, &p, &bodies, cow_pos);

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
		double mv_speed = 20;
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

		SteeringInput steer_input;
		DrivingInput drive_input;

		while (rndr::render_loop(&fElapsedTime)) {

			if (rndr::getKeyDown(GLFW_KEY_I)) {
				pos += orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				pos += orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
			}
			if (rndr::getKeyDown(GLFW_KEY_J)) {
				pos += orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_L)) {
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

			if (rndr::getKeyDown(GLFW_KEY_W)) {
				drive_input = FORWARD;
			}
			else if (rndr::getKeyDown(GLFW_KEY_S)) {
				drive_input = BACK;
			}
			else {
				drive_input = NEUTRAL;
			}

			if (rndr::getKeyDown(GLFW_KEY_A)) {
				steer_input = LEFT;
			}
			else if (rndr::getKeyDown(GLFW_KEY_D)) {
				steer_input = RIGHT;
			}
			else {
				steer_input = STRAIGHT;
			}

			if (rndr::getKeyPressed(GLFW_KEY_SPACE)) {
				//printf("%f, %f, %f\n", car.getBody()->getPos().x, car.getBody()->getPos().y, car.getBody()->getPos().z);
				//car.getBody()->applyForce(mthz::Vec3(0, 2, 0));
				freeze = false;
			}

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
			camera.setOrbitPosition(car.getBody()->getExtrapolatedPos());
			camera.update(fElapsedTime);

			rndr::MousePos new_mouse = rndr::getMousePosition();
			double mouse_delta_x = new_mouse.x -mouse_position.x;
			double mouse_delta_y = new_mouse.y -mouse_position.y;
			mouse_position = new_mouse;

			bool ignore_mouse_aim_input = false;

			//read mouse input
			if (true) {
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

			phyz_time += fElapsedTime / slow_factor;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);

			if (!freeze) {
				if (phyz_time < timestep) {
					p.extrapolateObjectPositions(fElapsedTime / slow_factor);
				}
				bool ticked = false;
				while (phyz_time > timestep && !freeze) {
					mthz::Vec3 lift_dir = car.getBody()->getOrientation().applyRotation(mthz::Vec3(0, -1, 0));
					mthz::Vec3 forward_dir = car.getBody()->getOrientation().applyRotation(mthz::Vec3(-1, 0, 0));
					mthz::Vec3 side_dir = car.getBody()->getOrientation().applyRotation(mthz::Vec3(0, 0, 1));

					car.update(p, drive_input, steer_input, timestep);

					phyz_time -= timestep;
					p.timeStep();
					ticked = true;
				}
				if (ticked && !freeze) {
					p.extrapolateObjectPositions(phyz_time);
				}
			}

			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = lock_cam? camera.getPos() : pos;
			mthz::Quaternion cam_orient = lock_cam? camera.getOrientation() : orient;

			mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos -cam_pos);

			float aspect_ratio = (float)properties.window_height / properties.window_width;
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 550.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
			shader.setUniform3f("u_ambient_light", 0.8f, 0.8f, 0.8f);
			shader.setUniform3f("u_pointlight_pos", static_cast<float>(trnsfm_light_pos.x), static_cast<float>(trnsfm_light_pos.y), static_cast<float>(trnsfm_light_pos.z));
			shader.setUniform3f("u_pointlight_col", 0.2f, 0.2f, 0.2f);
			shader.setUniform1i("u_Asleep", false);

			for (const PhysBod& b : bodies) {

				Mesh transformed_mesh = getTransformed(b.mesh, b.r->getPos(), b.r->getOrientation(), cam_pos, cam_orient, b.r->getAsleep(), color{ 1.0f, 0.0f, 0.0f });

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<int>(transformed_mesh.vertices.size()), transformed_mesh.indices);
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
				batch_array.push(transformed_mesh.vertices.data(), static_cast<int>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			for (MeshAndPosTransform m : car.getAlMeshesWithPosititons()) {

				Mesh transformed_mesh = getTransformed(*m.m, m.position, m.orientation, cam_pos, cam_orient, false, color{ 1.0f, 0.0f, 0.0f });

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() /*|| batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()*/) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<int>(transformed_mesh.vertices.size()), transformed_mesh.indices);

			}

			rndr::draw(batch_array, shader);
		}
	}
};