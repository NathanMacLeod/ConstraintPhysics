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
		p.setSleepingEnabled(false);
		p.setPGSIterations(45, 35);

		bool lock_cam = true;

		std::vector<PhysBod> bodies;
		std::vector<phyz::ConstraintID> constraints;
		
		
			mthz::Vec3 base_dim(8.5, 0.25, 0.75);
			phyz::Geometry base = phyz::Geometry::box(mthz::Vec3(), base_dim.x, base_dim.y, base_dim.z);
			double box_height = 48;
			phyz::Geometry negx_wall = phyz::Geometry::box(mthz::Vec3(0, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
			phyz::Geometry posx_wall = phyz::Geometry::box(mthz::Vec3(base_dim.x - base_dim.y, base_dim.y, 0), base_dim.y, box_height, base_dim.z);
			phyz::Geometry back_wall = phyz::Geometry::box(mthz::Vec3(0, 0, 0), base_dim.x, box_height + base_dim.y, -base_dim.y);
			phyz::Geometry front_wall = phyz::Geometry::box(mthz::Vec3(0, 0, base_dim.z), base_dim.x, base_dim.y + box_height, base_dim.y);


			double effective_width = base_dim.x - 2 * base_dim.y;
			int n_rows = 4;
			int n_pins = 3;
			double pin_gap = effective_width / (n_pins + 1);
			double row_verticle_spacing = 2;
			double cylinder_radius = 0.15;
			double pin_start_y = 10;
			for (int i = 0; i < n_rows; i++) {
				double start_x = base_dim.y + pin_gap * ((i % 2 == 0) ? 1 : 1.5);
				for (int j = 0; j < ((i % 2 == 0) ? n_pins : n_pins - 1); j++) {
					mthz::Vec3 pin_pos = mthz::Vec3(start_x + pin_gap * j, pin_start_y + row_verticle_spacing * i, 0);
					phyz::Geometry pin = phyz::Geometry::cylinder(pin_pos, cylinder_radius, base_dim.z)
						.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), pin_pos);

					bodies.push_back({ fromGeometry(pin), p.createRigidBody(pin, true) });
				}
			}

			double cube_size = effective_width / (16 * 2);
			phyz::Geometry g = /*phyz::Geometry::octahedron(mthz::Vec3(), cube_size / 2.0);*/phyz::Geometry::sphere(mthz::Vec3(), cube_size / 2.0);

			phyz::RigidBody* ball_r = p.createRigidBody(g);
			ball_r->setCOMtoPosition(mthz::Vec3(5.8906498990466201, 22.836214149571784, 0.62500526804359435));
			//ball_r->setOrientation(mthz::Quaternion(-0.82399634441035130, -0.16414793437481767, 0.51331333588541173, -0.17491397668868225));
			ball_r->setVel(mthz::Vec3(0.087282235307977865, -0.16404399264491565, -8.5426457574477865e-15));
			ball_r->setAngVel(mthz::Vec3(-1.1490186078256817, -0.69825788246386356, 0.49999995774193268));
			bodies.push_back({ fromGeometry(g), ball_r });

			double spinner_y = 23;
			double spinner_radius = effective_width / 4.75;

			mthz::Vec3 spinner1_pos = mthz::Vec3(base_dim.y + effective_width / 4.0, spinner_y, 0);
			mthz::Vec3 spinner2_pos = mthz::Vec3(base_dim.y + effective_width * 3 / 4.0, spinner_y, 0);
			double spinner_density = 10000;
			phyz::Geometry spinner1 = phyz::Geometry::gear(spinner1_pos, cylinder_radius * 2, spinner_radius, base_dim.z, 4, false, phyz::Material::modified_density(spinner_density))
				.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner1_pos);
			phyz::Geometry spinner2 = phyz::Geometry::gear(spinner2_pos, cylinder_radius * 2, spinner_radius, base_dim.z, 4, false, phyz::Material::modified_density(spinner_density))
				.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), spinner2_pos);

			phyz::RigidBody* base_r = p.createRigidBody(base, true);
			phyz::RigidBody* negx_wall_r = p.createRigidBody(negx_wall, true);
			phyz::RigidBody* posx_wall_r = p.createRigidBody(posx_wall, true);
			phyz::RigidBody* back_wall_r = p.createRigidBody(back_wall, true);
			phyz::RigidBody* front_wall_r = p.createRigidBody(front_wall, true);

			phyz::RigidBody* spinner1_r = p.createRigidBody(spinner1);
			phyz::RigidBody* spinner2_r = p.createRigidBody(spinner2);

			phyz::ConstraintID spinner1_motor = p.addHingeConstraint(front_wall_r, spinner1_r, spinner1_pos, mthz::Vec3(0, 0, 1));
			p.setMotorTargetVelocity(spinner1_motor, 10000, 0.5);
			phyz::ConstraintID spinner2_motor = p.addHingeConstraint(front_wall_r, spinner2_r, spinner2_pos, mthz::Vec3(0, 0, 1));
			p.setMotorTargetVelocity(spinner2_motor, 10000, -0.5);

			bodies.push_back({ fromGeometry(base), base_r });
			bodies.push_back({ fromGeometry(negx_wall), negx_wall_r });
			bodies.push_back({ fromGeometry(posx_wall), posx_wall_r });
			bodies.push_back({ fromGeometry(back_wall), back_wall_r });
			//bodies.push_back({ fromGeometry(front_wall), front_wall_r });
			bodies.push_back({ fromGeometry(spinner1), spinner1_r });
			bodies.push_back({ fromGeometry(spinner2), spinner2_r });

			spinner2_r->setCOMtoPosition(mthz::Vec3(6.2500000000266827, 22.999999999995897, 0.37500000000001638));
			spinner2_r->setOrientation(mthz::Quaternion(-0.11870263824785451, 2.0982500000746412e-12, 1.1376674435583084e-15, 0.99292984831406850));
			spinner2_r->setVel(mthz::Vec3(1.2756948660794449e-12, -0.020416666659167953, -4.6651984362506619e-14));
			spinner2_r->setAngVel(mthz::Vec3(-1.4314074496400589e-12, 7.6703616913054963e-12, 0.49999999999998201));
		

		/*double r = 1;
		double eps = 0.001;
		double wall_width = 10;
		double wall_thick = 1.0;

		mthz::Vec3 wall1_pos(-wall_thick - r + eps, r + wall_thick - eps - wall_width, -wall_width/2.0);
		phyz::Geometry wall1 = phyz::Geometry::box(wall1_pos, wall_thick, wall_width, wall_width);
		
		mthz::Vec3 wall2_pos = wall1_pos + mthz::Vec3(0, wall_width - wall_thick, 0);
		phyz::Geometry wall2 = phyz::Geometry::box(wall2_pos, wall_width, wall_thick, wall_width);

		phyz::Geometry ball = phyz::Geometry::sphere(mthz::Vec3(), 1);

		phyz::RigidBody* wall1_r = p.createRigidBody(wall1, true);
		bodies.push_back({ fromGeometry(wall1), wall1_r });
		phyz::RigidBody* wall2_r = p.createRigidBody(wall2, true);
		bodies.push_back({ fromGeometry(wall2), wall2_r });

		phyz::RigidBody* ball_r = p.createRigidBody(ball);
		ball_r->setVel(mthz::Vec3(0, 0.1, 0));
		ball_r->setAngVel(mthz::Vec3(0, 0, -2));
		bodies.push_back({ fromGeometry(ball), ball_r });*/

		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(spinner2_r->getCOM().x, spinner2_r->getCOM().y, 10);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 240.0;
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

			t += fElapsedTime;

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				for (PhysBod b : bodies) {
					delete b.mesh.ib;
					delete b.mesh.va;
				}
				manager->deselectCurrentScene();
				return;
			}


			phyz_time += fElapsedTime / 2.0;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > timestep) {
				phyz_time -= timestep;
				p.timeStep();
			}


			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

			for (const PhysBod& b : bodies) {
				mthz::Vec3 cam_pos = pos;
				mthz::Quaternion cam_orient = orient;

				mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
				mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

				double aspect_ratio = (double)properties.window_height / properties.window_width;

				shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(cam_pos, cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
				shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0 * aspect_ratio, 120.0));
				shader.setUniform3f("u_ambient_light", 0.4, 0.4, 0.4);
				shader.setUniform3f("u_pointlight_pos", trnsfm_light_pos.x, trnsfm_light_pos.y, trnsfm_light_pos.z);
				shader.setUniform3f("u_pointlight_col", 0.6, 0.6, 0.6);
				shader.setUniform1i("u_Asleep", b.r->getAsleep());
				rndr::draw(*b.mesh.va, *b.mesh.ib, shader);

			}
		}
	}
};