#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

const double pallet_width = 4.0;

class ForkliftDemo : public DemoScene {
public:
	ForkliftDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~ForkliftDemo() override {

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

	class Pallet {
	public:
		Pallet(mthz::Vec3 position, std::vector<PhysBod>* bodies, phyz::PhysicsEngine* pe, phyz::RigidBody* floor)
			: weld_broken(false), p(pe)
		{
			double pallet_height = 0.65;
			double board_thickness = 0.05;
			double board_width = 0.45;
			double spacer_height = pallet_height - 3 * board_thickness;
			int num_top_boards = 5;

			phyz::ConvexUnionGeometry xlong_board = phyz::ConvexUnionGeometry::box(mthz::Vec3(), pallet_width, board_thickness, board_width, phyz::Material::high_friction());
			phyz::ConvexUnionGeometry zlong_board = phyz::ConvexUnionGeometry::box(mthz::Vec3(), board_width, board_thickness, pallet_width, phyz::Material::high_friction());
			phyz::ConvexUnionGeometry spacer = phyz::ConvexUnionGeometry::box(mthz::Vec3(), board_width, spacer_height, board_width, phyz::Material::high_friction());

			double mid_d = (pallet_width - board_width) / 2.0;
			double far_d = pallet_width - board_width;

			phyz::ConvexUnionGeometry pallet_geom = {
				xlong_board.getTranslated(mthz::Vec3(0, 0, 0)), xlong_board.getTranslated(mthz::Vec3(0, 0, mid_d)), xlong_board.getTranslated(mthz::Vec3(0, 0, far_d)),

				spacer.getTranslated(mthz::Vec3(0, board_thickness, 0)), spacer.getTranslated(mthz::Vec3(mid_d, board_thickness, 0)), spacer.getTranslated(mthz::Vec3(far_d, board_thickness, 0)),
				spacer.getTranslated(mthz::Vec3(0, board_thickness, mid_d)), spacer.getTranslated(mthz::Vec3(mid_d, board_thickness, mid_d)), spacer.getTranslated(mthz::Vec3(far_d, board_thickness, mid_d)),
				spacer.getTranslated(mthz::Vec3(0, board_thickness, far_d)), spacer.getTranslated(mthz::Vec3(mid_d, board_thickness, far_d)), spacer.getTranslated(mthz::Vec3(far_d, board_thickness, far_d)),

				zlong_board.getTranslated(mthz::Vec3(0, board_thickness + spacer_height, 0)), zlong_board.getTranslated(mthz::Vec3(mid_d, board_thickness + spacer_height, 0)), zlong_board.getTranslated(mthz::Vec3(far_d, board_thickness + spacer_height, 0)),
			};
			for (int i = 0; i < num_top_boards; i++) {
				double dz = (pallet_width - board_width) / (num_top_boards - 1);
				phyz::ConvexUnionGeometry board = xlong_board.getTranslated(mthz::Vec3(0, 2 * board_thickness + spacer_height, i * dz));

				pallet_geom = phyz::ConvexUnionGeometry::merge(pallet_geom, board);
			}

			phyz::ConvexUnionGeometry pallet = pallet_geom.getTranslated(position);
			phyz::RigidBody* pallet_r = p->createRigidBody(pallet);

			bodies->push_back({ fromGeometry(pallet), pallet_r });

			//boxes
			double box_width = 1;
			double box_height = 0.5;
			int n_box_width = 3;
			int n_box_height = 4;
	
			for (int i = 0; i < n_box_width; i++) {
				for (int j = 0; j < n_box_width; j++) {
					for (int k = 0; k < n_box_height; k++) {
						phyz::Material box_material = phyz::Material::modified_density(0.1);
						phyz::ConvexUnionGeometry box = phyz::ConvexUnionGeometry::box(position + mthz::Vec3((pallet_width - n_box_width * box_width) / 2.0 + i * box_width, pallet_height + k * box_height, (pallet_width - n_box_width * box_width) / 2.0 + j * box_width), box_width, box_height, box_width, box_material);
						phyz::RigidBody* box_r = p->createRigidBody(box);
						boxes.push_back(box_r);
						bodies->push_back({ fromGeometry(box), box_r });

						welds.push_back(p->addWeldConstraint(pallet_r, box_r, box_r->getCOM()));
						p->registerCollisionAction(phyz::CollisionTarget::with(box_r), phyz::CollisionTarget::with(floor), [&](phyz::RigidBody* b1, phyz::RigidBody* b2, const std::vector<phyz::Manifold>& manifold) {
							if (!weld_broken) {
								weld_broken = true;
								for (phyz::ConstraintID c : welds) {
									p->removeConstraint(c);
								}

								for (int i = 0; i < boxes.size(); i++) {
									for (int j = i + 1; j < boxes.size(); j++) {
										p->reallowCollision(boxes[i], boxes[j]);
									}
								}
							}
						});
					}
				}
			}

			for (int i = 0; i < boxes.size(); i++) {
				for (int j = i + 1; j < boxes.size(); j++) {
					p->disallowCollision(boxes[i], boxes[j]);
				}
			}
		}

	private:
		bool weld_broken;
		std::vector<phyz::ConstraintID> welds;
		std::vector<phyz::RigidBody*> boxes;
		phyz::PhysicsEngine* p;
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

		//************************
		//*******Fork Lift********
		//************************
		//reference: https://youtu.be/mq18jajrx68?t=13

		mthz::Vec3 forklift_position = mthz::Vec3(0, 2, 0);
		mthz::Vec3 forward_dir(0, 0, 1);

		phyz::Material chasis_material = phyz::Material::modified_density(3.0);
		double chasis_base_width = 2.25;
		double chasis_base_length = 5;
		double chasis_base_height = 1;
		mthz::Vec3 chasis_base_pos = forklift_position + mthz::Vec3(-chasis_base_length / 2.0, -chasis_base_width / 2.0, -chasis_base_height / 2.0);
		phyz::ConvexUnionGeometry chasis_base = phyz::ConvexUnionGeometry::box(chasis_base_pos, chasis_base_width, chasis_base_height, chasis_base_length, chasis_material);

		double chasis_layer2_height = 0.6;
		double layer2_front_length = 0.5;
		double layer2_rear_length = 3.25;
		mthz::Vec3 layer2_front_block_position = chasis_base_pos + mthz::Vec3(0, chasis_base_height, chasis_base_length - layer2_front_length);
		phyz::ConvexUnionGeometry layer2_front_block = phyz::ConvexUnionGeometry::box(layer2_front_block_position, chasis_base_width, chasis_layer2_height, layer2_front_length);

		mthz::Vec3 layer2_rear_block_position = chasis_base_pos + mthz::Vec3(0, chasis_base_height, 0);
		phyz::ConvexUnionGeometry layer2_rear_block = phyz::ConvexUnionGeometry::box(layer2_rear_block_position, chasis_base_width, chasis_layer2_height, layer2_rear_length);

		double chasis_layer3_height = 0.4;
		double chasis_layer3_length = 2.5;
		mthz::Vec3 layer3_block_position = layer2_rear_block_position + mthz::Vec3(0, chasis_layer2_height, 0);
		phyz::ConvexUnionGeometry layer3_block = phyz::ConvexUnionGeometry::box(layer3_block_position, chasis_base_width, chasis_layer3_height, chasis_layer3_length);

		double roof_rear_support_length = 0.25;
		double roof_rear_support_width = 0.25;
		double roof_rear_support_height = 2.25;
		mthz::Vec3 roof_rear_support1_position = layer3_block_position + mthz::Vec3(0, chasis_layer3_height, chasis_layer3_length - roof_rear_support_length);
		phyz::ConvexUnionGeometry roof_rear_support1 = phyz::ConvexUnionGeometry::box(roof_rear_support1_position, roof_rear_support_width, roof_rear_support_height, roof_rear_support_length);

		mthz::Vec3 roof_rear_support2_position = layer3_block_position + mthz::Vec3(chasis_base_width - roof_rear_support_width, chasis_layer3_height, chasis_layer3_length - roof_rear_support_length);
		phyz::ConvexUnionGeometry roof_rear_support2 = phyz::ConvexUnionGeometry::box(roof_rear_support2_position, roof_rear_support_width, roof_rear_support_height, roof_rear_support_length);

		double roof_thickness = 0.2;
		double roof_length = 2.25;

		mthz::Vec3 roof_position = roof_rear_support1_position + mthz::Vec3(0, roof_rear_support_height, 0);
		phyz::ConvexUnionGeometry roof = phyz::ConvexUnionGeometry::box(roof_position, chasis_base_width, roof_thickness, roof_length);

		double front_support_gap = 0.01;
		double front_support_width = 0.25;
		double front_support_verticle_dist = roof_position.y + roof_thickness - layer2_front_block_position.y - chasis_layer2_height - front_support_gap;
		double front_support_horz_dist = chasis_base_pos.z + chasis_base_length - roof_position.z - roof_length;

		double front_support_length = sqrt(front_support_verticle_dist * front_support_verticle_dist + front_support_horz_dist * front_support_horz_dist);
		double front_support_angle = PI / 2.0 - atan(front_support_verticle_dist / front_support_horz_dist);

		mthz::Vec3 front_support1_position = layer2_front_block_position + mthz::Vec3(front_support_gap, chasis_layer2_height, layer2_front_length);
		phyz::ConvexUnionGeometry front_support1 = phyz::ConvexUnionGeometry::box(front_support1_position + mthz::Vec3(0, 0, -front_support_width), front_support_width, front_support_length, front_support_width)
			.getRotated(mthz::Quaternion(-front_support_angle, mthz::Vec3(1, 0, 0)), front_support1_position); 

		mthz::Vec3 front_support2_position = layer2_front_block_position + mthz::Vec3(chasis_base_width - front_support_width - front_support_gap, chasis_layer2_height, layer2_front_length);
		phyz::ConvexUnionGeometry front_support2 = phyz::ConvexUnionGeometry::box(front_support2_position + mthz::Vec3(0, 0, -front_support_width), front_support_width, front_support_length, front_support_width)
			.getRotated(mthz::Quaternion(-front_support_angle, mthz::Vec3(1, 0, 0)), front_support2_position);


		double front_wheel_radius = 0.9;
		double wheel_width = 0.55;
		double front_wheel_attach_height = 0.25;
		double rear_wheel_radius = 0.8;
		double rear_wheel_attach_height = front_wheel_attach_height - front_wheel_radius + rear_wheel_radius;
		double rear_wheel_attach_dist = 0.5;
		double front_wheel_attach_dist = chasis_base_length - 0.5;
		double wheel_attach_gap = 0.1;

		double suspension_block_size = 1.0;

		mthz::Vec3 front_wheel1_position = chasis_base_pos + mthz::Vec3(-wheel_attach_gap, front_wheel_attach_height, front_wheel_attach_dist);
		phyz::ConvexUnionGeometry front_wheel1 = phyz::ConvexUnionGeometry::cylinder(front_wheel1_position, front_wheel_radius, wheel_width, phyz::Material::super_friction())
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), front_wheel1_position);

		phyz::ConvexUnionGeometry suspension_block1 = phyz::ConvexUnionGeometry::box(front_wheel1_position + mthz::Vec3(0, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);

		mthz::Vec3 front_wheel2_position = chasis_base_pos + mthz::Vec3(chasis_base_width + wheel_attach_gap, front_wheel_attach_height, front_wheel_attach_dist);
		phyz::ConvexUnionGeometry front_wheel2 = phyz::ConvexUnionGeometry::cylinder(front_wheel2_position, front_wheel_radius, wheel_width, phyz::Material::super_friction())
			.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1)), front_wheel2_position);

		phyz::ConvexUnionGeometry suspension_block2 = phyz::ConvexUnionGeometry::box(front_wheel2_position + mthz::Vec3(-suspension_block_size, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);
		
		mthz::Vec3 rear_wheel1_position = chasis_base_pos + mthz::Vec3(-wheel_attach_gap, rear_wheel_attach_height, rear_wheel_attach_dist);
		phyz::ConvexUnionGeometry rear_wheel1 = phyz::ConvexUnionGeometry::cylinder(rear_wheel1_position, rear_wheel_radius, wheel_width, phyz::Material::high_friction())
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), rear_wheel1_position);

		phyz::ConvexUnionGeometry steering_block1 = phyz::ConvexUnionGeometry::box(rear_wheel1_position + mthz::Vec3(0, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);

		mthz::Vec3 rear_wheel2_position = chasis_base_pos + mthz::Vec3(chasis_base_width + wheel_attach_gap, rear_wheel_attach_height, rear_wheel_attach_dist);
		phyz::ConvexUnionGeometry rear_wheel2 = phyz::ConvexUnionGeometry::cylinder(rear_wheel2_position, rear_wheel_radius, wheel_width, phyz::Material::high_friction())
			.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1)), rear_wheel2_position);

		phyz::ConvexUnionGeometry steering_block2 = phyz::ConvexUnionGeometry::box(rear_wheel2_position + mthz::Vec3(-suspension_block_size, -suspension_block_size / 2.0, -suspension_block_size / 2.0), suspension_block_size, suspension_block_size, suspension_block_size);

		double mast_width = 0.4;
		double mast_height = 7;
		double mast_thickness = 0.35;
		double mast_gap = 0.25;
		double mast_separation = 0.8;

		mthz::Vec3 mast_position = chasis_base_pos + mthz::Vec3(chasis_base_width / 2.0, 0, chasis_base_length + mast_gap);

		mthz::Vec3 mast1_position = mast_position + mthz::Vec3(-mast_separation / 2.0 - mast_width, 0, 0);
		phyz::ConvexUnionGeometry mast1 = phyz::ConvexUnionGeometry::box(mast1_position, mast_width, mast_height, mast_thickness);

		mthz::Vec3 mast2_position = mast_position + mthz::Vec3(mast_separation / 2.0, 0, 0);
		phyz::ConvexUnionGeometry mast2 = phyz::ConvexUnionGeometry::box(mast2_position, mast_width, mast_height, mast_thickness);

		double backrest_height_diff = 0.2;
		double backrest_thickness = 0.25;
		double backrest_beam_thickness = 0.15;
		double backrest_section1_height = 1.25;
		double backrest_section2_height = 1.0;
		double backrest_width = 3;
		const int n_section2_supports = 7;
		const int n_section1_bars = 4;

		mthz::Vec3 backrest_position = mast_position + mthz::Vec3(-backrest_width / 2.0, -backrest_height_diff, mast_thickness);

		phyz::ConvexUnionGeometry backrest_vert_bar1 = phyz::ConvexUnionGeometry::box(backrest_position, backrest_beam_thickness, backrest_section1_height + backrest_section2_height, backrest_thickness);
		phyz::ConvexUnionGeometry backrest_vert_bar2 = phyz::ConvexUnionGeometry::box(backrest_position + mthz::Vec3(backrest_width - backrest_beam_thickness, 0, 0), backrest_beam_thickness, backrest_section1_height + backrest_section2_height, backrest_thickness);
		phyz::ConvexUnionGeometry backrest_bar = phyz::ConvexUnionGeometry::box(backrest_position + mthz::Vec3(backrest_beam_thickness, 0, 0), backrest_width - 2 * backrest_beam_thickness, backrest_beam_thickness, backrest_thickness);
		phyz::ConvexUnionGeometry backrest_top_bar = backrest_bar.getTranslated(mthz::Vec3(0, backrest_section1_height + backrest_section2_height - backrest_beam_thickness, 0));

		phyz::ConvexUnionGeometry backrest_bars[n_section1_bars];
		double dy = (backrest_section1_height - backrest_beam_thickness) / (n_section1_bars - 1);
		for (int i = 0; i < n_section1_bars; i++) {
			backrest_bars[i] = backrest_bar.getTranslated(mthz::Vec3(0, i*dy, 0));
		}

		phyz::ConvexUnionGeometry backrest_support = phyz::ConvexUnionGeometry::box(backrest_position + mthz::Vec3(0, backrest_section1_height, 0), backrest_beam_thickness, backrest_section2_height - backrest_beam_thickness, backrest_thickness);
		phyz::ConvexUnionGeometry backrest_supports[n_section2_supports - 2];
		double dx = (backrest_width - backrest_beam_thickness) / (n_section2_supports - 1);
		for (int i = 1; i < n_section2_supports - 1; i++) {
			backrest_supports[i - 1] = backrest_support.getTranslated(mthz::Vec3(dx * i, 0, 0));
		}
		
		double fork_width = 0.3;
		double fork_gap = 1.8;
		double fork_thickness = 0.1;
		double fork_depression = 0.2;
		double fork_height = 1.3;
		double fork_length = 4;

		mthz::Vec3 fork1_pos = backrest_position + mthz::Vec3(backrest_width / 2.0 - fork_gap / 2.0 - fork_width, -fork_depression, backrest_thickness);
		phyz::ConvexUnionGeometry fork1_vert = phyz::ConvexUnionGeometry::box(fork1_pos, fork_width, fork_height, fork_thickness);
		phyz::ConvexUnionGeometry fork1_horz = phyz::ConvexUnionGeometry::box(fork1_pos + mthz::Vec3(0, 0, fork_thickness), fork_width, fork_thickness, fork_length - fork_thickness, phyz::Material::high_friction());

		mthz::Vec3 fork2_pos = backrest_position + mthz::Vec3(backrest_width / 2.0 + fork_gap / 2.0, -fork_depression, backrest_thickness);
		phyz::ConvexUnionGeometry fork2_vert = phyz::ConvexUnionGeometry::box(fork2_pos, fork_width, fork_height, fork_thickness);
		phyz::ConvexUnionGeometry fork2_horz = phyz::ConvexUnionGeometry::box(fork2_pos + mthz::Vec3(0, 0, fork_thickness), fork_width, fork_thickness, fork_length - fork_thickness, phyz::Material::high_friction());


		phyz::ConvexUnionGeometry chasis = { chasis_base, layer2_front_block, layer2_rear_block, layer3_block, roof_rear_support1, roof_rear_support2, front_support1, front_support2, roof};
		phyz::RigidBody* chasis_r = p.createRigidBody(chasis);
		phyz::RigidBody::PKey lock_cam_pos = chasis_r->trackPoint(chasis_base_pos + mthz::Vec3(chasis_base_width/2.0, 3, 3));

		phyz::RigidBody* suspension_block1_r = p.createRigidBody(suspension_block1);
		phyz::RigidBody* suspension_block2_r = p.createRigidBody(suspension_block2);

		phyz::RigidBody* front_wheel1_r = p.createRigidBody(front_wheel1);
		phyz::RigidBody* front_wheel2_r = p.createRigidBody(front_wheel2);

		phyz::RigidBody* steering_block1_r = p.createRigidBody(steering_block1);
		phyz::RigidBody* steering_block2_r = p.createRigidBody(steering_block2);

		phyz::RigidBody* rear_wheel1_r = p.createRigidBody(rear_wheel1);
		phyz::RigidBody* rear_wheel2_r = p.createRigidBody(rear_wheel2);

		phyz::ConvexUnionGeometry mast = { mast1, mast2 };
		phyz::RigidBody* mast_r = p.createRigidBody(mast);

		phyz::ConvexUnionGeometry backrest = { backrest_vert_bar1, backrest_vert_bar2, backrest_top_bar, fork1_vert, fork1_horz, fork2_vert, fork2_horz };
		for (const phyz::ConvexUnionGeometry& b : backrest_bars) {
			backrest = phyz::ConvexUnionGeometry::merge(b, backrest);
		}
		for (const phyz::ConvexUnionGeometry& b : backrest_supports) {
			backrest = phyz::ConvexUnionGeometry::merge(b, backrest);
		}
		phyz::RigidBody* backrest_r = p.createRigidBody(backrest);

		double mast_rotation = 0.16;
		double mast_torque = 300;
		phyz::ConstraintID mast_motor = p.addMotorConstraint(
			p.addHingeConstraint(chasis_r, mast_r, mast_position, mthz::Vec3(1, 0, 0)),
			0, mast_rotation
		);

		double backrest_positive_slide_limit = 6.5;
		double backrest_negative_slide_limit = 0;
		double backrest_force = 300;
		phyz::ConstraintID backrest_piston = p.addPistonConstraint(
			p.addSliderConstraint(mast_r, backrest_r, mast_position, mthz::Vec3(0, 1, 0)),
			backrest_negative_slide_limit, backrest_positive_slide_limit
		);

		double break_torque = 150;
		double steer_torque = 40;
		double steer_max_angle = 0.5;
		double steer_angle = 0;
		double steer_speed = 1.2;
		double power_torque = 30;
		double forward_speed = 30;
		double reverse_speed = 10;
		double hard_suspension_limit_dist = std::numeric_limits<double>::infinity();
		double spring_dist = 1.0;
		double spring_stiffness = 900;
		double spring_damping = 25.5;

		p.addPistonConstraint(
			p.addSliderConstraint(chasis_r, suspension_block1_r, front_wheel1_position, mthz::Vec3(0, -1, 0)),
			-hard_suspension_limit_dist, hard_suspension_limit_dist
		);
		p.addPistonConstraint(
			p.addSliderConstraint(chasis_r, suspension_block2_r, front_wheel2_position, mthz::Vec3(0, -1, 0)),
			-hard_suspension_limit_dist, hard_suspension_limit_dist
		);

		
		phyz::ConstraintID hinge1 = p.addSlidingHingeConstraint(chasis_r, steering_block1_r, rear_wheel1_position, mthz::Vec3(0, -1, 0));
		phyz::ConstraintID steer_motor1 = p.addMotorConstraint(hinge1, -steer_max_angle, steer_max_angle);
		p.addPistonConstraint(hinge1, -hard_suspension_limit_dist, hard_suspension_limit_dist);
		phyz::ConstraintID hinge2 = p.addSlidingHingeConstraint(chasis_r, steering_block2_r, rear_wheel2_position, mthz::Vec3(0, -1, 0));
		phyz::ConstraintID steer_motor2 = p.addMotorConstraint(hinge2, -steer_max_angle, steer_max_angle);
		p.addPistonConstraint(hinge2, -hard_suspension_limit_dist, hard_suspension_limit_dist);

		p.addSpring(chasis_r, suspension_block1_r, front_wheel1_position + mthz::Vec3(0, spring_dist, 0), front_wheel1_position, spring_damping, spring_stiffness);
		p.addSpring(chasis_r, suspension_block2_r, front_wheel2_position + mthz::Vec3(0, spring_dist, 0), front_wheel2_position, spring_damping, spring_stiffness);
		p.addSpring(chasis_r, steering_block1_r, rear_wheel1_position + mthz::Vec3(0, spring_dist, 0), rear_wheel1_position, spring_damping, spring_stiffness);
		p.addSpring(chasis_r, steering_block2_r, rear_wheel2_position + mthz::Vec3(0, spring_dist, 0), rear_wheel2_position, spring_damping, spring_stiffness);

		phyz::ConstraintID power_motor1 = p.addMotorConstraint(p.addHingeConstraint(front_wheel1_r, suspension_block1_r, front_wheel1_position, mthz::Vec3(1, 0, 0)));
		phyz::ConstraintID power_motor2 = p.addMotorConstraint(p.addHingeConstraint(front_wheel2_r, suspension_block2_r, front_wheel2_position, mthz::Vec3(1, 0, 0)));

		p.addHingeConstraint(steering_block1_r, rear_wheel1_r, rear_wheel1_position, mthz::Vec3(-1, 0, 0));
		p.addHingeConstraint(steering_block2_r, rear_wheel2_r, rear_wheel2_position, mthz::Vec3(1, 0, 0));
		
		p.disallowCollision(rear_wheel1_r, chasis_r);
		p.disallowCollision(rear_wheel2_r, chasis_r);

		suspension_block1_r->setNoCollision(true);
		suspension_block2_r->setNoCollision(true);
		steering_block1_r->setNoCollision(true);
		steering_block2_r->setNoCollision(true);

		bodies.push_back({ fromGeometry(chasis), chasis_r });
		bodies.push_back({ fromGeometry(front_wheel1), front_wheel1_r });
		bodies.push_back({ fromGeometry(front_wheel2), front_wheel2_r });
		bodies.push_back({ fromGeometry(rear_wheel1), rear_wheel1_r });
		bodies.push_back({ fromGeometry(rear_wheel2), rear_wheel2_r });
		bodies.push_back({ fromGeometry(mast), mast_r });
		bodies.push_back({ fromGeometry(backrest), backrest_r });

		std::vector<Pallet*> pallets;

		//************************
		//*********Shelf**********
		//************************
		double shelf_height = 4;
		double shelf_width = 5;
		double shelf_thickness = 0.4;
		double leg_width = 0.4;
		int n_slots = 5;
		int pallet_prob = RAND_MAX / 2;

		mthz::Vec3 shelf_position(-shelf_width * n_slots / 2.0, 0, 20);
		phyz::ConvexUnionGeometry shelf = phyz::ConvexUnionGeometry::box(shelf_position + mthz::Vec3(0, shelf_height, 0), shelf_width * n_slots, shelf_thickness, shelf_width);
		phyz::RigidBody* shelf_r = p.createRigidBody(shelf, phyz::RigidBody::FIXED);
		bodies.push_back({ fromGeometry(shelf), shelf_r });

		phyz::ConvexUnionGeometry leg_shape = phyz::ConvexUnionGeometry::box(mthz::Vec3(), leg_width, shelf_height, leg_width);
		phyz::ConvexUnionGeometry leg1 = leg_shape.getTranslated(shelf_position);
		phyz::ConvexUnionGeometry leg2 = leg_shape.getTranslated(shelf_position + mthz::Vec3(0, 0, shelf_width - leg_width));

		phyz::RigidBody* leg1_r = p.createRigidBody(leg1, phyz::RigidBody::FIXED);
		phyz::RigidBody* leg2_r = p.createRigidBody(leg2, phyz::RigidBody::FIXED);

		bodies.push_back({ fromGeometry(leg1), leg1_r });
		bodies.push_back({ fromGeometry(leg2), leg2_r });

		for (int i = 0; i < n_slots; i++) {
			double leg_x = (shelf_width) * (1 + i);
			phyz::ConvexUnionGeometry close_leg = leg_shape.getTranslated(shelf_position + mthz::Vec3(leg_x - leg_width, 0, 0));
			phyz::ConvexUnionGeometry far_leg = leg_shape.getTranslated(shelf_position + mthz::Vec3(leg_x - leg_width, 0, shelf_width - leg_width));

			phyz::RigidBody* close_leg_r = p.createRigidBody(close_leg, phyz::RigidBody::FIXED);
			phyz::RigidBody* far_leg_r = p.createRigidBody(far_leg, phyz::RigidBody::FIXED);

			bodies.push_back({ fromGeometry(close_leg), close_leg_r });
			bodies.push_back({ fromGeometry(far_leg), far_leg_r });

			if (rand() > pallet_prob) {
				pallets.push_back(new Pallet(shelf_position + mthz::Vec3(leg_x - shelf_width + (shelf_width - pallet_width) / 2.0, 0, (shelf_width - pallet_width) / 2.0), &bodies, &p, r2));
			}
			if (rand() > pallet_prob) {
				pallets.push_back(new Pallet(shelf_position + mthz::Vec3(leg_x - shelf_width + (shelf_width - pallet_width) / 2.0, shelf_height + shelf_thickness, (shelf_width - pallet_width) / 2.0), &bodies, &p, r2));
			}
		}

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

			mthz::Vec3 oriented_forward = chasis_r->getOrientation().applyRotation(forward_dir);

			if (rndr::getKeyDown(GLFW_KEY_I)) {
				bool breaking = oriented_forward.dot(chasis_r->getVel()) < 0;

				p.setMotorTargetVelocity(power_motor1, breaking? break_torque : power_torque, forward_speed);
				p.setMotorTargetVelocity(power_motor2, breaking ? break_torque : power_torque, forward_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_K)) {
				bool breaking = oriented_forward.dot(chasis_r->getVel()) > 0;

				p.setMotorTargetVelocity(power_motor1, breaking ? break_torque : power_torque, -reverse_speed);
				p.setMotorTargetVelocity(power_motor2, breaking ? break_torque : power_torque, -reverse_speed);
			}
			else {
				p.setMotorTargetVelocity(power_motor1, power_torque, 0);
				p.setMotorTargetVelocity(power_motor2, power_torque, 0);
			}

			if (rndr::getKeyDown(GLFW_KEY_H)) {
				p.setMotorTargetVelocity(mast_motor, mast_torque, 0.15);
			}
			else if (rndr::getKeyDown(GLFW_KEY_F)) {
				p.setMotorTargetVelocity(mast_motor, mast_torque, -0.15);
			}
			else {
				p.setMotorTargetVelocity(mast_motor, mast_torque, 0);
			}

			if (rndr::getKeyDown(GLFW_KEY_T)) {
				p.setPiston(backrest_piston, backrest_force, 1);
			}
			else if (rndr::getKeyDown(GLFW_KEY_G)) {
				p.setPiston(backrest_piston, backrest_force, -1);
			}
			else {
				p.setPiston(backrest_piston, backrest_force, 0);
			}

			p.setMotorTargetPosition(steer_motor1, steer_torque, steer_angle);
			p.setMotorTargetPosition(steer_motor2, steer_torque, steer_angle);
			if (rndr::getKeyDown(GLFW_KEY_J)) {
				steer_angle = -steer_max_angle;
			}
			else if (rndr::getKeyDown(GLFW_KEY_L)) {
				steer_angle = steer_max_angle;
			}
			else {
				steer_angle = 0;
			}

			if (rndr::getKeyPressed(GLFW_KEY_Y)) {
				lock_cam = !lock_cam;
				
			}

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				for (Pallet* p : pallets) {
					delete p;
				}
				manager->deselectCurrentScene();
				return;
			}

			double slow_factor = 1;

			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > slow_factor * timestep) {
				phyz_time -= slow_factor * timestep;
				p.timeStep();
			}

			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = (lock_cam) ? chasis_r->getTrackedP(lock_cam_pos) : pos;
			mthz::Quaternion cam_orient = (lock_cam) ? chasis_r->getOrientation() * orient : orient;

			mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

			float aspect_ratio = (float)properties.window_height / properties.window_width;
			shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1f, 550.0f, 2.0f, 2.0f * aspect_ratio, 60.0f));
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

			rndr::draw(batch_array, shader);
		}
	}
};