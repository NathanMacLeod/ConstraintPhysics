#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class TrebuchetDemo : public DemoScene {
public:
	TrebuchetDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~TrebuchetDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"R", "Reset the trebuchet"},
			ControlDescription{"F", "Launch the trebuchet"},
			ControlDescription{"T", "Reset tower"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	void generateChain(phyz::PhysicsEngine* p, mthz::Vec3 start_pos, int n_links, double link_length, double link_width, std::vector<PhysBod>* bodies, phyz::RigidBody** first_link_out, phyz::RigidBody** final_link_out, mthz::Vec3* final_link_attach_p_out, std::vector<phyz::RigidBody*>* all_links_r ) {
		phyz::Geometry link_piece = phyz::Geometry::box(mthz::Vec3(-link_width / 6.0, -link_length + link_width / 6.0, -link_width / 6.0), link_width / 3.0, link_length, link_width / 3.0);
		phyz::Geometry split_piece = { link_piece.getTranslated(mthz::Vec3(link_width / 3.0, 0, 0)), link_piece.getTranslated(mthz::Vec3(-link_width / 3.0, 0, 0)) };
		double link_dist = link_length - link_width / 3.0;

		phyz::Geometry first_link = split_piece.getTranslated(start_pos);
		phyz::RigidBody* first_link_r = p->createRigidBody(first_link);
		bodies->push_back({ fromGeometry(first_link), first_link_r });
		all_links_r->push_back(first_link_r);
		
		phyz::RigidBody* prev_link_r = first_link_r;
		mthz::Vec3 prev_link_attach_point = start_pos + mthz::Vec3(0, -link_dist, 0);
		for (int i = 1; i < n_links; i++) {

			phyz::Geometry connecting_link = link_piece.getTranslated(prev_link_attach_point);
			phyz::RigidBody* connecting_link_r = p->createRigidBody(connecting_link);
			bodies->push_back({ fromGeometry(connecting_link), connecting_link_r });
			all_links_r->push_back(connecting_link_r);

			p->addHingeConstraint(connecting_link_r, prev_link_r, prev_link_attach_point, mthz::Vec3(1.0, 0, 0));

			mthz::Vec3 intermediate_joint_pos = prev_link_attach_point + mthz::Vec3(0, -link_dist, 0);

			phyz::Geometry open_link = split_piece.getTranslated(intermediate_joint_pos);
			phyz::RigidBody* open_link_r = p->createRigidBody(open_link);
			bodies->push_back({ fromGeometry(open_link),open_link_r });
			all_links_r->push_back(open_link_r);

			p->addHingeConstraint(connecting_link_r, open_link_r, intermediate_joint_pos, mthz::Vec3(1.0, 0, 0));

			prev_link_r = open_link_r;
			prev_link_attach_point = intermediate_joint_pos + mthz::Vec3(0, -link_dist, 0);
		}

		*first_link_out = first_link_r;
		*final_link_out = prev_link_r;
		*final_link_attach_p_out = prev_link_attach_point;
	}

	phyz::Geometry generateChainLoop(mthz::Vec3 pos, double chain_width, double loop_width, double loop_height, double base_length) {
		double fundamental_width = chain_width / 3.0;

		mthz::Vec3 base_pos = pos + mthz::Vec3(-fundamental_width / 2.0, 0, -fundamental_width / 2.0);
		phyz::Geometry base_piece = phyz::Geometry::box(base_pos + mthz::Vec3(0, -fundamental_width/2.0, 0), fundamental_width, base_length - fundamental_width/2.0, fundamental_width);

		mthz::Vec3 loop_pos = pos + mthz::Vec3(-loop_width / 2.0 - fundamental_width, base_length - fundamental_width, -fundamental_width / 2.0);
		phyz::Geometry loop1 = phyz::Geometry::box(loop_pos, loop_width + 2 * fundamental_width, fundamental_width, fundamental_width);
		phyz::Geometry loop2 = phyz::Geometry::box(loop_pos + mthz::Vec3(0, fundamental_width, 0), fundamental_width, loop_height, fundamental_width);
		phyz::Geometry loop3 = phyz::Geometry::box(loop_pos + mthz::Vec3(loop_width + fundamental_width, fundamental_width, 0), fundamental_width, loop_height, fundamental_width);
		phyz::Geometry loop4 = phyz::Geometry::box(loop_pos + mthz::Vec3(0, loop_height + fundamental_width, 0), loop_width + 2 * fundamental_width, fundamental_width, fundamental_width);

		return phyz::Geometry({ base_piece, loop1, loop2, loop3, loop4 });
	}

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "Wrecking Ball Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(true);
		p.setPGSIterations(55, 55);
		double timestep = 1 / 220.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -6.0, 0));

		bool lock_cam = true;

		std::vector<PhysBod> bodies;

		//************************
		//*******BASE PLATE*******
		//************************
		double s = 200;
		phyz::Geometry geom2 = phyz::Geometry::box(mthz::Vec3(-s / 2, -2, -s / 2), s, 2, s);
		Mesh m2 = fromGeometry(geom2);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, true);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		bodies.push_back({ m2, r2 });

		//************************
		//****TREBUCHET FRAME*****
		//************************
		mthz::Vec3 trebuchet_pos = mthz::Vec3(0, 0, 0);
		double arm_channel_width = 0.575;
		double rail_width = 0.25;
		double rail_length = 5;
		double rail_height = 2.5;
		double drop_channel_width = 0.15;
		double drop_channel_height = 5;
		double drop_channel_support_width = 0.33;

		mthz::Vec3 channel_beam1_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0 + rail_width, 0, -drop_channel_width / 2.0 - drop_channel_support_width);
		phyz::Geometry channel_beam1 = phyz::Geometry::box(channel_beam1_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		mthz::Vec3 channel_beam2_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0 + rail_width, 0, drop_channel_width / 2.0);
		phyz::Geometry channel_beam2 = phyz::Geometry::box(channel_beam2_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		phyz::Geometry drop_channel_cap1 = phyz::Geometry::box(channel_beam1_pos + mthz::Vec3(0, drop_channel_height, 0), drop_channel_support_width, drop_channel_support_width, 2 * drop_channel_support_width + drop_channel_width);

		mthz::Vec3 channel_beam3_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width - drop_channel_support_width, 0, -drop_channel_width / 2.0 - drop_channel_support_width);
		phyz::Geometry channel_beam3 = phyz::Geometry::box(channel_beam3_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		mthz::Vec3 channel_beam4_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width - drop_channel_support_width, 0, drop_channel_width / 2.0);
		phyz::Geometry channel_beam4 = phyz::Geometry::box(channel_beam4_pos, drop_channel_support_width, drop_channel_height, drop_channel_support_width);

		phyz::Geometry drop_channel_cap2 = phyz::Geometry::box(channel_beam3_pos + mthz::Vec3(0, drop_channel_height, 0), drop_channel_support_width, drop_channel_support_width, 2 * drop_channel_support_width + drop_channel_width);

		double each_rail_length = (rail_length - drop_channel_width) / 2.0;
		mthz::Vec3 rail1_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0, rail_height - rail_width, -rail_length / 2.0);
		phyz::Geometry rail1 = phyz::Geometry::box(rail1_pos, rail_width, rail_width, each_rail_length);

		mthz::Vec3 rail2_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0, rail_height - rail_width, drop_channel_width / 2.0);
		phyz::Geometry rail2 = phyz::Geometry::box(rail2_pos, rail_width, rail_width, each_rail_length);

		mthz::Vec3 rail3_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width, rail_height - rail_width, -rail_length / 2.0);
		phyz::Geometry rail3 = phyz::Geometry::box(rail3_pos, rail_width, rail_width, each_rail_length);

		mthz::Vec3 rail4_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width, rail_height - rail_width, drop_channel_width / 2.0);
		phyz::Geometry rail4 = phyz::Geometry::box(rail4_pos, rail_width, rail_width, each_rail_length);

		phyz::Geometry arm_rest_box = phyz::Geometry::box(rail3_pos + mthz::Vec3(rail_width, 0, 0), arm_channel_width, rail_width, rail_width);

		mthz::Vec3 bot_rail1_position = rail1_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y, 0);
		phyz::Geometry bot_rail1 = phyz::Geometry::box(bot_rail1_position, rail_width, rail_width, each_rail_length);
		
		mthz::Vec3 bot_rail2_position = rail2_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y, 0);
		phyz::Geometry bot_rail2 = phyz::Geometry::box(bot_rail2_position, rail_width, rail_width, each_rail_length);

		mthz::Vec3 bot_rail3_position = rail3_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y, 0);
		phyz::Geometry bot_rail3 = phyz::Geometry::box(bot_rail3_position, rail_width, rail_width, each_rail_length);

		mthz::Vec3 bot_rail4_position = rail4_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y, 0);
		phyz::Geometry bot_rail4 = phyz::Geometry::box(bot_rail4_position, rail_width, rail_width, each_rail_length);

		double leg_height = rail1_pos.y - trebuchet_pos.y;
		phyz::Geometry leg1 = phyz::Geometry::box(rail1_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y + rail_width, 0), rail_width, leg_height - rail_width, rail_width);
		phyz::Geometry leg2 = phyz::Geometry::box(rail2_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y + rail_width, each_rail_length - rail_width), rail_width, leg_height - rail_width, rail_width); 
		phyz::Geometry leg3 = phyz::Geometry::box(rail3_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y + rail_width, 0), rail_width, leg_height - rail_width, rail_width);
		phyz::Geometry leg4 = phyz::Geometry::box(rail4_pos + mthz::Vec3(0, trebuchet_pos.y - rail1_pos.y + rail_width, each_rail_length - rail_width), rail_width, leg_height - rail_width, rail_width);

		double support_width = drop_channel_support_width * 0.75;
		double support_height = 4;
		double support_floor_length = rail_length/2.0 - drop_channel_width/2.0 - drop_channel_support_width;
		double support_hypotenous = sqrt(support_height * support_height + support_floor_length + support_floor_length);
		double support_angle = PI/2.0 - atan(support_height / support_floor_length);
		double support_gap = 0.01;

		mthz::Vec3 support1_pos = bot_rail1_position + mthz::Vec3(rail_width + support_gap, 0, 0);
		phyz::Geometry support1 = phyz::Geometry::box(support1_pos, support_width, support_hypotenous, support_width)
			.getRotated(mthz::Quaternion(support_angle, mthz::Vec3(1, 0, 0)), support1_pos);

		mthz::Vec3 support2_pos = bot_rail2_position + mthz::Vec3(rail_width + support_gap, 0, each_rail_length);
		phyz::Geometry support2 = phyz::Geometry::box(support2_pos + mthz::Vec3(0, 0, -support_width), support_width, support_hypotenous, support_width)
			.getRotated(mthz::Quaternion(-support_angle, mthz::Vec3(1, 0, 0)), support2_pos);

		mthz::Vec3 support3_pos = bot_rail3_position + mthz::Vec3(-support_width - support_gap, 0, 0);
		phyz::Geometry support3 = phyz::Geometry::box(support3_pos, support_width, support_hypotenous, support_width)
			.getRotated(mthz::Quaternion(support_angle, mthz::Vec3(1, 0, 0)), support3_pos);

		mthz::Vec3 support4_pos = bot_rail4_position + mthz::Vec3(-support_width - support_gap, 0, each_rail_length);
		phyz::Geometry support4 = phyz::Geometry::box(support4_pos + mthz::Vec3(0, 0, -support_width), support_width, support_hypotenous, support_width)
			.getRotated(mthz::Quaternion(-support_angle, mthz::Vec3(1, 0, 0)), support4_pos);


		double weight_width = 0.6;
		double weight_thickness = 0.5;
		double weight_gap = 0.01;
		double weight_axle_radius = 0.06;
		double weight_height = drop_channel_height - weight_axle_radius;
		double weight_axle_excess = 0.05;

		phyz::Material weight_material = phyz::Material::modified_density(6);

		double weight_dist = arm_channel_width/2.0 + (rail_width + drop_channel_support_width + weight_gap);
		double weight_axle_length = 2 * (weight_dist + weight_thickness + weight_axle_excess);
		mthz::Vec3 weight_center = trebuchet_pos + mthz::Vec3(0, weight_height, 0);
		phyz::Geometry weight_axle = phyz::Geometry::cylinder(weight_center - mthz::Vec3(0, weight_axle_length / 2.0, 0), weight_axle_radius, weight_axle_length)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), weight_center);

		phyz::Geometry weight1 = phyz::Geometry::box(weight_center + mthz::Vec3(weight_dist, -weight_width / 2.0, -weight_width / 2.0), weight_thickness, weight_width, weight_width, weight_material);
		phyz::Geometry weight2 = phyz::Geometry::box(weight_center + mthz::Vec3(-weight_dist - weight_thickness, -weight_width / 2.0, -weight_width / 2.0), weight_thickness, weight_width, weight_width, weight_material);


		double release_pin_radius = 0.06;
		double release_pin_gap = 0.01;
		double release_pin_excess = 0.05;

		double release_pin_height = weight_height - weight_axle_radius - release_pin_radius - release_pin_gap;
		double release_pin_length = drop_channel_width + 2 * drop_channel_support_width + 2 * release_pin_excess;

		mthz::Vec3 release_pin1_pos = trebuchet_pos + mthz::Vec3(arm_channel_width / 2.0 + rail_width + drop_channel_support_width / 2.0, release_pin_height, -drop_channel_width / 2.0 - drop_channel_support_width - release_pin_excess);
		phyz::Geometry release_pin1 = phyz::Geometry::cylinder(release_pin1_pos, release_pin_radius, release_pin_length, 10, weight_material)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), release_pin1_pos);

		mthz::Vec3 release_pin2_pos = trebuchet_pos + mthz::Vec3(-arm_channel_width / 2.0 - rail_width - drop_channel_support_width / 2.0, release_pin_height, -drop_channel_width / 2.0 - drop_channel_support_width - release_pin_excess);
		phyz::Geometry release_pin2 = phyz::Geometry::cylinder(release_pin2_pos, release_pin_radius, release_pin_length, 10, weight_material)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(1, 0, 0)), release_pin2_pos);


		double trebuchet_arm_length = 5.75;
		double trebuchet_arm_height = 0.3;
		double trebuchet_arm_thickness = 0.3;

		mthz::Vec3 trebuchet_arm_pos = weight_center + mthz::Vec3(-trebuchet_arm_thickness / 2.0, -trebuchet_arm_height / 2.0, -trebuchet_arm_length + trebuchet_arm_height / 2.0);
		phyz::Geometry trebuchet_main_arm = phyz::Geometry::box(trebuchet_arm_pos, trebuchet_arm_thickness, trebuchet_arm_height, trebuchet_arm_length);

		double roller_distance = 2;
		double roller_axle_excess = 0.05;
		double roller_axle_radius = 0.06;
		double roller_gap_to_arm = 0.01;
		double roller_gap_to_wall = 0.075;

		double roller_axle_length = arm_channel_width + 2 * (rail_width - roller_gap_to_wall + roller_axle_excess);
		mthz::Vec3 roller_center_position = weight_center + mthz::Vec3(0, 0, -roller_distance);

		phyz::Geometry roller_axle = phyz::Geometry::cylinder(roller_center_position + mthz::Vec3(0, -roller_axle_length / 2.0, 0), roller_axle_radius, roller_axle_length)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), roller_center_position);

		double roller_radius = 0.25;
		double roller_length = arm_channel_width / 2.0 + rail_width - roller_gap_to_wall - roller_gap_to_arm - trebuchet_arm_thickness / 2.0;

		phyz::Geometry roller_wheel1 = phyz::Geometry::cylinder(roller_center_position + mthz::Vec3(0, trebuchet_arm_thickness/2.0 + roller_gap_to_arm, 0), roller_radius, roller_length)
			.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), roller_center_position);

		phyz::Geometry roller_wheel2 = phyz::Geometry::cylinder(roller_center_position + mthz::Vec3(0, trebuchet_arm_thickness / 2.0 + roller_gap_to_arm, 0), roller_radius, roller_length)
			.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1)), roller_center_position);

		double chain_loop_length = 0.7;
		double chain_loop_gap_size = 0.08;
		double chain_loop_thickness = 0.15;

		phyz::Geometry chain_loop_piece1 = phyz::Geometry::box(trebuchet_arm_pos + mthz::Vec3(0, - chain_loop_gap_size, 0), trebuchet_arm_thickness, chain_loop_gap_size, chain_loop_thickness);
		phyz::Geometry chain_loop_piece2 = phyz::Geometry::box(trebuchet_arm_pos + mthz::Vec3(0, - chain_loop_gap_size, chain_loop_length - chain_loop_thickness), trebuchet_arm_thickness, chain_loop_gap_size, chain_loop_thickness);
		phyz::Geometry chain_loop_piece3 = phyz::Geometry::box(trebuchet_arm_pos + mthz::Vec3(0, - chain_loop_gap_size - chain_loop_thickness, 0), trebuchet_arm_thickness, chain_loop_thickness, chain_loop_length);

		double hook_penetration = 0.1;
		mthz::Vec3 hook_position = trebuchet_arm_pos + mthz::Vec3(trebuchet_arm_thickness / 2.0, 0, 0);
		
		double hook_width = 0.1;
		double hook_length = 0.4;
		double hook_angle = 0;
		phyz::Geometry hook = phyz::Geometry::box(hook_position + mthz::Vec3(-hook_width / 2.0, -hook_penetration, -hook_width / 2.0), hook_width, hook_length + hook_penetration, hook_width)
			.getRotated(mthz::Quaternion(-PI / 2.0 + hook_angle, mthz::Vec3(1, 0, 0)), hook_position);

		double chain_arm_gap = 0.1;
		double chain_start_dist = 0.6;
		double chain_sepr_dist = 0.6;
		double chain_width = 0.15;
		double chain_link_length = 0.29;
		int n_links = 7;
		mthz::Vec3 chain1_pos = hook_position + mthz::Vec3(0, -chain_start_dist, -chain_arm_gap);
		mthz::Vec3 chain2_pos = chain1_pos + mthz::Vec3(0, 0, chain_sepr_dist);

		phyz::RigidBody* chain1_first_link_r;
		phyz::RigidBody* chain1_final_link_r;
		mthz::Vec3 chain1_final_link_pos;
		std::vector<phyz::RigidBody*> all_links_r;
		generateChain(&p, chain1_pos, n_links, chain_link_length, chain_width, &bodies, &chain1_first_link_r, &chain1_final_link_r, &chain1_final_link_pos, &all_links_r);

		phyz::RigidBody* chain2_first_link_r;
		phyz::RigidBody* chain2_final_link_r;
		mthz::Vec3 chain2_final_link_pos;
		generateChain(&p, chain2_pos, n_links, chain_link_length, chain_width, &bodies, &chain2_first_link_r, &chain2_final_link_r, &chain2_final_link_pos, &all_links_r);

		double chain1_loop_gap = 0.2;
		double chain1_loop_size = hook_width + chain1_loop_gap;
		double chain1_base_dist = hook_position.y - chain1_pos.y - (chain1_loop_size + chain1_loop_gap) / 2.0;
		phyz::Geometry chain1_loop = generateChainLoop(chain1_pos, chain_width, chain1_loop_size, chain1_loop_size, chain1_base_dist);

		double chain2_loop_width = trebuchet_arm_thickness + 0.05;
		double chain2_loop_height = 2 * chain_loop_thickness;
		double chain2_base_dist = trebuchet_arm_pos.y - chain2_pos.y - chain_loop_gap_size / 2.0 - chain2_loop_height - chain_width / 6.0;
		phyz::Geometry chain2_loop = generateChainLoop(chain2_pos, chain_width, chain2_loop_width, chain2_loop_height, chain2_base_dist);
		
		double bucket_clip_width = chain_width / 3.0;
		double bucket_clip_length = bucket_clip_width * 2;

		mthz::Vec3 bucket_clip1_pos = chain1_final_link_pos + mthz::Vec3(-bucket_clip_width / 2.0, -bucket_clip_width / 2.0, -bucket_clip_width / 2.0);
		phyz::Geometry bucket_clip1 = phyz::Geometry::box(bucket_clip1_pos, bucket_clip_width, bucket_clip_width, bucket_clip_length);

		mthz::Vec3 bucket_clip2_pos = chain2_final_link_pos + mthz::Vec3(-bucket_clip_width / 2.0, -bucket_clip_width / 2.0, - bucket_clip_length + bucket_clip_width / 2.0);
		phyz::Geometry bucket_clip2 = phyz::Geometry::box(bucket_clip2_pos, bucket_clip_width, bucket_clip_width, bucket_clip_length);

		double bucket_width = chain2_pos.z - chain1_pos.z - 2 * bucket_clip_length + bucket_clip_width;
		double bucket_wall_thickness = 0.06;
		double bucket_depth = 0.1;
		mthz::Vec3 bucket_pos = bucket_clip1_pos + mthz::Vec3(-bucket_width / 2.0 + bucket_clip_width / 2.0, -bucket_depth + bucket_clip_width, bucket_clip_length);

		double projectile_radius = 1.2 * bucket_width/2.0;
		phyz::Material projectile_material = phyz::Material::modified_density(0.2);
		mthz::Vec3 projectile_position = mthz::Vec3(chain1_final_link_pos.x, chain1_final_link_pos.y + projectile_radius - bucket_depth + bucket_wall_thickness, 0.5 * (chain1_final_link_pos.z + chain2_final_link_pos.z));
		phyz::Geometry projectile = phyz::Geometry::sphere(projectile_position, projectile_radius);

		phyz::Geometry bucket_floor = phyz::Geometry::box(bucket_pos, bucket_width, bucket_wall_thickness, bucket_width);
		phyz::Geometry bucket_wall1 = phyz::Geometry::box(bucket_pos + mthz::Vec3(0, bucket_wall_thickness, 0), bucket_width, bucket_depth - bucket_wall_thickness, bucket_wall_thickness);
		phyz::Geometry bucket_wall2 = phyz::Geometry::box(bucket_pos + mthz::Vec3(0, bucket_wall_thickness, bucket_wall_thickness), bucket_wall_thickness, bucket_depth - bucket_wall_thickness, bucket_width - 2 * bucket_wall_thickness);
		phyz::Geometry bucket_wall3 = phyz::Geometry::box(bucket_pos + mthz::Vec3(bucket_width - bucket_wall_thickness, bucket_wall_thickness, bucket_wall_thickness), bucket_wall_thickness, bucket_depth - bucket_wall_thickness, bucket_width - 2 * bucket_wall_thickness);
		phyz::Geometry bucket_wall4 = phyz::Geometry::box(bucket_pos + mthz::Vec3(0, bucket_wall_thickness, bucket_width - bucket_wall_thickness), bucket_width, bucket_depth - bucket_wall_thickness, bucket_wall_thickness);

		std::vector<phyz::RigidBody*> trebuchet_bodies;

		phyz::Geometry trebuchet_frame = { channel_beam1, channel_beam2, channel_beam3, channel_beam4, drop_channel_cap1, drop_channel_cap2, 
			rail1, rail2, rail3, rail4, bot_rail1, bot_rail2, bot_rail3, bot_rail4, arm_rest_box, leg1, leg2, leg3, leg4, support1, support2, support3, support4 };
		phyz::RigidBody* trebuchet_frame_r = p.createRigidBody(trebuchet_frame, true);

		phyz::Geometry release_pins = { release_pin1, release_pin2 };
		phyz::RigidBody* release_pins_r = p.createRigidBody(release_pins);

		phyz::Geometry weight = { weight_axle, weight1, weight2 };
		phyz::RigidBody* weight_r = p.createRigidBody(weight);

		phyz::Geometry trebuchet_arm = { trebuchet_main_arm, roller_axle, chain_loop_piece1, chain_loop_piece2, chain_loop_piece3, hook };
		phyz::RigidBody* trebuchet_arm_r = p.createRigidBody(trebuchet_arm, false);

		phyz::Geometry roller = { roller_wheel1, roller_wheel2 };
		phyz::RigidBody* roller_r = p.createRigidBody(roller);

		phyz::RigidBody* chain1_loop_r = p.createRigidBody(chain1_loop);
		phyz::RigidBody* chain2_loop_r = p.createRigidBody(chain2_loop);

		phyz::Geometry bucket = { bucket_clip1, bucket_clip2, bucket_floor, bucket_wall1, bucket_wall2, bucket_wall3, bucket_wall4 };
		phyz::RigidBody* bucket_r = p.createRigidBody(bucket);

		phyz::RigidBody* projectile_r = p.createRigidBody(projectile);

		bodies.push_back({ fromGeometry(trebuchet_frame), trebuchet_frame_r });
		bodies.push_back({ fromGeometry(weight), weight_r });
		bodies.push_back({ fromGeometry(release_pins), release_pins_r });
		bodies.push_back({ fromGeometry(trebuchet_arm), trebuchet_arm_r });
		bodies.push_back({ fromGeometry(roller), roller_r });
		bodies.push_back({ fromGeometry(chain1_loop), chain1_loop_r });
		bodies.push_back({ fromGeometry(chain2_loop), chain2_loop_r });
		bodies.push_back({ fromGeometry(bucket), bucket_r });
		bodies.push_back({ fromGeometry(projectile), projectile_r });

		phyz::ConstraintID release = p.addSliderConstraint(trebuchet_frame_r, release_pins_r, release_pins_r->getCOM(), mthz::Vec3(0, 0, -1), 350, 350, 0, 2 * release_pin_excess + drop_channel_width + drop_channel_support_width);
		p.setPiston(release, 10, -1);

		p.addHingeConstraint(trebuchet_arm_r, weight_r, weight_center, mthz::Vec3(1, 0, 0));

		p.addHingeConstraint(trebuchet_arm_r, roller_r, roller_center_position, mthz::Vec3(1, 0, 0));

		p.addHingeConstraint(chain1_first_link_r, chain1_loop_r, chain1_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(chain2_first_link_r, chain2_loop_r, chain2_pos, mthz::Vec3(1, 0, 0));

		p.addHingeConstraint(chain1_final_link_r, bucket_r, chain1_final_link_pos, mthz::Vec3(1, 0, 0));
		p.addHingeConstraint(chain2_final_link_r, bucket_r, chain2_final_link_pos, mthz::Vec3(1, 0, 0));

		bool weld_removed = false;
		phyz::ConstraintID hook_weld = p.addWeldConstraint(chain1_loop_r, trebuchet_arm_r, hook_position);
		phyz::ConstraintID projectile_weld = p.addWeldConstraint(bucket_r, projectile_r, projectile_position);

		mthz::Vec3 initial_bucket_force(0, 0.01, 0.55);
		bucket_r->applyForce(initial_bucket_force);

		trebuchet_bodies = { trebuchet_frame_r, release_pins_r, weight_r, trebuchet_arm_r, roller_r, chain1_loop_r, chain2_loop_r, bucket_r, projectile_r };
		for (phyz::RigidBody* l_r : all_links_r) { trebuchet_bodies.push_back(l_r); }


		//*************
		//****TOWER****
		//*************
		std::vector<phyz::RigidBody*> tower_bodies;

		if (true) {
			mthz::Vec3 tower_pos(0, 0, 70);
			double tower_width = 4;
			double tower_story_height = 1.25;
			double floor_height = 0.25;
			double pillar_width = 0.3;
			int n_stories = 2;
			phyz::Material tower_material = phyz::Material::modified_density(0.2);
			phyz::Geometry pillar = phyz::Geometry::box(mthz::Vec3(), pillar_width, tower_story_height, pillar_width, tower_material);
			phyz::Geometry floor_plate = phyz::Geometry::box(mthz::Vec3(), tower_width / 2.0, floor_height, tower_width / 2.0, tower_material);



			for (int i = 0; i < n_stories; i++) {
				mthz::Vec3 story_pos = tower_pos + mthz::Vec3(0, i * (floor_height + tower_story_height), 0);
				phyz::Geometry pillar1 = pillar.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, 0, -tower_width / 2.0));
				phyz::Geometry pillar2 = pillar.getTranslated(story_pos + mthz::Vec3(-pillar_width / 2.0, 0, -tower_width / 2.0));
				phyz::Geometry pillar3 = pillar.getTranslated(story_pos + mthz::Vec3(tower_width / 2.0 - pillar_width, 0, -tower_width / 2.0));
				phyz::Geometry pillar4 = pillar.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, 0, -pillar_width / 2.0));
				phyz::Geometry pillar5 = pillar.getTranslated(story_pos + mthz::Vec3(-pillar_width / 2.0, 0, -pillar_width / 2.0));
				phyz::Geometry pillar6 = pillar.getTranslated(story_pos + mthz::Vec3(tower_width / 2.0 - pillar_width, 0, -pillar_width / 2.0));
				phyz::Geometry pillar7 = pillar.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, 0, tower_width / 2.0 - pillar_width));
				phyz::Geometry pillar8 = pillar.getTranslated(story_pos + mthz::Vec3(-pillar_width / 2.0, 0, tower_width / 2.0 - pillar_width));
				phyz::Geometry pillar9 = pillar.getTranslated(story_pos + mthz::Vec3(tower_width / 2.0 - pillar_width, 0, tower_width / 2.0 - pillar_width));

				phyz::Geometry floor_plate1 = floor_plate.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, tower_story_height, -tower_width / 2.0));
				phyz::Geometry floor_plate2 = floor_plate.getTranslated(story_pos + mthz::Vec3(0, tower_story_height, -tower_width / 2.0));
				phyz::Geometry floor_plate3 = floor_plate.getTranslated(story_pos + mthz::Vec3(-tower_width / 2.0, tower_story_height, 0));
				phyz::Geometry floor_plate4 = floor_plate.getTranslated(story_pos + mthz::Vec3(0, tower_story_height, 0));

				phyz::RigidBody* pillar1_r = p.createRigidBody(pillar1);
				phyz::RigidBody* pillar2_r = p.createRigidBody(pillar2);
				phyz::RigidBody* pillar3_r = p.createRigidBody(pillar3);
				phyz::RigidBody* pillar4_r = p.createRigidBody(pillar4);
				phyz::RigidBody* pillar5_r = p.createRigidBody(pillar5);
				phyz::RigidBody* pillar6_r = p.createRigidBody(pillar6);
				phyz::RigidBody* pillar7_r = p.createRigidBody(pillar7);
				phyz::RigidBody* pillar8_r = p.createRigidBody(pillar8);
				phyz::RigidBody* pillar9_r = p.createRigidBody(pillar9);

				phyz::RigidBody* floor_plate1_r = p.createRigidBody(floor_plate1);
				phyz::RigidBody* floor_plate2_r = p.createRigidBody(floor_plate2);
				phyz::RigidBody* floor_plate3_r = p.createRigidBody(floor_plate3);
				phyz::RigidBody* floor_plate4_r = p.createRigidBody(floor_plate4);

				std::vector<phyz::RigidBody*> new_bodies = {
					pillar1_r, pillar2_r, pillar3_r, pillar4_r, pillar5_r, pillar6_r, pillar7_r, pillar8_r, pillar9_r,
					floor_plate1_r, floor_plate2_r, floor_plate3_r, floor_plate4_r
				};

				tower_bodies.insert(tower_bodies.end(), new_bodies.begin(), new_bodies.end());

				bodies.push_back({ fromGeometry(pillar1), pillar1_r });
				bodies.push_back({ fromGeometry(pillar2), pillar2_r });
				bodies.push_back({ fromGeometry(pillar3), pillar3_r });
				bodies.push_back({ fromGeometry(pillar4), pillar4_r });
				bodies.push_back({ fromGeometry(pillar5), pillar5_r });
				bodies.push_back({ fromGeometry(pillar6), pillar6_r });
				bodies.push_back({ fromGeometry(pillar7), pillar7_r });
				bodies.push_back({ fromGeometry(pillar8), pillar8_r });
				bodies.push_back({ fromGeometry(pillar9), pillar9_r });

				bodies.push_back({ fromGeometry(floor_plate1), floor_plate1_r });
				bodies.push_back({ fromGeometry(floor_plate2), floor_plate2_r });
				bodies.push_back({ fromGeometry(floor_plate3), floor_plate3_r });
				bodies.push_back({ fromGeometry(floor_plate4), floor_plate4_r });
			}
		}

		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(-10, 3, 0);
		mthz::Quaternion orient(-PI/2.0, mthz::Vec3(0, 1, 0));
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;

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

			if (rndr::getKeyDown(GLFW_KEY_F)) {
				p.setPiston(release, 30, 2);

				if (!weld_removed) {
					weld_removed = true;
					p.removeConstraint(hook_weld);
					p.removeConstraint(projectile_weld);
				}
			}

			//reset trebuchet
			if (rndr::getKeyDown(GLFW_KEY_R)) {
				for (phyz::RigidBody* r : trebuchet_bodies) {
					r->setOrientation(mthz::Quaternion());
					r->setToPosition(mthz::Vec3());
					r->setAngVel(mthz::Vec3());
					r->setVel(mthz::Vec3());
					p.deleteWarmstartData(r);
				}

				p.setPiston(release, 10, -1);

				if (!weld_removed) {
					p.removeConstraint(hook_weld);
					p.removeConstraint(projectile_weld);
				}
				weld_removed = false;
				hook_weld = p.addWeldConstraint(chain1_loop_r, trebuchet_arm_r, hook_position);
				projectile_weld = p.addWeldConstraint(bucket_r, projectile_r, projectile_position);
					

				bucket_r->applyForce(initial_bucket_force);
				
			}

			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				for (phyz::RigidBody* r : tower_bodies) {
					r->setOrientation(mthz::Quaternion());
					r->setToPosition(mthz::Vec3());
					r->setAngVel(mthz::Vec3());
					r->setVel(mthz::Vec3());
				}
			}

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				for (PhysBod b : bodies) {
					delete b.mesh.ib;
					delete b.mesh.va;
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

			for (const PhysBod& b : bodies) {
				mthz::Vec3 cam_pos = pos;
				mthz::Quaternion cam_orient = orient;

				mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
				mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

				double aspect_ratio = (double)properties.window_height / properties.window_width;

				shader.setUniformMat4f("u_MV", rndr::Mat4::cam_view(cam_pos, cam_orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
				shader.setUniformMat4f("u_P", rndr::Mat4::proj(0.1, 150.0, 2.0, 2.0 * aspect_ratio, 120.0));
				shader.setUniform3f("u_ambient_light", 0.4, 0.4, 0.4);
				shader.setUniform3f("u_pointlight_pos", trnsfm_light_pos.x, trnsfm_light_pos.y, trnsfm_light_pos.z);
				shader.setUniform3f("u_pointlight_col", 0.6, 0.6, 0.6);
				shader.setUniform1i("u_Asleep", b.r->getAsleep());
				rndr::draw(*b.mesh.va, *b.mesh.ib, shader);

			}
		}
	}
};