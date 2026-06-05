#include "Common.h"
#include "cassert"

// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~
// ~=~=~=~=Duration Based State Transitions (used in many tests)~=~=~=~=
// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~

CurrentState get_current_state(const std::initializer_list<StateWithDuration>& states, float current_time) {
	assert(states.size() > 0);

	int i = 0;
	for (const StateWithDuration& s : states) {
		if (current_time <= s.duration) {
			return CurrentState{ false, s, current_time };
		}

		if (i + 1 == states.size()) {
			// time has passed the duration of the final element
			return CurrentState{ true, s, current_time };
		}

		current_time -= s.duration;
		i++;
	}

	// we should never hit this
	assert(false);
	return CurrentState{};
}

// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=
// ~=~=~=~=Scissor Lift~=~=~=~=
// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=

struct CreateScissorliftOut {
	phyz::RigidBody* top_front_right_rung_r;
	mthz::Vec3 top_front_right_pos;
	phyz::RigidBody* top_rear_right_rung_r;
	mthz::Vec3 top_rear_right_pos;
	phyz::RigidBody* top_front_left_rung_r;
	mthz::Vec3 top_front_left_pos;
	phyz::RigidBody* top_rear_left_rung_r;
	mthz::Vec3 top_rear_left_pos;
	phyz::RigidBody* bot_front_right_rung_r;
	mthz::Vec3 bot_front_right_pos;
	phyz::RigidBody* bot_rear_right_rung_r;
	mthz::Vec3 bot_rear_right_pos;
	phyz::RigidBody* bot_front_left_rung_r;
	mthz::Vec3 bot_front_left_pos;
	phyz::RigidBody* bot_rear_left_rung_r;
	mthz::Vec3 bot_rear_left_pos;

	mthz::Vec3 bot_left_middle_pos;
	mthz::Vec3 bot_right_middle_pos;
	mthz::Vec3 top_left_middle_pos;
	mthz::Vec3 top_right_middle_pos;
};

static CreateScissorliftOut createScissors(std::vector<PhysBod>* bodies, phyz::PhysicsEngine* p, mthz::Vec3 pos, float right_left_spacing, float rung_length, float rung_width, float rung_thickness, int num_levels) {
	CreateScissorliftOut out;

	float hinge_dist_from_edge = rung_width / 2;
	float rung_height = (rung_length - 2.0f * hinge_dist_from_edge) / sqrt(2.0f);
	color rung_color = { 0.4f, 0.4f, 0.1f };
	phyz::ConvexUnionGeometry rung = phyz::ConvexUnionGeometry::box(mthz::Vec3(0, -hinge_dist_from_edge, -hinge_dist_from_edge), rung_thickness, rung_length, rung_width);
	phyz::ConvexUnionGeometry rear_rung = rung.getRotated(mthz::Quaternion(PI / 4, mthz::Vec3(1, 0, 0)));
	phyz::ConvexUnionGeometry front_rung = rung.getRotated(mthz::Quaternion(-PI / 4, mthz::Vec3(1, 0, 0)));

	phyz::RigidBody* prev_rear_right_rung = nullptr;
	phyz::RigidBody* prev_front_right_rung = nullptr;
	phyz::RigidBody* prev_rear_left_rung = nullptr;
	phyz::RigidBody* prev_front_left_rung = nullptr;

	for (int i = 0; i < num_levels; i++) {

		mthz::Vec3 elevation_change(0, rung_height * i, 0);

		mthz::Vec3 rear_right_pos = pos + elevation_change;
		phyz::ConvexUnionGeometry rear_right_rung = rear_rung.getTranslated(rear_right_pos);
		mthz::Vec3 front_right_pos = pos + mthz::Vec3(rung_thickness, 0, rung_height) + elevation_change;
		phyz::ConvexUnionGeometry front_right_rung = front_rung.getTranslated(front_right_pos);
		mthz::Vec3 rear_left_pos = pos + mthz::Vec3(right_left_spacing - rung_thickness, 0, 0) + elevation_change;
		phyz::ConvexUnionGeometry rear_left_rung = rear_rung.getTranslated(rear_left_pos);
		mthz::Vec3 front_left_pos = pos + mthz::Vec3(right_left_spacing - 2 * rung_thickness, 0, rung_height) + elevation_change;
		phyz::ConvexUnionGeometry front_left_rung = front_rung.getTranslated(front_left_pos);

		phyz::RigidBody* rear_right_r = p->createRigidBody(rear_right_rung);
		bodies->push_back({ fromGeometry(rear_right_rung, rung_color), rear_right_r });
		phyz::RigidBody* front_right_r = p->createRigidBody(front_right_rung);
		bodies->push_back({ fromGeometry(front_right_rung, rung_color), front_right_r });
		phyz::RigidBody* rear_left_r = p->createRigidBody(rear_left_rung);
		bodies->push_back({ fromGeometry(rear_left_rung, rung_color), rear_left_r });
		phyz::RigidBody* front_left_r = p->createRigidBody(front_left_rung);
		bodies->push_back({ fromGeometry(front_left_rung, rung_color), front_left_r });

		mthz::Vec3 right_middle_attach_pos = rear_right_pos + mthz::Vec3(0, rung_height / 2.0, rung_height / 2.0);
		mthz::Vec3 left_middle_attach_pos = rear_left_pos + mthz::Vec3(0, rung_height / 2.0, rung_height / 2.0);

		p->addHingeConstraint(rear_right_r, front_right_r, right_middle_attach_pos, mthz::Vec3(1, 0, 0));
		p->addHingeConstraint(rear_left_r, front_left_r, left_middle_attach_pos, mthz::Vec3(1, 0, 0));

		if (i + 1 == num_levels) {
			out.top_front_left_pos = rear_left_pos + mthz::Vec3(0, rung_height, 0);
			out.top_front_left_rung_r = rear_left_r;
			out.top_front_right_pos = rear_right_pos + mthz::Vec3(0, rung_height, 0);
			out.top_front_right_rung_r = rear_right_r;
			out.top_rear_left_pos = front_left_pos + mthz::Vec3(0, rung_height, 0);
			out.top_rear_left_rung_r = front_left_r;
			out.top_rear_right_pos = front_right_pos + mthz::Vec3(0, rung_height, 0);
			out.top_rear_right_rung_r = front_right_r;

			out.top_left_middle_pos = left_middle_attach_pos;
			out.top_right_middle_pos = right_middle_attach_pos;
		}
		if (i == 0) {
			out.bot_front_left_pos = front_left_pos;
			out.bot_front_left_rung_r = front_left_r;
			out.bot_front_right_pos = front_right_pos;
			out.bot_front_right_rung_r = front_right_r;
			out.bot_rear_left_pos = rear_left_pos;
			out.bot_rear_left_rung_r = rear_left_r;
			out.bot_rear_right_pos = rear_right_pos;
			out.bot_rear_right_rung_r = rear_right_r;

			out.bot_left_middle_pos = left_middle_attach_pos;
			out.bot_right_middle_pos = right_middle_attach_pos;
		}
		else {
			p->addHingeConstraint(prev_front_right_rung, rear_right_r, rear_right_pos, mthz::Vec3(1, 0, 0));
			p->addHingeConstraint(prev_rear_right_rung, front_right_r, front_right_pos, mthz::Vec3(1, 0, 0));
			p->addHingeConstraint(prev_front_left_rung, rear_left_r, rear_left_pos, mthz::Vec3(1, 0, 0));
			p->addHingeConstraint(prev_rear_left_rung, front_left_r, front_left_pos, mthz::Vec3(1, 0, 0));
		}

		prev_front_right_rung = front_right_r;
		prev_rear_right_rung = rear_right_r;
		prev_front_left_rung = front_left_r;
		prev_rear_left_rung = rear_left_r;
	}

	return out;
}

ScissorLiftConstruct create_scissor_lift(phyz::PhysicsEngine* p, std::vector<PhysBod>* body_dest, mthz::Vec3 scissor_lift_pos) {
	float base_width = 1.0f;
	float base_length = 3.0f;
	float base_thickness = 0.1f;
	mthz::Vec3 base_position = scissor_lift_pos + mthz::Vec3(-base_width / 2.0, 0, -base_length / 2.0);
	phyz::ConvexUnionGeometry base_bottom = phyz::ConvexUnionGeometry::box(base_position, base_width, base_thickness, base_length);

	float base_side_height = 0.3f;
	color base_color = { 0.5f, 0.1f, 0.1f };
	mthz::Vec3 base_rear_wall_pos = base_position + mthz::Vec3(0, base_thickness, 0);
	phyz::ConvexUnionGeometry base_rear_wall = phyz::ConvexUnionGeometry::box(base_rear_wall_pos, base_width, base_side_height, base_thickness);

	mthz::Vec3 base_front_wall_pos = base_rear_wall_pos + mthz::Vec3(0, 0, base_length - base_thickness);
	phyz::ConvexUnionGeometry base_front_wall = phyz::ConvexUnionGeometry::box(base_front_wall_pos, base_width, base_side_height, base_thickness);

	mthz::Vec3 base_right_wall_pos = base_rear_wall_pos + mthz::Vec3(0, 0, base_thickness);
	phyz::ConvexUnionGeometry base_right_wall = phyz::ConvexUnionGeometry::box(base_right_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);

	mthz::Vec3 base_left_wall_pos = base_rear_wall_pos + mthz::Vec3(base_width - base_thickness, 0, base_thickness);
	phyz::ConvexUnionGeometry base_left_wall = phyz::ConvexUnionGeometry::box(base_left_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);

	phyz::ConvexUnionGeometry base_geom = { base_bottom, base_rear_wall, base_front_wall, base_right_wall, base_left_wall };

	float wheel_radius = 0.3f;
	float wheel_thickness = 0.15f;
	float wheel_dist_off_bottom = 0.1f;
	float wheel_dist_off_front = 0.1f;
	color wheel_color = { 0.1f, 0.1f, 0.1f };
	phyz::ConvexUnionGeometry wheel = phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), wheel_radius, wheel_thickness).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)));

	mthz::Vec3 front_right_wheel_pos = base_position + mthz::Vec3(0, wheel_dist_off_bottom, base_length - wheel_dist_off_front);
	phyz::ConvexUnionGeometry front_right_wheel = wheel.getTranslated(mthz::Vec3(front_right_wheel_pos));

	mthz::Vec3 rear_right_wheel_pos = base_position + mthz::Vec3(0, wheel_dist_off_bottom, wheel_dist_off_front);
	phyz::ConvexUnionGeometry rear_right_wheel = wheel.getTranslated(mthz::Vec3(rear_right_wheel_pos));

	mthz::Vec3 front_left_wheel_pos = base_position + mthz::Vec3(base_width + wheel_thickness, wheel_dist_off_bottom, base_length - wheel_dist_off_front);
	phyz::ConvexUnionGeometry front_left_wheel = wheel.getTranslated(mthz::Vec3(front_left_wheel_pos));

	mthz::Vec3 rear_left_wheel_pos = base_position + mthz::Vec3(base_width + wheel_thickness, wheel_dist_off_bottom, wheel_dist_off_front);
	phyz::ConvexUnionGeometry rear_left_wheel = wheel.getTranslated(mthz::Vec3(rear_left_wheel_pos));

	float dist_from_rear = 0.33f;
	float dist_above_base = 0.1f;
	float scissor_thickness = 0.03f;
	CreateScissorliftOut scissor_out = createScissors(body_dest, p, base_rear_wall_pos + mthz::Vec3(base_thickness, dist_above_base, dist_from_rear), base_width - 2.0f * base_thickness, 1.0f, 0.1f, scissor_thickness, 7);

	float slider_width = 0.1f;
	color sider_color = { 0.6f, 0.6f, 0.6f };
	mthz::Vec3 right_bottom_slider_pos = scissor_out.bot_front_right_pos + mthz::Vec3(-scissor_thickness, -slider_width / 2.0, -slider_width / 2.0);
	phyz::ConvexUnionGeometry bottom_right_slider = phyz::ConvexUnionGeometry::box(right_bottom_slider_pos, scissor_thickness, slider_width, slider_width, phyz::Material::modified_density(1000));
	phyz::ConvexUnionGeometry bottom_left_slider = bottom_right_slider.getTranslated(mthz::Vec3(base_width - scissor_thickness - 2 * base_thickness, 0, 0));
	phyz::ConvexUnionGeometry bottom_slider = { bottom_right_slider, bottom_left_slider };

	mthz::Vec3 right_top_slider_pos = scissor_out.top_front_right_pos + mthz::Vec3(0, -slider_width / 2.0, -slider_width / 2.0);
	phyz::ConvexUnionGeometry top_right_slider = phyz::ConvexUnionGeometry::box(right_top_slider_pos, scissor_thickness, slider_width, slider_width, phyz::Material::modified_density(1000));
	mthz::Vec3 left_top_slider_pos = right_top_slider_pos + mthz::Vec3(base_width - scissor_thickness - 2 * base_thickness, 0, 0);
	phyz::ConvexUnionGeometry top_left_slider = phyz::ConvexUnionGeometry::box(left_top_slider_pos, scissor_thickness, slider_width, slider_width);
	phyz::ConvexUnionGeometry top_slider = { top_right_slider, top_left_slider };

	color platform_color = { 0.1f, 0.6f, 0.1f };
	mthz::Vec3 platform_pos = mthz::Vec3(base_position.x, right_top_slider_pos.y + right_bottom_slider_pos.y - base_position.y - base_thickness, base_position.z);
	phyz::ConvexUnionGeometry base_platform = phyz::ConvexUnionGeometry::box(platform_pos, base_width, base_thickness, base_length);

	mthz::Vec3 platform_rear_wall_pos = platform_pos + mthz::Vec3(0, -base_side_height, 0);
	phyz::ConvexUnionGeometry platform_rear_wall = phyz::ConvexUnionGeometry::box(platform_rear_wall_pos, base_width, base_side_height, base_thickness);

	mthz::Vec3 platform_front_wall_pos = platform_rear_wall_pos + mthz::Vec3(0, 0, base_length - base_thickness);
	phyz::ConvexUnionGeometry platform_front_wall = phyz::ConvexUnionGeometry::box(platform_front_wall_pos, base_width, base_side_height, base_thickness);

	mthz::Vec3 platform_right_wall_pos = platform_rear_wall_pos + mthz::Vec3(0, 0, base_thickness);
	phyz::ConvexUnionGeometry platform_right_wall = phyz::ConvexUnionGeometry::box(platform_right_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);

	mthz::Vec3 platform_left_wall_pos = platform_rear_wall_pos + mthz::Vec3(base_width - base_thickness, 0, base_thickness);
	phyz::ConvexUnionGeometry platform_left_wall = phyz::ConvexUnionGeometry::box(platform_left_wall_pos, base_thickness, base_side_height, base_length - 2 * base_thickness);

	phyz::ConvexUnionGeometry platform_geom = { base_platform, platform_rear_wall, platform_front_wall, platform_right_wall, platform_left_wall };

	phyz::RigidBody* base_r = p->createRigidBody(base_geom);
	body_dest->push_back({ fromGeometry(base_geom, base_color), base_r });
	phyz::RigidBody* front_right_wheel_r = p->createRigidBody(front_right_wheel);
	body_dest->push_back({ fromGeometry(front_right_wheel, wheel_color), front_right_wheel_r });
	phyz::RigidBody* rear_right_wheel_r = p->createRigidBody(rear_right_wheel);
	body_dest->push_back({ fromGeometry(rear_right_wheel, wheel_color), rear_right_wheel_r });
	phyz::RigidBody* front_left_wheel_r = p->createRigidBody(front_left_wheel);
	body_dest->push_back({ fromGeometry(front_left_wheel, wheel_color), front_left_wheel_r });
	phyz::RigidBody* rear_left_wheel_r = p->createRigidBody(rear_left_wheel);
	body_dest->push_back({ fromGeometry(rear_left_wheel, wheel_color), rear_left_wheel_r });
	phyz::RigidBody* bottom_slider_r = p->createRigidBody(bottom_slider);
	body_dest->push_back({ fromGeometry(bottom_slider, sider_color), bottom_slider_r });
	phyz::RigidBody* top_slider_r = p->createRigidBody(top_slider);
	body_dest->push_back({ fromGeometry(top_slider, sider_color), top_slider_r });
	phyz::RigidBody* platform_r = p->createRigidBody(platform_geom);
	body_dest->push_back({ fromGeometry(platform_geom, platform_color), platform_r });

	double inf = std::numeric_limits<double>::infinity();

	p->addHingeConstraint(base_r, front_right_wheel_r, front_right_wheel_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(base_r, rear_right_wheel_r, rear_right_wheel_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(base_r, front_left_wheel_r, front_left_wheel_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(base_r, rear_left_wheel_r, rear_left_wheel_pos, mthz::Vec3(1, 0, 0));

	p->addHingeConstraint(base_r, scissor_out.bot_rear_right_rung_r, scissor_out.bot_rear_right_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(base_r, scissor_out.bot_rear_left_rung_r, scissor_out.bot_rear_left_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(bottom_slider_r, scissor_out.bot_front_right_rung_r, scissor_out.bot_front_right_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(bottom_slider_r, scissor_out.bot_front_left_rung_r, scissor_out.bot_front_left_pos, mthz::Vec3(1, 0, 0));
	p->addSliderConstraint(base_r, bottom_slider_r, bottom_slider_r->getCOM(), mthz::Vec3(0, 0, 1));

	p->addHingeConstraint(top_slider_r, scissor_out.top_rear_right_rung_r, scissor_out.top_front_right_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(top_slider_r, scissor_out.top_rear_left_rung_r, scissor_out.top_front_left_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(platform_r, scissor_out.top_front_right_rung_r, scissor_out.top_rear_right_pos, mthz::Vec3(1, 0, 0));
	p->addHingeConstraint(platform_r, scissor_out.top_front_left_rung_r, scissor_out.top_rear_left_pos, mthz::Vec3(1, 0, 0));
	p->addSliderConstraint(platform_r, top_slider_r, top_slider_r->getCOM(), mthz::Vec3(0, 0, 1));

	ScissorLiftConstruct out;

	out.top_platform = platform_r;
	out.right_distance_constraint = p->addDistanceConstraint(scissor_out.bot_rear_right_rung_r, scissor_out.top_rear_right_rung_r, scissor_out.bot_right_middle_pos, scissor_out.top_right_middle_pos);
	out.left_distance_constraint = p->addDistanceConstraint(scissor_out.bot_rear_left_rung_r, scissor_out.top_rear_left_rung_r, scissor_out.bot_left_middle_pos, scissor_out.top_left_middle_pos);
	out.state = ScissorLiftConstruct::MovementState::FIXED;
	out.max_height = 4.45f;
	out.min_height = 0.7f;
	out.move_speed = 1.5f;

	return out;
}


void set_scissor_lift_movement_input(phyz::PhysicsEngine* p, ScissorLiftConstruct* s, ScissorLiftConstruct::MovementState desired_state) {
	float current_left_dist = static_cast<float>(p->getDistanceConstraintCurrentDistance(s->left_distance_constraint));
	float current_right_dist = static_cast<float>(p->getDistanceConstraintCurrentDistance(s->right_distance_constraint));
	float dist_diff = current_left_dist - current_right_dist;
	float catchup_multiplier = 1.0f + 15.0f * abs(dist_diff);
	float left_multipier, right_multiplier;

	if ((desired_state == ScissorLiftConstruct::MovementState::RAISING && current_left_dist >= s->max_height) ||
		(desired_state == ScissorLiftConstruct::MovementState::LOWERING && current_left_dist <= s->min_height)) {
		// we want to raise / lower, but it would move beyond the boundary. in this case we will just fix at the boundary value

		s->state = ScissorLiftConstruct::MovementState::FIXED;
		float bound = (desired_state == ScissorLiftConstruct::MovementState::RAISING)? s->max_height : s->min_height;
		p->setDistanceConstraintTargetDistance(s->right_distance_constraint, bound);
		p->setDistanceConstraintTargetDistance(s->left_distance_constraint, bound);
	}
	else if (desired_state == ScissorLiftConstruct::MovementState::RAISING) {
		s->state = ScissorLiftConstruct::MovementState::RAISING;
		if (dist_diff < 0) { left_multipier = catchup_multiplier; right_multiplier = 1.0; }
		else               { left_multipier = 1.0;                right_multiplier = catchup_multiplier; }
		p->setDistanceConstraintToMoveAtTargetVelocity(s->right_distance_constraint, s->move_speed * right_multiplier);
		p->setDistanceConstraintToMoveAtTargetVelocity(s->left_distance_constraint, s->move_speed * left_multipier);
	}
	else if (desired_state == ScissorLiftConstruct::MovementState::LOWERING) {
		s->state = ScissorLiftConstruct::MovementState::LOWERING;
		if (dist_diff < 0) { left_multipier = 1.0;                right_multiplier = catchup_multiplier; }
		else               { left_multipier = catchup_multiplier; right_multiplier = 1.0; }
		p->setDistanceConstraintToMoveAtTargetVelocity(s->right_distance_constraint, -s->move_speed * right_multiplier);
		p->setDistanceConstraintToMoveAtTargetVelocity(s->left_distance_constraint, -s->move_speed * left_multipier);
	}
	else if (desired_state == ScissorLiftConstruct::MovementState::FIXED) {
		//fixed
		s->state = ScissorLiftConstruct::MovementState::FIXED;
		p->setDistanceConstraintTargetDistance(s->right_distance_constraint, current_left_dist);
		p->setDistanceConstraintTargetDistance(s->left_distance_constraint, current_left_dist);
	}
}

// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
// ~=~=~=~=Circular Tower~=~=~=~=
// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
std::vector<phyz::RigidBody*> createCircularTower(phyz::PhysicsEngine* p, std::vector<PhysBod>* body_dest, mthz::Vec3 block_dim, double radius, double n_blocks_per_layer, mthz::Vec3 pos, int n_layers) {
	std::vector<phyz::RigidBody*> out;
	phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(mthz::Vec3(0.0, 0.0, -block_dim.z / 2.0), block_dim.x, block_dim.y, block_dim.z);

	double dtheta = 2 * PI / n_blocks_per_layer;
	for (int i = 0; i < n_layers; i++) {
		double theta_offset = i * dtheta / 2;
		for (int j = 0; j < n_blocks_per_layer; j++) {
			double theta = theta_offset + dtheta * j;
			mthz::Quaternion rotation(theta, mthz::Vec3(0, 1, 0));

			mthz::Vec3 block_pos = pos + mthz::Vec3(0.0, block_dim.y * i, 0.0) + rotation.applyRotation(mthz::Vec3(radius, 0, 0));

			phyz::ConvexUnionGeometry block_oriented = block.getRotated(rotation).getTranslated(block_pos);
			phyz::RigidBody* r = p->createRigidBody(block_oriented);
			Mesh m = { fromGeometry(block_oriented) };
			body_dest->push_back(PhysBod{ m, r });
			out.push_back(r);
		}
	}
	
	return out;
}

// ~=~=~=~=~=~=~=~=~=~=~=~
// ~=~=~=~=Ragdoll~=~=~=~=
// ~=~=~=~=~=~=~=~=~=~=~=~
std::vector<phyz::RigidBody*> createRagdoll(phyz::PhysicsEngine* p, std::vector<PhysBod>* body_dest, mthz::Vec3 pos, double scale) {

	std::vector<phyz::RigidBody*> out;
	color col = color{ 0.8f, 0.5f, 0.1f, 0.5f, 0.5f, 0.63f, 51.2f };

	// chest

	double shoulder_width = 0.45 * scale;
	double shoulder_radius = 0.288 * scale;
	phyz::ConvexUnionGeometry shoulders_geom = phyz::ConvexUnionGeometry::capsule(pos - mthz::Vec3(0, shoulder_width / 2.0, 0), shoulder_radius, shoulder_width).getRotated(mthz::Quaternion(PI/2.0, mthz::Vec3(0, 0, 1)), pos);
	phyz::RigidBody* shoulder_r = p->createRigidBody(shoulders_geom);
	out.push_back(shoulder_r);
	body_dest->push_back(PhysBod{ fromGeometry(shoulders_geom, col), shoulder_r });

	double chest_width = 0.423 * scale;
	double chest_radius = 0.25 * scale;
	mthz::Vec3 chest_position = pos + mthz::Vec3(0, -0.345 * scale, 0);
	phyz::ConvexUnionGeometry chest_geom = phyz::ConvexUnionGeometry::capsule(chest_position - mthz::Vec3(0, chest_width / 2.0, 0), chest_radius, chest_width).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), chest_position);
	phyz::RigidBody* chest_r = p->createRigidBody(chest_geom);
	out.push_back(chest_r);
	body_dest->push_back(PhysBod{ fromGeometry(chest_geom, col), chest_r });

	double abdomen_width = 0.345 * scale;
	double abdomen_radius = 0.23 * scale;
	mthz::Vec3 abdomen_position = chest_position + mthz::Vec3(0, -0.385 * scale, 0);
	phyz::ConvexUnionGeometry abdomen_geom = phyz::ConvexUnionGeometry::capsule(abdomen_position - mthz::Vec3(0, abdomen_width / 2.0, 0), abdomen_radius, abdomen_width).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), abdomen_position);
	phyz::RigidBody* abdomen_r = p->createRigidBody(abdomen_geom);
	out.push_back(abdomen_r);
	body_dest->push_back(PhysBod{ fromGeometry(abdomen_geom, col), abdomen_r });

	double hip_width = 0.39 * scale;
	double hip_radius = 0.2 * scale;
	mthz::Vec3 hip_position = abdomen_position + mthz::Vec3(0, -0.288 * scale, 0);
	phyz::ConvexUnionGeometry hip_geom = phyz::ConvexUnionGeometry::capsule(hip_position - mthz::Vec3(0, hip_width / 2.0, 0), hip_radius, hip_width).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)), hip_position);
	phyz::RigidBody* hip_r = p->createRigidBody(hip_geom);
	out.push_back(hip_r);
	body_dest->push_back(PhysBod{ fromGeometry(hip_geom, col), hip_r });

	// legs

	double thigh_horz_offset = 0.25 * scale;
	double thigh_vericle_offset = 0.25 * scale;
	double thigh_radius = 0.212 * scale;
	double thigh_length = 0.9 * scale;
	phyz::ConvexUnionGeometry thigh_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(), thigh_radius, thigh_length).getRotated(mthz::Quaternion(PI, mthz::Vec3(0, 0, 1)));

	mthz::Vec3 right_thigh_pos = hip_position + mthz::Vec3(-thigh_horz_offset, -thigh_vericle_offset, 0);
	phyz::ConvexUnionGeometry right_thigh_geom = thigh_geom.getTranslated(right_thigh_pos);
	phyz::RigidBody* right_thigh_r = p->createRigidBody(right_thigh_geom);
	out.push_back(right_thigh_r);
	body_dest->push_back(PhysBod{ fromGeometry(right_thigh_geom, col), right_thigh_r });

	mthz::Vec3 left_thigh_pos = hip_position + mthz::Vec3(thigh_horz_offset, -thigh_vericle_offset, 0);
	phyz::ConvexUnionGeometry left_thigh_geom = thigh_geom.getTranslated(left_thigh_pos);
	phyz::RigidBody* left_thigh_r = p->createRigidBody(left_thigh_geom);
	out.push_back(left_thigh_r);
	body_dest->push_back(PhysBod{ fromGeometry(left_thigh_geom, col), left_thigh_r });

	double shin_length = 0.9 * scale;
	double shin_radius = 0.142 * scale;
	double shift_verticle_offset = thigh_length + thigh_radius;
	phyz::ConvexUnionGeometry shin_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(), shin_radius, shin_length).getRotated(mthz::Quaternion(PI, mthz::Vec3(0, 0, 1)));

	mthz::Vec3 right_shin_pos = right_thigh_pos + mthz::Vec3(0, -shift_verticle_offset, 0);
	phyz::ConvexUnionGeometry right_shin_geom = shin_geom.getTranslated(right_shin_pos);
	phyz::RigidBody* right_shin_r = p->createRigidBody(right_shin_geom);
	out.push_back(right_shin_r);
	body_dest->push_back(PhysBod{ fromGeometry(right_shin_geom, col), right_shin_r });

	mthz::Vec3 left_shin_pos = left_thigh_pos + mthz::Vec3(0, -shift_verticle_offset, 0);
	phyz::ConvexUnionGeometry left_shin_geom = shin_geom.getTranslated(left_shin_pos);
	phyz::RigidBody* left_shin_r = p->createRigidBody(left_shin_geom);
	out.push_back(left_shin_r);
	body_dest->push_back(PhysBod{ fromGeometry(left_shin_geom, col), left_shin_r });

	double foot_verticle_offset = shin_length + shin_radius * 0.75;
	double foot_height = 0.12 * scale;
	double foot_length = 0.5 * scale;
	double heel_length = 0.2 * scale;
	double foot_width = 0.35 * scale;
	mthz::Vec3 foot_offset(-foot_width / 2.0, -foot_height, -heel_length);
	phyz::ConvexUnionGeometry foot_geom = phyz::ConvexUnionGeometry::box(foot_offset, foot_width, foot_height, foot_length + heel_length);

	mthz::Vec3 right_foot_pos = right_shin_pos + mthz::Vec3(0, -foot_verticle_offset, 0);
	phyz::ConvexUnionGeometry right_foot_geom = foot_geom.getTranslated(right_foot_pos);
	phyz::RigidBody* right_foot_r = p->createRigidBody(right_foot_geom);
	out.push_back(right_foot_r);
	body_dest->push_back(PhysBod{ fromGeometry(right_foot_geom, col), right_foot_r });

	mthz::Vec3 left_foot_pos = left_shin_pos + mthz::Vec3(0, -foot_verticle_offset, 0);
	phyz::ConvexUnionGeometry left_foot_geom = foot_geom.getTranslated(left_foot_pos);
	phyz::RigidBody* left_foot_r = p->createRigidBody(left_foot_geom);
	out.push_back(left_foot_r);
	body_dest->push_back(PhysBod{ fromGeometry(left_foot_geom, col), left_foot_r });

	// arms

	double arm_length = 0.55 * scale;
	double arm_radius = 0.138 * scale;
	double arm_verticle_offset = 0.0962 * scale;
	double arm_horz_offset = 0.508 * scale;
	phyz::ConvexUnionGeometry arm_geometry = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(), arm_radius, arm_length);
	
	mthz::Vec3 right_arm_pos = pos + mthz::Vec3(-arm_horz_offset, arm_verticle_offset, 0);
	phyz::ConvexUnionGeometry right_arm_geom = arm_geometry.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1))).getTranslated(right_arm_pos);
	phyz::RigidBody* right_arm_r = p->createRigidBody(right_arm_geom);
	out.push_back(right_arm_r);
	body_dest->push_back(PhysBod{ fromGeometry(right_arm_geom, col), right_arm_r });

	mthz::Vec3 left_arm_pos = pos + mthz::Vec3(arm_horz_offset, arm_verticle_offset, 0);
	phyz::ConvexUnionGeometry left_arm_geom = arm_geometry.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1))).getTranslated(left_arm_pos);
	phyz::RigidBody* left_arm_r = p->createRigidBody(left_arm_geom);
	out.push_back(left_arm_r);
	body_dest->push_back(PhysBod{ fromGeometry(left_arm_geom, col), left_arm_r });

	double forearm_offset = arm_length + arm_radius;
	double forearm_length = 0.55 * scale;
	double forearm_radius = 0.0923 * scale;
	phyz::ConvexUnionGeometry forearm_geom = phyz::ConvexUnionGeometry::capsule(mthz::Vec3(), forearm_radius, forearm_length);

	mthz::Vec3 right_forearm_pos = right_arm_pos + mthz::Vec3(-forearm_offset, 0, 0);
	phyz::ConvexUnionGeometry right_forearm_geom = forearm_geom.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1))).getTranslated(right_forearm_pos);
	phyz::RigidBody* right_forearm_r = p->createRigidBody(right_forearm_geom);
	out.push_back(right_forearm_r);
	body_dest->push_back(PhysBod{ fromGeometry(right_forearm_geom, col), right_forearm_r });

	mthz::Vec3 left_forearm_pos = left_arm_pos + mthz::Vec3(forearm_offset, 0, 0);
	phyz::ConvexUnionGeometry left_forearm_geom = forearm_geom.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1))).getTranslated(left_forearm_pos);
	phyz::RigidBody* left_forearm_r = p->createRigidBody(left_forearm_geom);
	out.push_back(left_forearm_r);
	body_dest->push_back(PhysBod{ fromGeometry(left_forearm_geom, col), left_forearm_r });

	double hand_offset = forearm_length + forearm_radius / 2.0;
	double hand_height = 0.12 * scale;
	double hand_length = 0.4 * scale;
	double hand_width = 0.2 * scale;
	phyz::ConvexUnionGeometry hand_geom = phyz::ConvexUnionGeometry::box(mthz::Vec3(-hand_width/2.0, 0, -hand_height /2.0), hand_width, hand_length, hand_height);

	mthz::Vec3 right_hand_pos = right_forearm_pos + mthz::Vec3(-hand_offset, 0, 0);
	phyz::ConvexUnionGeometry right_hand_geom = hand_geom.getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1))).getTranslated(right_hand_pos);
	phyz::RigidBody* right_hand_r = p->createRigidBody(right_hand_geom);
	out.push_back(right_hand_r);
	body_dest->push_back(PhysBod{ fromGeometry(right_hand_geom, col), right_hand_r });

	mthz::Vec3 left_hand_pos = left_forearm_pos + mthz::Vec3(hand_offset, 0, 0);
	phyz::ConvexUnionGeometry left_hand_geom = hand_geom.getRotated(mthz::Quaternion(-PI / 2.0, mthz::Vec3(0, 0, 1))).getTranslated(left_hand_pos);
	phyz::RigidBody* left_hand_r = p->createRigidBody(left_hand_geom);
	out.push_back(left_hand_r);
	body_dest->push_back(PhysBod{ fromGeometry(left_hand_geom, col), left_hand_r });

	// neck + head
	mthz::Vec3 neck_pos = pos + mthz::Vec3(0, 0.245 * scale, 0);
	double neck_radius = 0.1 * scale;
	double neck_length = 0.25 * scale;
	phyz::ConvexUnionGeometry neck_geom = phyz::ConvexUnionGeometry::capsule(neck_pos, neck_radius, neck_length);
	phyz::RigidBody* neck_r = p->createRigidBody(neck_geom);
	out.push_back(neck_r);
	body_dest->push_back(PhysBod{ fromGeometry(neck_geom, col), neck_r });

	double head_radius = 0.27 * scale;
	mthz::Vec3 head_pos = neck_pos + mthz::Vec3(0, neck_length + head_radius * 0.5, head_radius * 0.33);
	phyz::ConvexUnionGeometry head_geom = phyz::ConvexUnionGeometry::sphere(head_pos, head_radius);
	phyz::RigidBody* head_r = p->createRigidBody(head_geom);
	out.push_back(head_r);
	body_dest->push_back(PhysBod{ fromGeometry(head_geom, col), head_r });

	// constraints

	// head to neck
	mthz::Vec3 neck_head_attach_pos = neck_pos + mthz::Vec3(0, neck_length, 0);
	p->addBallSocketConstraint(head_r, neck_r, neck_head_attach_pos);
	p->addConeLimitConstraint(head_r, neck_r, mthz::Vec3(0, -1, 0), PI / 4.0);
	p->addTwistLimitConstraint(head_r, neck_r, mthz::Vec3(0, -1, 0), -PI / 4.0, PI / 4.0);
	// neck to shoulder
	p->addBallSocketConstraint(neck_r, shoulder_r, neck_pos);
	p->addConeLimitConstraint(neck_r, shoulder_r, mthz::Vec3(0, -1, 0), PI / 4.0);
	p->addTwistLimitConstraint(neck_r, shoulder_r, mthz::Vec3(0, -1, 0), -PI / 4.0, PI / 4.0);

	//chest to shoulder
	p->addBallSocketConstraint(chest_r, shoulder_r, (chest_position + pos) / 2.0);
	p->addConeLimitConstraint(chest_r, shoulder_r, mthz::Vec3(0, -1, 0), PI / 8.0);
	p->addTwistLimitConstraint(chest_r, shoulder_r, mthz::Vec3(0, -1, 0), -PI / 8.0, PI / 8.0);

	//abdomen to chest
	p->addBallSocketConstraint(abdomen_r, chest_r, (abdomen_position + chest_position) / 2.0);
	p->addConeLimitConstraint(abdomen_r, chest_r, mthz::Vec3(0, -1, 0), PI / 8.0);
	p->addTwistLimitConstraint(abdomen_r, chest_r, mthz::Vec3(0, -1, 0), -PI / 8.0, PI / 8.0);

	//hip to abdomen
	p->addBallSocketConstraint(hip_r, abdomen_r, (hip_position + abdomen_position) / 2.0);
	p->addConeLimitConstraint(hip_r, abdomen_r, mthz::Vec3(0, -1, 0), PI / 8.0);
	p->addTwistLimitConstraint(hip_r, abdomen_r, mthz::Vec3(0, -1, 0), -PI / 8.0, PI / 8.0);

	//thighs to hip
	mthz::Vec3 hip_cone_direction = mthz::Vec3(0, -1, 1).normalize();
	double hip_out_twist_limit = PI / 2.0;
	double hip_in_twist_limit = PI / 6.0;
	p->addBallSocketConstraint(right_thigh_r, hip_r, right_thigh_pos);
	p->addConeLimitConstraint(right_thigh_r, hip_r, mthz::Vec3(0, -1, 0), hip_cone_direction, PI / 4);
	p->addTwistLimitConstraint(right_thigh_r, hip_r, mthz::Vec3(0, -1, 0), -hip_in_twist_limit, hip_out_twist_limit);
	p->addBallSocketConstraint(left_thigh_r, hip_r, left_thigh_pos);
	p->addConeLimitConstraint(left_thigh_r, hip_r, mthz::Vec3(0, -1, 0), hip_cone_direction, PI / 4);
	p->addTwistLimitConstraint(left_thigh_r, hip_r, mthz::Vec3(0, -1, 0), -hip_out_twist_limit, hip_in_twist_limit);

	// shins to thighs
	double max_knee_rotation = 0.75 * PI;
	double min_knee_rotation = 0;
	p->addMotorConstraint(
		p->addHingeConstraint(right_shin_r, right_thigh_r, right_shin_pos, mthz::Vec3(1, 0, 0)),
		min_knee_rotation, max_knee_rotation
	);
	p->addMotorConstraint(
		p->addHingeConstraint(left_shin_r, left_thigh_r, left_shin_pos, mthz::Vec3(1, 0, 0)),
		min_knee_rotation, max_knee_rotation
	);

	// feet to shins
	p->addBallSocketConstraint(right_foot_r, right_shin_r, right_foot_pos);
	p->addConeLimitConstraint(right_foot_r, right_shin_r, mthz::Vec3(0, -1, 0), PI / 8.0);
	p->addTwistLimitConstraint(right_foot_r, right_shin_r, mthz::Vec3(0, -1, 0), -PI / 8.0, PI / 8.0);
	p->addBallSocketConstraint(left_foot_r, left_shin_r, left_foot_pos);
	p->addConeLimitConstraint(left_foot_r, left_shin_r, mthz::Vec3(0, -1, 0), PI / 8.0);
	p->addTwistLimitConstraint(left_foot_r, left_shin_r, mthz::Vec3(0, -1, 0), -PI / 8.0, PI / 8.0);

	// arms to shoulders
	double cone_forward_angle = PI / 2.0;
	p->addBallSocketConstraint(right_arm_r, shoulder_r, right_arm_pos);
	mthz::Vec3 right_arm_cone_dir(-cos(cone_forward_angle), 0, sin(cone_forward_angle));
	p->addConeLimitConstraint(right_arm_r, shoulder_r, mthz::Vec3(-1, 0, 0), right_arm_cone_dir, cone_forward_angle);
	p->addTwistLimitConstraint(right_arm_r, shoulder_r, mthz::Vec3(-1, 0, 0), -PI / 2.0, PI / 2.0);
	p->addBallSocketConstraint(left_arm_r, shoulder_r, left_arm_pos);
	mthz::Vec3 left_arm_cone_dir(cos(cone_forward_angle), 0, sin(cone_forward_angle));
	p->addConeLimitConstraint(left_arm_r, shoulder_r, mthz::Vec3(1, 0, 0), left_arm_cone_dir, cone_forward_angle);
	p->addTwistLimitConstraint(left_arm_r, shoulder_r, mthz::Vec3(1, 0, 0), -PI / 2.0, PI / 2.0);

	// forearms to arms
	p->addMotorConstraint(
		p->addHingeConstraint(right_forearm_r, right_arm_r, right_forearm_pos, mthz::Vec3(0, 1, 0)),
		0, PI * 3.0 / 4.0
	);
	p->addMotorConstraint(
		p->addHingeConstraint(left_forearm_r, left_arm_r, left_forearm_pos, mthz::Vec3(0, 1, 0)),
		-PI * 3.0 / 4.0, 0
	);

	// hands to forearms
	p->addBallSocketConstraint(right_hand_r, right_forearm_r, right_hand_pos);
	p->addConeLimitConstraint(right_hand_r, right_forearm_r, mthz::Vec3(0, -1, 0), PI / 2.0);
	p->addTwistLimitConstraint(right_hand_r, right_forearm_r, mthz::Vec3(0, -1, 0), -PI / 2.0, PI / 2.0);
	p->addBallSocketConstraint(left_hand_r, left_forearm_r, left_hand_pos);
	p->addConeLimitConstraint(left_hand_r, left_forearm_r, mthz::Vec3(0, 1, 0), PI / 2.0);
	p->addTwistLimitConstraint(left_hand_r, left_forearm_r, mthz::Vec3(0, 1, 0), -PI / 2.0, PI / 2.0);
	return out;
}