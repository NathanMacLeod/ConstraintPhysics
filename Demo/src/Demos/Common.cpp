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