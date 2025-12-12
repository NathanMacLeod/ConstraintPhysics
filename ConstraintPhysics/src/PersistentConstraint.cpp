#include "PersistentConstraint.h"

namespace phyz {

	Motor::Motor(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double min_angle, double max_angle, uint32_t id_value)
		: PersistentConstraint(ConstraintID(ConstraintID::Type::MOTOR, id_value), b1, b2), b1_rot_axis_body_space(b1_rot_axis_local), target_velocity(0), max_torque(0), target_position(0),
		  mode(OFF), min_motor_position(min_angle), max_motor_position(max_angle)
	{
		mthz::Vec3 u1, w1;
		b1_rot_axis_local.getPerpendicularBasis(&u1, &w1);
		mthz::Vec3 u2, tmp;
		b2_rot_axis_local.getPerpendicularBasis(&u2, &tmp);

		b1_u_axis_reference = u1;
		b1_w_axis_reference = w1;
		b2_rot_comparison_axis = u2;
		mthz::Vec3 rot_ref_axis_u = b1->getOrientation().applyRotation(b1_u_axis_reference);
		mthz::Vec3 rot_ref_axis_w = b1->getOrientation().applyRotation(b1_w_axis_reference);
		mthz::Vec3 rot_compare_axis = b2->getOrientation().applyRotation(b2_rot_comparison_axis);
		mthz::Vec3 b1_hinge_axis = b1->getOrientation().applyRotation(b1_rot_axis_local);

		motor_angular_position = 0; //normalizing for the calculatePosition calculation;
		motor_angular_position = calculatePosition(b1_hinge_axis, mthz::Vec3(), mthz::Vec3(), 0);
	}

	void Motor::addSolverConstraints(std::vector<Constraint*>* accumulator) {
		if (constraintIsActive()) { accumulator->push_back(&motor_constraint); }
	}

	double Motor::calculatePosition(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double step_time) {
		mthz::Vec3 ref_axis_u = b1->getOrientation().applyRotation(b1_u_axis_reference);
		mthz::Vec3 ref_axis_w = b1->getOrientation().applyRotation(b1_w_axis_reference);
		mthz::Vec3 compare_axis = b2->getOrientation().applyRotation(b2_rot_comparison_axis);

		//estimate based on velocity
		double relative_angular_velocity = rot_axis.dot(b1_ang_vel) - rot_axis.dot(b2_ang_vel);
		double predicted_new_angle = motor_angular_position + relative_angular_velocity * step_time;

		//use concrete orientations to eliminate drifting error
		if (abs(compare_axis.dot(rot_axis)) > 0.999) {
			//doesnt work if compairson is perp to rotation plane. just give up.
			return predicted_new_angle;
		}
		else {
			//normalize compare axis to rotation plane.
			mthz::Vec3 compare_norm = (compare_axis - rot_axis * rot_axis.dot(compare_axis)).normalize();

			//numerical accuracy issue breaks acos
			double dot_v = compare_norm.dot(ref_axis_u);
			if (dot_v > 1.0) dot_v = 1.0;
			if (dot_v < -1.0) dot_v = -1.0;

			//relative angle is flipped such that angles are clockwise. this is because i did things weird and this easiest way to fix it
			double relative_angle = 2 * PI - acos(dot_v);
			if (compare_norm.dot(ref_axis_w) < 0) relative_angle = 2 * PI - relative_angle;

			//pred_angle = n * 2PI + r
			double r = fmod(predicted_new_angle, 2 * PI);
			double n2PI = predicted_new_angle - r;

			//this is to handle edge cases where we are crossing the threshold between n2PI and (n+1)2PI, or n2PI to (n-1)2PI
			double corrected_angle_candidates[] = { n2PI + relative_angle - 2 * PI, n2PI + relative_angle, n2PI + relative_angle + 2 * PI };
			double closest = std::numeric_limits<double>::infinity();
			for (double c : corrected_angle_candidates) {
				if (abs(predicted_new_angle - c) < abs(predicted_new_angle - closest))
					closest = c;
			}

			return closest;
		}
	}

	void Motor::writePrevVel(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel) {
		prev_velocity = rot_axis.dot(b1_ang_vel) - rot_axis.dot(b2_ang_vel);
	}

	double Motor::getConstraintTargetVelocityValue(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double step_time) {
		switch (mode) {
		case OFF:
			return 0;
		case CONST_TORQUE:
			return (max_torque > 0) ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
		case TARGET_VELOCITY:
			return target_velocity;
		case TARGET_POSITION:
			double ang_diff = motor_angular_position - target_position;
			double target_dir = (ang_diff > 0) ? -1 : 1;
			//account for fixed/unfixed potentially changing after constraint created
			double vel_diff_respect_time = abs((rot_axis.dot(b1_ang_vel) - rot_axis.dot(b2_ang_vel)) - prev_velocity);

			const double EPS = 0.0000001;

			double inertia;
			if (vel_diff_respect_time < EPS || abs(motor_constraint.impulse.v[0] < EPS)) {
				//avoid dividing by 0, just try a different way (not the most accurate which is why is not used for general method)
				inertia = 1.0 / (rot_axis.dot(b1->getInvTensor() * rot_axis) + rot_axis.dot(b2->getInvTensor() * rot_axis));
			}
			else {
				inertia = abs(motor_constraint.impulse.v[0]) / vel_diff_respect_time;
			}

			double returnval = target_dir * sqrt(2 * abs(ang_diff) * max_torque / inertia);
			assert(!std::isinf(returnval));
			return returnval;
		}

		assert(false);
		return -1.0;
	}

	bool Motor::constraintIsActive() {
		return (mode != OFF && max_torque != 0) || motor_angular_position < min_motor_position || motor_angular_position > max_motor_position;
	}

	Piston::Piston(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_point_key, RigidBody::PKey b2_point_key, mthz::Vec3 b1_slide_axis_body_space, double min_pos, double max_pos, uint32_t id_value)
		: PersistentConstraint(ConstraintID(ConstraintID::Type::PISTON, id_value), b1, b2), b1_point_key(b1_point_key), b2_point_key(b2_point_key), b1_slide_axis_body_space(b1_slide_axis_body_space),
		  target_velocity(0), max_force(0), target_position(0), mode(OFF), min_slide_limit(min_pos), max_slide_limit(max_pos), slide_limit_exceeded(false), piston_constraint(PistonConstraint()), slide_limit(SlideLimitConstraint())
	{
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b1->getTrackedP(b2_point_key);
		mthz::Vec3 slide_axis = b1->getOrientation().applyRotation(b1_slide_axis_body_space);
		piston_position = calculatePosition(b1_pos, b2_pos, slide_axis);
	}

	void Piston::addSolverConstraints(std::vector<Constraint*>* accumulator) {
		if (slide_limit_exceeded) { accumulator->push_back(&slide_limit); }
		if (pistonIsActive()) { accumulator->push_back(&piston_constraint); }
	}

	double Piston::calculatePosition(mthz::Vec3 b1_pos, mthz::Vec3 b2_pos, mthz::Vec3 slide_axis) {
		return b2_pos.dot(slide_axis) - b1_pos.dot(slide_axis);
	}
	double Piston::getPistonTargetVelocityValue(double piston_pos, mthz::Vec3 slide_axis, double step_time) {
		switch (mode) {
		case OFF:
			return 0;
		case CONST_FORCE:
			return (max_force > 0) ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
		case TARGET_VELOCITY:
			return target_velocity;
		case TARGET_POSITION:
			double ang_diff = piston_pos - target_position;
			double target_dir = (ang_diff > 0) ? -1 : 1;
			//account for fixed/unfixed potentially changing after constraint created
			double vel_diff_respect_time = abs((slide_axis.dot(b1->getVel()) - slide_axis.dot(b2->getVel())) - prev_velocity);

			const double EPS = 0.0000001;

			double inertia;
			if (vel_diff_respect_time < EPS || abs(piston_constraint.impulse.v[0] < EPS)) {
				//avoid dividing by 0, just try a different way (not the most accurate which is why is not used for general method)
				inertia = 1.0 / (b1->getInvMass() + b2->getInvMass());
			}
			else {
				inertia = abs(piston_constraint.impulse.v[0]) / vel_diff_respect_time;
			}

			double returnval = target_dir * sqrt(2 * abs(ang_diff) * max_force / inertia);
			assert(!std::isinf(returnval));
			return returnval;
		}

		assert(false);
		return -1.0;
	}

	void Piston::writePrevVel(mthz::Vec3 slide_axis) {
		prev_velocity = b2->getVel().dot(slide_axis) - b1->getVel().dot(slide_axis);
	}

	bool Piston::slideLimitExceeded(double piston_pos) {
		return piston_pos < min_slide_limit || piston_pos > max_slide_limit;
	}

	bool Piston::pistonIsActive() {
		return mode != OFF && max_force != 0;
	}

	static double posCorrectCoeff(double pos_correct_strength, double step_time) { return std::min<double>(pos_correct_strength * step_time, 1.0 / step_time); }

	void Contact::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getOrientation().applyRotation(contact_pos_local_b1) + b1->getCOM();
		mthz::Vec3 b2_pos = b2->getOrientation().applyRotation(contact_pos_local_b2) + b2->getCOM();
		mthz::Vec3 norm = b1->getOrientation().applyRotation(normal_local_b1);

		double adjusted_depth = (b2_pos - b1_pos).dot(norm) + pen_depth;
		//todo
		mthz::NVec<1> contact_impulse = warm_start_disabled ? mthz::NVec<1>{ 0.0 } : warm_start_coefficient * contact.impulse;
		mthz::NVec<2> friction_impulse = warm_start_disabled ? mthz::NVec<2> { 0.0 } : warm_start_coefficient * friction.impulse;

		double pos_correct_coeff = posCorrectCoeff(pos_correct_hardness, step_time);
		double normal_impulse_limit = std::numeric_limits<double>::infinity();
		contact = ContactConstraint(b1, b2, norm, b1_pos, bounce, adjusted_depth, pos_correct_coeff, cfm.getCFMValue(global_cfm), contact_impulse, cutoff_vel);
		friction = FrictionConstraint(b1, b2, norm, b1_pos, friction_coeff, &contact, cfm.getCFMValue(global_cfm), friction_impulse, friction.u, friction.w, normal_impulse_limit);
	}

	void Distance::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b2->getTrackedP(b2_point_key);
		mthz::NVec<1> starting_impulse = warm_start_disabled ? mthz::NVec<1>{ 0.0} : warm_start_coefficient * constraint.impulse;
		double pos_correct_coeff = posCorrectCoeff(pos_correct_hardness, step_time);
		constraint = DistanceConstraint(b1, b2, b1_pos, b2_pos, target_distance, pos_correct_coeff, cfm.getCFMValue(global_cfm), is_in_holonomic_system, starting_impulse);
	}

	void BallSocket::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b2->getTrackedP(b2_point_key);
		mthz::NVec<3> starting_impulse = warm_start_disabled ? mthz::NVec<3>{ 0.0} : warm_start_coefficient * constraint.impulse;
		double pos_correct_coeff = posCorrectCoeff(pos_correct_hardness, step_time);
		constraint = BallSocketConstraint(b1, b2, b1_pos, b2_pos, pos_correct_coeff, cfm.getCFMValue(global_cfm), is_in_holonomic_system, starting_impulse);
	}

	void Hinge::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b2->getTrackedP(b2_point_key);
		mthz::Vec3 b1_hinge_axis = b1->getOrientation().applyRotation(b1_rot_axis_body_space);
		mthz::Vec3 b2_hinge_axis = b2->getOrientation().applyRotation(b2_rot_axis_body_space);

		double pos_correct_coeff = posCorrectCoeff(pos_correct_hardness, step_time);
		double rot_correct_coeff = posCorrectCoeff(rot_correct_hardness, step_time);

		mthz::NVec<5> starting_impulse = warm_start_disabled ? mthz::NVec<5>{ 0.0} : warm_start_coefficient * constraint.impulse;
		constraint = HingeConstraint(b1, b2, b1_pos, b2_pos, b1_hinge_axis, b2_hinge_axis, pos_correct_coeff, rot_correct_coeff, cfm.getCFMValue(global_cfm), is_in_holonomic_system, starting_impulse, constraint.u, constraint.w);
	}

	void Slider::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b2->getTrackedP(b2_point_key);
		mthz::Vec3 b1_slide_axis = b1->getOrientation().applyRotation(b1_slide_axis_body_space);
		mthz::Vec3 b2_slide_axis = b2->getOrientation().applyRotation(b2_slide_axis_body_space);

		double pos_correct_coeff = posCorrectCoeff(pos_correct_hardness, step_time);
		double rot_correct_coeff = posCorrectCoeff(rot_correct_hardness, step_time);

		mthz::NVec<5> starting_impulse = warm_start_disabled ? mthz::NVec<5>{ 0.0} : warm_start_coefficient * constraint.impulse;
		constraint = SliderConstraint(b1, b2, b1_pos, b2_pos, b1_slide_axis, pos_correct_coeff, pos_correct_coeff, cfm.getCFMValue(global_cfm), is_in_holonomic_system, starting_impulse, constraint.u, constraint.w);
	}

	void Piston::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b2->getTrackedP(b2_point_key);
		mthz::Vec3 b1_slide_axis = b1->getOrientation().applyRotation(b1_slide_axis_body_space);

		piston_position = calculatePosition(b1_pos, b2_pos, b1_slide_axis);
		slide_limit_exceeded = slideLimitExceeded(piston_position);

		if (slide_limit_exceeded) {
			mthz::NVec<1> limit_starting_impulse = warm_start_disabled ? mthz::NVec<1>{ 0.0} : warm_start_coefficient * slide_limit.impulse;
			slide_limit = SlideLimitConstraint(b1, b2, b1_slide_axis, piston_position, max_slide_limit, min_slide_limit, posCorrectCoeff(pos_correct_hardness, step_time), cfm.getCFMValue(global_cfm), limit_starting_impulse);
		}

		if (pistonIsActive()) {
			mthz::NVec<1> piston_starting_impulse = warm_start_disabled ? mthz::NVec<1>{ 0.0} : warm_start_coefficient * piston_constraint.impulse;
			double piston_target_velocity = getPistonTargetVelocityValue(piston_position, b1_slide_axis, step_time);
			piston_constraint = PistonConstraint(b1, b2, b1_slide_axis, piston_target_velocity, max_force * step_time, cfm.getCFMValue(global_cfm), piston_starting_impulse);
		}
	}

	void Motor::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_hinge_axis = b1->getOrientation().applyRotation(b1_rot_axis_body_space);
		motor_angular_position = calculatePosition(b1_hinge_axis, b1->getAngVel(), b2->getAngVel(), step_time);
		mthz::NVec<1> motor_starting_impulse = warm_start_disabled ? mthz::NVec<1>{ 0.0} : warm_start_coefficient * motor_constraint.impulse;
		if (constraintIsActive()) {
			motor_constraint = MotorConstraint(b1, b2, b1_hinge_axis, warm_start_coefficient * getConstraintTargetVelocityValue(b1_hinge_axis, b1->getAngVel(), b2->getAngVel(),
				step_time), abs(max_torque * step_time), motor_angular_position, min_motor_position, max_motor_position, posCorrectCoeff(rot_correct_hardness, step_time), cfm.getCFMValue(global_cfm), motor_starting_impulse);
		}
		writePrevVel(b1_hinge_axis, b1->getAngVel(), b2->getAngVel());
	}

	void SlidingHinge::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b2->getTrackedP(b2_point_key);
		mthz::Vec3 b1_slide_axis = b1->getOrientation().applyRotation(b1_slide_axis_body_space);
		mthz::Vec3 b2_slide_axis = b2->getOrientation().applyRotation(b2_slide_axis_body_space);

		double pos_correct_coeff = posCorrectCoeff(pos_correct_hardness, step_time);
		double rot_correct_coeff = posCorrectCoeff(rot_correct_hardness, step_time);

		mthz::NVec<4> starting_impulse = warm_start_disabled ? mthz::NVec<4>{ 0.0} : warm_start_coefficient * constraint.impulse;
		constraint = SlidingHingeConstraint(b1, b2, b1_pos, b2_pos, b1_slide_axis, b2_slide_axis, pos_correct_coeff, rot_correct_coeff, cfm.getCFMValue(global_cfm), is_in_holonomic_system, starting_impulse, constraint.u, constraint.w);
	}

	void Weld::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		mthz::Vec3 b1_pos = b1->getTrackedP(b1_point_key);
		mthz::Vec3 b2_pos = b2->getTrackedP(b2_point_key);

		double pos_correct_coeff = posCorrectCoeff(pos_correct_hardness, step_time);
		double rot_correct_coeff = posCorrectCoeff(rot_correct_hardness, step_time);

		mthz::NVec<6> starting_impulse = warm_start_disabled ? mthz::NVec<6>{ 0.0} : warm_start_coefficient * constraint.impulse;
		constraint = WeldConstraint(b1, b2, b1_pos, b2_pos, pos_correct_coeff, rot_correct_coeff, cfm.getCFMValue(global_cfm), is_in_holonomic_system, constraint.impulse);
	}

	void Spring::updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) {
		//do nothing
	}

}