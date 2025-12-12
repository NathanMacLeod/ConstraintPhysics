#pragma once
#include "RigidBody.h"
#include "ConstraintSolver.h"

namespace phyz {
	class PhysicsEngine;

	//todo: refactor this to just typedef a uint32_t. its weird to have both type and id value in this class. also very weird for contacts that use magic id instead
	struct ConstraintID {
		enum Type { INVALID, CONTACT, DISTANCE, BALL, HINGE, SLIDER, PISTON, MOTOR, SPRING, SLIDING_HINGE, WELD };

		ConstraintID() : uniqueID(-1), type(INVALID) {}
		ConstraintID(Type type, uint32_t id) : type(type), uniqueID(id) {}

		inline bool operator==(const ConstraintID c) const { return type == c.type && uniqueID == c.uniqueID; }
		
		inline Type getType() const { return type; }
		inline uint32_t getID() const { return uniqueID; }
	private:
		Type type;
		uint32_t uniqueID;
	};

	enum PoweredConstraitMode { OFF, CONST_TORQUE, CONST_FORCE, TARGET_VELOCITY, TARGET_POSITION };

	class PersistentConstraint {
	public:
		PersistentConstraint(ConstraintID id, RigidBody* b1, RigidBody* b2) : constraint_id(id), b1(b1), b2(b2) {}
		virtual ~PersistentConstraint() {};
		
		virtual bool isSolverConstraint() const = 0; // spring is the only exception here
		virtual void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) = 0;
		virtual void addSolverConstraints(std::vector<Constraint*>* accumulator) = 0;
		virtual bool isCFMConfigurable() const = 0;
		virtual void setCFM(CFM new_cfm) = 0;
		virtual bool isHolonomicConstraint() const = 0;
		virtual bool isPoweredConstraint() const = 0;
		virtual void setPoweredConstraintMode(PoweredConstraitMode new_mode) = 0;
		virtual void clearWarmstartData() = 0;

		inline ConstraintID getId() const { return constraint_id; }
		RigidBody* b1;
		RigidBody* b2;
		friend class PhysicsEngine;
	private:
		inline void setId(ConstraintID new_id) { constraint_id = new_id; }

		ConstraintID constraint_id;
	};

	struct Contact : public PersistentConstraint {
		Contact(RigidBody* b1, RigidBody* b2, mthz::Vec3 contact_local_b1, mthz::Vec3 contact_local_b2, mthz::Vec3 norm_local_b1, double pen_depth, double bounce, double pos_correct_hardness, double friction_coeff,
			    double cutoff_vel, MagicID magic, int memory_life)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::CONTACT, -1), b1, b2), contact_pos_local_b1(contact_local_b1), contact_pos_local_b2(contact_local_b2), normal_local_b1(norm_local_b1),
			  pen_depth(pen_depth), bounce(bounce), pos_correct_hardness(pos_correct_hardness), friction_coeff(friction_coeff), cutoff_vel(cutoff_vel), cfm(CFM{USE_GLOBAL}), magic(magic), memory_life(memory_life),
			  is_live_contact(true), contact(ContactConstraint()), friction(FrictionConstraint())
		{}

		ContactConstraint contact;
		FrictionConstraint friction;
		mthz::Vec3 contact_pos_local_b1;
		mthz::Vec3 contact_pos_local_b2;
		mthz::Vec3 normal_local_b1;
		double pen_depth;
		double bounce;
		double pos_correct_hardness;
		double friction_coeff;
		double cutoff_vel;
		CFM cfm;
		MagicID magic;
		int memory_life; //how many frames can be referenced for warm starting before is deleted
		bool is_live_contact;

		inline bool isExpired() const { return !b1->getAsleep() && !b2->getAsleep() && memory_life < 0; }

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { accumulator->push_back(&contact); accumulator->push_back(&friction); }
		inline void setCFM(CFM new_cfm) override { cfm = new_cfm; }
		inline bool isCFMConfigurable() const override { return true; }
		inline bool isHolonomicConstraint() const override { return false; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { contact.impulse = mthz::NVec<1>{ 0.0 }; friction.impulse = mthz::NVec<2>{ 0.0 }; };

	};

	struct Distance : public PersistentConstraint {
		Distance(RigidBody* b1, RigidBody* b2, double target_distance, RigidBody::PKey b1_key, RigidBody::PKey b2_key, double pos_correct_strength, uint32_t id_value)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::DISTANCE, id_value), b1, b2), target_distance(target_distance), b1_point_key(b1_key), b2_point_key(b2_key),
			  cfm(CFM{USE_GLOBAL}), pos_correct_hardness(pos_correct_strength), constraint(DistanceConstraint())
		{
			//init b1, b2 ptr on constraint as holonomic logic needs it
			constraint.a = b1;
			constraint.b = b2;
		}

		DistanceConstraint constraint;
		double target_distance;
		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		CFM cfm;
		double pos_correct_hardness;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { accumulator->push_back(&constraint); }
		inline void setCFM(CFM new_cfm) override { cfm = new_cfm; }
		inline bool isCFMConfigurable() const override { return true; }
		inline bool isHolonomicConstraint() const override { return true; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { constraint.impulse = mthz::NVec<1>{ 0.0 }; };
	};

	struct BallSocket : public PersistentConstraint {
		BallSocket(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_key, RigidBody::PKey b2_key, double pos_correct_strength, uint32_t id_value)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::BALL, id_value), b1, b2), b1_point_key(b1_key), b2_point_key(b2_key),
			cfm(CFM{ USE_GLOBAL }), pos_correct_hardness(pos_correct_strength), constraint(BallSocketConstraint())
		{
			//init b1, b2 ptr on constraint as holonomic logic needs it
			constraint.a = b1;
			constraint.b = b2;
		}

		BallSocketConstraint constraint;
		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		CFM cfm;
		double pos_correct_hardness;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { accumulator->push_back(&constraint); }
		inline void setCFM(CFM new_cfm) override { cfm = new_cfm; }
		inline bool isCFMConfigurable() const override { return true; }
		inline bool isHolonomicConstraint() const override { return true; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { constraint.impulse = mthz::NVec<3>{ 0.0 }; };
	};

	// todo: piston and motor used to be "child constraints" of other constraints. want to seperate them entirely for simplicity
	struct Piston : public PersistentConstraint {
		Piston(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_point_key, RigidBody::PKey b2_point_key, mthz::Vec3 b1_slide_axis_body_space, double min_pos, double max_pos, uint32_t id_value);
		double calculatePosition(mthz::Vec3 b1_pos, mthz::Vec3 b2_pos, mthz::Vec3 slide_axis);
		double getPistonTargetVelocityValue(double piston_pos, mthz::Vec3 slide_axis, double step_time);
		void writePrevVel(mthz::Vec3 slide_axis);
		bool slideLimitExceeded(double piston_pos);
		bool pistonIsActive();

		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		mthz::Vec3 b1_slide_axis_body_space;
		SlideLimitConstraint slide_limit;
		PistonConstraint piston_constraint;
		PoweredConstraitMode mode;
		CFM cfm;
		double pos_correct_hardness;
		double piston_position;
		bool slide_limit_exceeded;
		double min_slide_limit;
		double max_slide_limit;
		double max_force;
		double target_velocity;
		double target_position;
		double prev_velocity;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		void addSolverConstraints(std::vector<Constraint*>* accumulator) override;
		inline void setCFM(CFM new_cfm) override { assert(false); }
		inline bool isCFMConfigurable() const override { return false; }
		inline bool isHolonomicConstraint() const override { return false; }
		inline bool isPoweredConstraint() const override { return true; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { mode = new_mode; }
		inline void clearWarmstartData() override { piston_constraint.impulse = mthz::NVec<1>{ 0.0 }; slide_limit.impulse = mthz::NVec<1>{ 0.0 }; };
	};

	struct Motor : public PersistentConstraint {
		Motor(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double min_angle, double max_angle, uint32_t id_value);
		double calculatePosition(mthz::Vec3 rot_axis, mthz::Vec3 ang_vel_b1, mthz::Vec3 ang_vel_b2, double timestep);
		double getConstraintTargetVelocityValue(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double step_time);
		void writePrevVel(mthz::Vec3 rot_axis, mthz::Vec3 ang_vel_b1, mthz::Vec3 ang_vel_b2);
		bool constraintIsActive();

		mthz::Vec3 b1_rot_axis_body_space;
		mthz::Vec3 b1_u_axis_reference;
		mthz::Vec3 b1_w_axis_reference;
		mthz::Vec3 b2_rot_comparison_axis;
		MotorConstraint motor_constraint;
		PoweredConstraitMode mode;
		CFM cfm;
		double rot_correct_hardness;
		double motor_angular_position;
		double min_motor_position;
		double max_motor_position;
		double max_torque;
		double target_velocity;
		double target_position;
		double prev_velocity;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		void addSolverConstraints(std::vector<Constraint*>* accumulator) override;
		inline void setCFM(CFM new_cfm) override { assert(false); }
		inline bool isCFMConfigurable() const override { return false; }
		inline bool isHolonomicConstraint() const override { return false; }
		inline bool isPoweredConstraint() const override { return true; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { mode = new_mode; }
		inline void clearWarmstartData() override { motor_constraint.impulse = mthz::NVec<1>{ 0.0 }; };
	};

	struct Hinge : public PersistentConstraint {
		Hinge(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_key, RigidBody::PKey b2_key, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local,
			  double pos_correct_strength, double rot_correct_strength, uint32_t id_value)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::HINGE, id_value), b1, b2), b1_point_key(b1_key), b2_point_key(b2_key),
			b1_rot_axis_body_space(b1_rot_axis_local), b2_rot_axis_body_space(b2_rot_axis_local), cfm(CFM{ USE_GLOBAL }), pos_correct_hardness(pos_correct_strength),
			rot_correct_hardness(rot_correct_strength), constraint(HingeConstraint())
		{
			//init b1, b2 ptr on constraint as holonomic logic needs it
			constraint.a = b1;
			constraint.b = b2;
		}

		HingeConstraint constraint;
		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		mthz::Vec3 b1_rot_axis_body_space;
		mthz::Vec3 b2_rot_axis_body_space;
		CFM cfm;
		double pos_correct_hardness;
		double rot_correct_hardness;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { accumulator->push_back(&constraint); }
		inline void setCFM(CFM new_cfm) override { cfm = new_cfm; }
		inline bool isCFMConfigurable() const override { return true; }
		inline bool isHolonomicConstraint() const override { return true; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { constraint.impulse = mthz::NVec<5>{ 0.0 }; };
	};

	struct Slider : public PersistentConstraint {
		Slider(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_key, RigidBody::PKey b2_key, mthz::Vec3 b1_slide_axis_local, mthz::Vec3 b2_slide_axis_local,
			double pos_correct_strength, double rot_correct_strength, uint32_t id_value)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::SLIDER, id_value), b1, b2), b1_point_key(b1_key), b2_point_key(b2_key),
			b1_slide_axis_body_space(b1_slide_axis_local), b2_slide_axis_body_space(b2_slide_axis_local), cfm(CFM{ USE_GLOBAL }), pos_correct_hardness(pos_correct_strength),
			rot_correct_hardness(rot_correct_strength), constraint(SliderConstraint())
		{
			//init b1, b2 ptr on constraint as holonomic logic needs it
			constraint.a = b1;
			constraint.b = b2;
		}

		SliderConstraint constraint;
		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		mthz::Vec3 b1_slide_axis_body_space;
		mthz::Vec3 b2_slide_axis_body_space;
		CFM cfm;
		double pos_correct_hardness;
		double rot_correct_hardness;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { accumulator->push_back(&constraint); }
		inline void setCFM(CFM new_cfm) override { cfm = new_cfm; }
		inline bool isCFMConfigurable() const override { return true; }
		inline bool isHolonomicConstraint() const override { return true; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { constraint.impulse = mthz::NVec<5>{ 0.0 }; };
	};

	struct SlidingHinge : public PersistentConstraint {
		SlidingHinge(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_key, RigidBody::PKey b2_key, mthz::Vec3 b1_slide_axis_local, mthz::Vec3 b2_slide_axis_local,
			double pos_correct_strength, double rot_correct_strength, uint32_t id_value)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::SLIDING_HINGE, id_value), b1, b2), b1_point_key(b1_key), b2_point_key(b2_key),
			b1_slide_axis_body_space(b1_slide_axis_local), b2_slide_axis_body_space(b2_slide_axis_local), cfm(CFM{ USE_GLOBAL }), pos_correct_hardness(pos_correct_strength),
			rot_correct_hardness(rot_correct_strength), constraint(SlidingHingeConstraint())
		{
			//init b1, b2 ptr on constraint as holonomic logic needs it
			constraint.a = b1;
			constraint.b = b2;
		}

		SlidingHingeConstraint constraint;
		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		mthz::Vec3 b1_slide_axis_body_space;
		mthz::Vec3 b2_slide_axis_body_space;
		CFM cfm;
		double pos_correct_hardness;
		double rot_correct_hardness;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { accumulator->push_back(&constraint); }
		inline void setCFM(CFM new_cfm) override { cfm = new_cfm; }
		inline bool isCFMConfigurable() const override { return true; }
		inline bool isHolonomicConstraint() const override { return true; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { constraint.impulse = mthz::NVec<4>{ 0.0 }; };
	};

	struct Weld : public PersistentConstraint {
		Weld(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_key, RigidBody::PKey b2_key, double pos_correct_strength, double rot_correct_strength, uint32_t id_value)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::WELD, id_value), b1, b2), b1_point_key(b1_key), b2_point_key(b2_key),
			cfm(CFM{ USE_GLOBAL }), pos_correct_hardness(pos_correct_strength), rot_correct_hardness(rot_correct_strength), constraint(WeldConstraint())
		{
			//init b1, b2 ptr on constraint as holonomic logic needs it
			constraint.a = b1;
			constraint.b = b2;
		}

		WeldConstraint constraint;
		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		CFM cfm;
		double pos_correct_hardness;
		double rot_correct_hardness;

		inline bool isSolverConstraint() const override { return true; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { accumulator->push_back(&constraint); }
		inline void setCFM(CFM new_cfm) override { cfm = new_cfm; }
		inline bool isCFMConfigurable() const override { return true; }
		inline bool isHolonomicConstraint() const override { return true; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { constraint.impulse = mthz::NVec<6>{ 0.0 }; };
	};

	//todo: soft constraint instead, I think that is as simple as a distance constraint with high CFM?
	struct Spring : public PersistentConstraint {
		Spring(RigidBody* b1, RigidBody* b2, RigidBody::PKey b1_key, RigidBody::PKey b2_key, double stiffness, double damping, double resting_length, uint32_t id_value)
			: PersistentConstraint(ConstraintID(ConstraintID::Type::SPRING, id_value), b1, b2), b1_point_key(b1_key), b2_point_key(b2_key), stiffness(stiffness),
			damping(damping), resting_length(resting_length)
		{}

		RigidBody::PKey b1_point_key;
		RigidBody::PKey b2_point_key;
		double stiffness;
		double damping;
		double resting_length;

		inline bool isSolverConstraint() const override { return false; }
		void updateSolverConstraints(bool warm_start_disabled, double warm_start_coefficient, double step_time, double global_cfm, bool is_in_holonomic_system) override;
		inline void addSolverConstraints(std::vector<Constraint*>* accumulator) override { assert(false); }
		inline void setCFM(CFM new_cfm) override { assert(false); }
		inline bool isCFMConfigurable() const override { return false; }
		inline bool isHolonomicConstraint() const override { return true; }
		inline bool isPoweredConstraint() const override { return false; }
		inline void setPoweredConstraintMode(PoweredConstraitMode new_mode) override { assert(false); }
		inline void clearWarmstartData() override { /*nothing to do*/ };
	};
}