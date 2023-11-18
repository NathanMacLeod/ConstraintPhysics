#pragma once
#include "RigidBody.h"
#include "HolonomicBlockSolver.h"
#include "../../Math/src/NVec.h"
#include "../../Math/src/NMat.h"

namespace phyz {

	class Constraint;
	class PhysicsEngine;
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, const std::set<HolonomicSystem*>& holonomic_systems, int n_itr_vel, int n_itr_pos);

	class Constraint {
	public:
		Constraint() : a(nullptr), b(nullptr), a_velocity_change(nullptr), a_psuedo_velocity_change(nullptr), b_velocity_change(nullptr), b_psuedo_velocity_change(nullptr), is_in_holonomic_system(false) {}
		Constraint(RigidBody* a, RigidBody* b) : a(a), b(b), a_velocity_change(nullptr), a_psuedo_velocity_change(nullptr), b_velocity_change(nullptr), b_psuedo_velocity_change(nullptr), is_in_holonomic_system(false) {}
		virtual ~Constraint() {}

		RigidBody* a;
		RigidBody* b;
		bool is_in_holonomic_system;
		mthz::NVec<6>* a_velocity_change;
		mthz::NVec<6>* a_psuedo_velocity_change;
		mthz::NVec<6>* b_velocity_change;
		mthz::NVec<6>* b_psuedo_velocity_change;

		virtual int getDegree() = 0;
		virtual bool isInequalityConstraint() = 0;
		virtual bool needsPosCorrect() = 0;
		virtual bool constraintWarmStarted() = 0;
		virtual void applyWarmStartVelocityChange() = 0;
	protected:
		//these are pretty inefficient to multiply with, but nice for sanity checking
		mthz::NMat<6, 6> aInvMass();
		mthz::NMat<6, 6> bInvMass();
	};

	template<int n>
	class DegreedConstraint : public Constraint {
	public:
		DegreedConstraint() : impulse(mthz::NVec<n>{0.0}) {}
		DegreedConstraint(RigidBody* a, RigidBody* b, mthz::NVec<n> impulse) : Constraint(a, b), impulse(impulse), psuedo_impulse(mthz::NVec<n>{0.0}) {}

		virtual mthz::NVec<n> projectValidImpulse(mthz::NVec<n> impulse) { return impulse; }
		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void applyWarmStartVelocityChange() override {
			computeAndApplyVelocityChange(impulse, a_velocity_change, b_velocity_change);
		}
		//for some constraints this can be computed much more effeciently than generalized matrix multiplciation, so providing override since this is performance impacting code
		virtual void computeAndApplyVelocityChange(mthz::NVec<n> impulse, mthz::NVec<6>* va, mthz::NVec<6>* vb) {
			*va += impulse_to_a_velocity * impulse;
			*vb += impulse_to_b_velocity * impulse;
		}

		mthz::NVec<n> getConstraintValue(const mthz::NVec<6>& va, const mthz::NVec<6>& vb) {
			return a_jacobian * va + b_jacobian * vb;
		}

		mthz::NVec<n> impulse;
		mthz::NVec<n> psuedo_impulse;
		mthz::NVec<n> target_val;
		mthz::NVec<n> psuedo_target_val;
		mthz::NMat<n, 6> a_jacobian;
		mthz::NMat<n, 6> b_jacobian;
		mthz::NMat<6, n> impulse_to_a_velocity;
		mthz::NMat<6, n> impulse_to_b_velocity;
		mthz::NMat<n, n> impulse_to_value;
		mthz::NMat<n, n> impulse_to_value_inverse;
 	};

	class ContactConstraint : public DegreedConstraint<1> {
	public:
		ContactConstraint() {}
		ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse=mthz::NVec<1>{ 0.0 }, double cutoff_vel=0);
		
		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		mthz::NVec<1> projectValidImpulse(mthz::NVec<1> impulse) override;
		inline bool needsPosCorrect() override { return true; }

	private:
		
		mthz::Vec3 norm;
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 rotDirA;
		mthz::Vec3 rotDirB;
	};

	class FrictionConstraint : public DegreedConstraint<2> {
	public:
		FrictionConstraint() : normal_impulse(nullptr) {}
		FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double coeff_friction, ContactConstraint* normal, double constraint_force_mixing, mthz::NVec<2> warm_start_impulse = mthz::NVec<2>{ 0.0, 0.0 }, mthz::Vec3 source_u = mthz::Vec3(), mthz::Vec3 source_w = mthz::Vec3(), double normal_impulse_limit = std::numeric_limits<double>::infinity());

		inline int getDegree() override { return 2; }
		inline bool isInequalityConstraint() override { return true; }
		mthz::NVec<2> projectValidImpulse(mthz::NVec<2> impulse) override;
		inline bool needsPosCorrect() override { return false; }

		bool getStaticReady() { return static_ready; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::NMat<3, 2> rotDirA;
		mthz::NMat<3, 2> rotDirB;
		double coeff_friction;
		mthz::NVec<1>* normal_impulse;
		double normal_impulse_limit;
		bool static_ready;
	};

	class BallSocketConstraint : public DegreedConstraint<3> {
	public:
		BallSocketConstraint() {}
		BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_pos_a, mthz::Vec3 socket_pos_b, double pos_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<3> warm_start_impulse=mthz::NVec<3>{ 0.0 });
		
		inline int getDegree() override { return 3; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Mat3 rotDirA;
		mthz::Mat3 rotDirB;
	};

	class HingeConstraint : public DegreedConstraint<5> {
	public:
		HingeConstraint() {}
		HingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<5> warm_start_impulse=mthz::NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u=mthz::Vec3(), mthz::Vec3 source_w=mthz::Vec3());

		inline int getDegree() override { return 5; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 n;
		mthz::NMat<3, 5> rotDirA;
		mthz::NMat<3, 5> rotDirB;
	};

	class MotorConstraint : public DegreedConstraint<1> {
	public:
		MotorConstraint() {}
		MotorConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 motor_axis, double target_velocity, double max_torque_impulse, double current_angle, double min_angle, double max_angle, double rot_correct_hardness, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse = mthz::NVec<1>{ 0.0 });

		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		mthz::NVec<1> projectValidImpulse(mthz::NVec<1> impulse) override;
		inline bool needsPosCorrect() override { return false; }

	private:
		enum LimitStatus { NOT_EXCEEDED = 0, BELOW_MIN, ABOVE_MAX };

		LimitStatus rot_limit_status;
		double max_torque_impulse;
		mthz::Vec3 motor_axis;
		mthz::Vec3 rotDirA;
		mthz::Vec3 rotDirB;
	};

	class SliderConstraint : public DegreedConstraint<5> {
	public:
		SliderConstraint() {}
		SliderConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slider_point_a, mthz::Vec3 slider_point_b, mthz::Vec3 slider_axis_a, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<5> warm_start_impulse = mthz::NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u = mthz::Vec3(), mthz::Vec3 source_w = mthz::Vec3());

		inline int getDegree() override { return 5; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::NMat<3, 5> rotDirA;
		mthz::NMat<3, 5> rotDirB;
	};

	class PistonConstraint : public DegreedConstraint<1> {
	public:
		PistonConstraint() {}
		PistonConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis, double target_velocity, double max_impulse, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse = mthz::NVec<1>{ 0.0 });

		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		mthz::NVec<1> projectValidImpulse(mthz::NVec<1> impulse) override;
		inline bool needsPosCorrect() override { return false; }

	private:

		double max_impulse;
		mthz::Vec3 rot_dir;
		mthz::Vec3 pos_diff;
		mthz::Vec3 slide_axis;
	};

	class SlideLimitConstraint : public DegreedConstraint<1> {
	public:
		SlideLimitConstraint() {}
		SlideLimitConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis, double slide_position, double positive_slide_limit, double negative_slide_limit, double pos_correct_hardness, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse = mthz::NVec<1>{ 0.0 });

		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		mthz::NVec<1> projectValidImpulse(mthz::NVec<1> impulse) override;
		inline bool needsPosCorrect() override { return true; }

	private:
		enum LimitStatus { BELOW_MIN, ABOVE_MAX };

		LimitStatus slide_limit_status;
		mthz::Vec3 rot_dir;
		mthz::Vec3 pos_diff;
		mthz::Vec3 slide_axis;
	};

	class SlidingHingeConstraint : public DegreedConstraint<4> {
	public:
		SlidingHingeConstraint() {}
		SlidingHingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<4> warm_start_impulse = mthz::NVec<4>{ 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u = mthz::Vec3(), mthz::Vec3 source_w = mthz::Vec3());

		inline int getDegree() override { return 4; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 n;
		mthz::NMat<3, 4> rotDirA;
		mthz::NMat<3, 4> rotDirB;
	};

	class WeldConstraint : public DegreedConstraint<6> {
	public:
		WeldConstraint() {}
		WeldConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 a_attach_point, mthz::Vec3 b_attach_point, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<6> warm_start_impulse = mthz::NVec<6>{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

		inline int getDegree() override { return 6; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 n;
		mthz::NMat<3, 6> rotDirA;
		mthz::NMat<3, 6> rotDirB;
	};

	class TestConstraint : public DegreedConstraint<6> {
	public:
		TestConstraint() {}
		TestConstraint(RigidBody* a, RigidBody* b);

		inline int getDegree() override { return 6; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return false; }
	};
}