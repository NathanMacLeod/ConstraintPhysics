#pragma once
#include "RigidBody.h"

namespace phyz {

	class Constraint;
	class PhysicsEngine;
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, int n_itr_vel, int n_itr_pos);

	template <int n>
	struct NVec {
		double v[n] = {0};

		bool isZero();

		NVec<n> operator+(const NVec<n>& r) const;
		NVec<n> operator-(const NVec<n>& r) const;
		NVec<n> operator-() const;
		double mag() const {
			double sum = 0;
			for (int i = 0; i < n; i++) {
				sum += v[i] * v[i];
			}
			return sqrt(sum);
		}
		void operator+=(const NVec<n>& r);
	};

	template <int n_row, int n_col>
	struct NMat {
		double v[n_row][n_col] = {0};

		NMat<n_row, n_col> inverse() const;
		NMat<n_col, n_row> transpose() const;
		template<int r, int c>
		void copyInto(const NMat<r, c>& m, int row, int col) {
			for (int i = 0; i < r; i++) {
				for (int j = 0; j < c; j++) {
					v[i + row][j + col] = m.v[i][j];
				}
			}
		}

		NMat<n_row, n_col> operator+(const NMat<n_row, n_col>& r) const;
		NMat<n_row, n_col> operator-(const NMat<n_row, n_col>& r) const;
		NVec<n_row> operator*(const NVec<n_col>& v) const;
		NMat<n_row, n_col> operator-() const;
		template<int x>
		NMat<n_row, x> operator*(const NMat<n_col, x>& r) const {
			NMat<n_row, x> out;
			for (int i = 0; i < n_row; i++) {
				for (int j = 0; j < x; j++) {
					out.v[i][j] = 0;
					for (int k = 0; k < n_col; k++) {
						out.v[i][j] += v[i][k] * r.v[k][j];
					}
				}
			}
			return out;
		}
		NMat<n_row, n_col> operator*(const double d) const {
			NMat<n_row, n_col> out;
			for (int i = 0; i < n_row; i++) {
				for (int j = 0; j < n_col; j++) {
					out.v[i][j] = v[i][j] * d;
				}
			}
			return out;
		}
	};

	class Constraint {
	public:
		Constraint() : a(nullptr), b(nullptr), a_velocity_change(nullptr), a_psuedo_velocity_change(nullptr), b_velocity_change(nullptr), b_psuedo_velocity_change(nullptr) {}
		Constraint(RigidBody* a, RigidBody* b) : a(a), b(b), a_velocity_change(nullptr), a_psuedo_velocity_change(nullptr), b_velocity_change(nullptr), b_psuedo_velocity_change(nullptr) {}
		virtual ~Constraint() {}

		RigidBody* a;
		RigidBody* b;
		NVec<6>* a_velocity_change;
		NVec<6>* a_psuedo_velocity_change;
		NVec<6>* b_velocity_change;
		NVec<6>* b_psuedo_velocity_change;

		virtual int getDegree() = 0;
		virtual bool isInequalityConstraint() = 0;
		virtual bool needsPosCorrect() = 0;
		virtual bool constraintWarmStarted() = 0;
		virtual void applyWarmStartVelocityChange() = 0;
	protected:
		//these are pretty inefficient to multiply with, but nice for sanity checking
		NMat<6, 6> aInvMass();
		NMat<6, 6> bInvMass();
	};

	template<int n>
	class DegreedConstraint : public Constraint {
	public:
		DegreedConstraint() : impulse(NVec<n>{0.0}) {}
		DegreedConstraint(RigidBody* a, RigidBody* b, NVec<n> impulse) : Constraint(a, b), impulse(impulse), psuedo_impulse(NVec<n>{0.0}) {}

		virtual NVec<n> projectValidImpulse(NVec<n> impulse) { return impulse; }
		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void applyWarmStartVelocityChange() override {
			computeAndApplyVelocityChange(impulse, a_velocity_change, b_velocity_change);
		}
		//for some constraints this can be computed much more effeciently than generalized matrix multiplciation, so providing override since this is performance impacting code
		virtual void computeAndApplyVelocityChange(NVec<n> impulse, NVec<6>* va, NVec<6>* vb) {
			*va += impulse_to_a_velocity * impulse;
			*vb += impulse_to_b_velocity * impulse;
		}

		NVec<n> getConstraintValue(const NVec<6>& va, const NVec<6>& vb) {
			return a_jacobian * va + b_jacobian * vb;
		}

		NVec<n> impulse;
		NVec<n> psuedo_impulse;
		NVec<n> target_val;
		NVec<n> psuedo_target_val;
		NMat<n, 6> a_jacobian;
		NMat<n, 6> b_jacobian;
		NMat<6, n> impulse_to_a_velocity;
		NMat<6, n> impulse_to_b_velocity;
		NMat<n, n> impulse_to_value;
		NMat<n, n> impulse_to_value_inverse;
 	};

	class ContactConstraint : public DegreedConstraint<1> {
	public:
		ContactConstraint() {}
		ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, double constraint_force_mixing, NVec<1> warm_start_impulse=NVec<1>{ 0.0 }, double cutoff_vel=0);
		
		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		NVec<1> projectValidImpulse(NVec<1> impulse) override;
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
		FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double coeff_friction, ContactConstraint* normal, double constraint_force_mixing, NVec<2> warm_start_impulse = NVec<2>{ 0.0, 0.0 }, mthz::Vec3 source_u = mthz::Vec3(), mthz::Vec3 source_w = mthz::Vec3(), double normal_impulse_limit = std::numeric_limits<double>::infinity());

		inline int getDegree() override { return 2; }
		inline bool isInequalityConstraint() override { return true; }
		NVec<2> projectValidImpulse(NVec<2> impulse) override;
		inline bool needsPosCorrect() override { return false; }

		bool getStaticReady() { return static_ready; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		NMat<3, 2> rotDirA;
		NMat<3, 2> rotDirB;
		double coeff_friction;
		NVec<1>* normal_impulse;
		double normal_impulse_limit;
		bool static_ready;
	};

	class BallSocketConstraint : public DegreedConstraint<3> {
	public:
		BallSocketConstraint() {}
		BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_pos_a, mthz::Vec3 socket_pos_b, double pos_correct_hardness, double constraint_force_mixing, NVec<3> warm_start_impulse=NVec<3>{ 0.0 });
		
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
		HingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, NVec<5> warm_start_impulse=NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u=mthz::Vec3(), mthz::Vec3 source_w=mthz::Vec3());

		inline int getDegree() override { return 5; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 n;
		NMat<3, 5> rotDirA;
		NMat<3, 5> rotDirB;
	};

	class MotorConstraint : public DegreedConstraint<1> {
	public:
		MotorConstraint() {}
		MotorConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 motor_axis, double target_velocity, double max_torque_impulse, double current_angle, double min_angle, double max_angle, double rot_correct_hardness, double constraint_force_mixing, NVec<1> warm_start_impulse = NVec<1>{ 0.0 });

		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		NVec<1> projectValidImpulse(NVec<1> impulse) override;
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
		SliderConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slider_point_a, mthz::Vec3 slider_point_b, mthz::Vec3 slider_axis_a, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, NVec<5> warm_start_impulse = NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u = mthz::Vec3(), mthz::Vec3 source_w = mthz::Vec3());

		inline int getDegree() override { return 5; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		NMat<3, 5> rotDirA;
		NMat<3, 5> rotDirB;
	};

	class PistonConstraint : public DegreedConstraint<1> {
	public:
		PistonConstraint() {}
		PistonConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis, double target_velocity, double max_impulse, double constraint_force_mixing, NVec<1> warm_start_impulse = NVec<1>{ 0.0 });

		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		NVec<1> projectValidImpulse(NVec<1> impulse) override;
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
		SlideLimitConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis, double slide_position, double positive_slide_limit, double negative_slide_limit, double pos_correct_hardness, double constraint_force_mixing, NVec<1> warm_start_impulse = NVec<1>{ 0.0 });

		inline int getDegree() override { return 1; }
		inline bool isInequalityConstraint() override { return true; }
		NVec<1> projectValidImpulse(NVec<1> impulse) override;
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
		SlidingHingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, NVec<4> warm_start_impulse = NVec<4>{ 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u = mthz::Vec3(), mthz::Vec3 source_w = mthz::Vec3());

		inline int getDegree() override { return 4; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 n;
		NMat<3, 4> rotDirA;
		NMat<3, 4> rotDirB;
	};

	class WeldConstraint : public DegreedConstraint<6> {
	public:
		WeldConstraint() {}
		WeldConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 a_attach_point, mthz::Vec3 b_attach_point, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, NVec<6> warm_start_impulse = NVec<6>{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

		inline int getDegree() override { return 6; }
		inline bool isInequalityConstraint() override { return false; }
		inline bool needsPosCorrect() override { return true; }

	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 n;
		NMat<3, 6> rotDirA;
		NMat<3, 6> rotDirB;
	};
}