#pragma once
#include "RigidBody.h"

namespace phyz {

	class Constraint;
	class PhysicsEngine;
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, int n_itr_vel, int n_itr_pos);

	template <int n>
	struct NVec {
		double v[n];

		bool isZero();

		NVec<n> operator+(const NVec<n>& r) const;
		NVec<n> operator-(const NVec<n>& r) const;
		NVec<n> operator-() const;
		void operator+=(const NVec<n>& r);
	};

	template <int n_row, int n_col>
	struct NMat {
		double v[n_row][n_col];

		NMat<n_row, n_col> inverse() const;
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

	};

	class Constraint {
	public:
		Constraint() : a(nullptr), b(nullptr), a_velocity_changes(nullptr), a_psuedo_velocity_changes(nullptr), b_velocity_changes(nullptr), b_psuedo_velocity_changes(nullptr) {}
		Constraint(RigidBody* a, RigidBody* b) : a(a), b(b), a_velocity_changes(nullptr), a_psuedo_velocity_changes(nullptr), b_velocity_changes(nullptr), b_psuedo_velocity_changes(nullptr) {}
		virtual ~Constraint() {}

		static struct VelVec {
			mthz::Vec3 lin;
			mthz::Vec3 ang;
		};

		RigidBody* a;
		RigidBody* b;
		VelVec* a_velocity_changes;
		VelVec* a_psuedo_velocity_changes;
		VelVec* b_velocity_changes;
		VelVec* b_psuedo_velocity_changes;
		bool debug_track = false;

		virtual bool constraintWarmStarted() = 0;
		virtual void warmStartVelocityChange(VelVec* va, VelVec* vb) = 0;
		virtual void performPGSConstraintStep() = 0;
		virtual void performPGSPsuedoConstraintStep() = 0;
	};

	class ContactConstraint : public Constraint {
	public:
		ContactConstraint() : impulse(NVec<1>{0.0}) {}
		ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, NVec<1> warm_start_impulse=NVec<1>{ 0.0 }, double cutoff_vel=0);

		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void warmStartVelocityChange(VelVec* va, VelVec* vb) override;
		void performPGSConstraintStep() override;
		void performPGSPsuedoConstraintStep() override;

		NVec<1> getConstraintValue(const VelVec& va, const VelVec& vb);
		void addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb);

		NVec<1> impulse;
		NVec<1> psuedo_impulse;
	private:
		
		mthz::Vec3 norm;
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 rotDirA;
		mthz::Vec3 rotDirB;
		NMat<1, 12> jacobian;
		NMat<1,1> inverse_inertia;
		NVec<1> target_val;
		NVec<1> psuedo_target_val;
	};

	class FrictionConstraint : public Constraint {
	public:
		FrictionConstraint() : normal_impulse(nullptr), impulse(NVec<2>{0.0}) {}
		FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double coeff_friction, int n_contact_points, ContactConstraint* normal, NVec<2> warm_start_impulse=NVec<2>{ 0.0, 0.0 }, mthz::Vec3 source_u=mthz::Vec3(), mthz::Vec3 source_w=mthz::Vec3());

		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void warmStartVelocityChange(VelVec* va, VelVec* vb) override;
		void performPGSConstraintStep() override;
		void performPGSPsuedoConstraintStep() override { return; };

		NVec<2> getConstraintValue(const VelVec& va, const VelVec& vb);
		void addVelocityChange(const NVec<2>& impulse, VelVec* va, VelVec* vb);
		bool getStaticReady() { return static_ready; }

		NVec<2> impulse;
		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		NMat<3, 2> rotDirA;
		NMat<3, 2> rotDirB;
		NMat<2, 12> jacobian;
		NMat<2,2> inverse_inertia;
		NVec<2> target_val;
		double coeff_friction;
		NVec<1>* normal_impulse;
		bool static_ready;
	};

	class BallSocketConstraint : public Constraint {
	public:
		BallSocketConstraint() : impulse(NVec<3>{0.0, 0.0, 0.0}) {}
		BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_pos_a, mthz::Vec3 socket_pos_b, double pos_correct_hardness, NVec<3> warm_start_impulse=NVec<3>{ 0.0 });
		
		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void warmStartVelocityChange(VelVec* va, VelVec* vb) override;
		void performPGSConstraintStep() override;
		void performPGSPsuedoConstraintStep() override;

		NVec<3> getConstraintValue(const VelVec& va, const VelVec& vb);
		void addVelocityChange(const NVec<3>& impulse, VelVec* va, VelVec* vb);

		NVec<3> impulse;
		NVec<3> psuedo_impulse;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Mat3 rotDirA;
		mthz::Mat3 rotDirB;
		NMat<3, 12> jacobian;
		NMat<3,3> inverse_inertia;
		NVec<3> target_val;
		NVec<3> psuedo_target_val;
		
	};

	class HingeConstraint : public Constraint {
	public:
		HingeConstraint() : impulse(NVec<5>{0.0, 0.0, 0.0, 0.0, 0.0}) {}
		HingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, NVec<5> warm_start_impulse=NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u=mthz::Vec3(), mthz::Vec3 source_w=mthz::Vec3());

		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void warmStartVelocityChange(VelVec* va, VelVec* vb) override;
		void performPGSConstraintStep() override;
		void performPGSPsuedoConstraintStep() override;

		NVec<5> getConstraintValue(const VelVec& va, const VelVec& vb);
		void addVelocityChange(const NVec<5>& impulse, VelVec* va, VelVec* vb);

		NVec<5> impulse;
		NVec<5> psuedo_impulse;
		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 n;
		NMat<3, 5> rotDirA;
		NMat<3, 5> rotDirB;
		NMat<5, 12> jacobian;
		NMat<5, 5> inverse_inertia;
		NVec<5> target_val;
		NVec<5> psuedo_target_val;
	};

	class MotorConstraint : public Constraint {
	public:
		MotorConstraint() : impulse(NVec<1>{0.0}) {}
		MotorConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 motor_axis, double target_velocity, double max_torque, NVec<1> warm_start_impulse = NVec<1>{ 0.0 });

		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void warmStartVelocityChange(VelVec* va, VelVec* vb) override;
		void performPGSConstraintStep() override;
		void performPGSPsuedoConstraintStep() override;

		NVec<1> getConstraintValue(const VelVec& va, const VelVec& vb);
		void addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb);

		NVec<1> impulse;
	private:
		double max_torque;
		mthz::Vec3 motor_axis;
		mthz::Vec3 rotDirA;
		mthz::Vec3 rotDirB;
		NMat<1, 1> inverse_inertia;
		NVec<1> target_val;
		NVec<1> psuedo_target_val;
	};

	class SliderConstraint : public Constraint {
	public:
		SliderConstraint() : impulse(NVec<5>{0.0, 0.0, 0.0, 0.0, 0.0}) {}
		SliderConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slider_point_a, mthz::Vec3 slider_point_b, mthz::Vec3 slider_axis_a, mthz::Vec3 slider_axis_b, double pos_correct_hardness, double rot_correct_hardness, NVec<5> warm_start_impulse = NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 }, mthz::Vec3 source_u = mthz::Vec3(), mthz::Vec3 source_w = mthz::Vec3());

		inline bool constraintWarmStarted() override { return !impulse.isZero(); }
		void warmStartVelocityChange(VelVec* va, VelVec* vb) override;
		void performPGSConstraintStep() override;
		void performPGSPsuedoConstraintStep() override;

		NVec<5> getConstraintValue(const VelVec& va, const VelVec& vb);
		void addVelocityChange(const NVec<5>& impulse, VelVec* va, VelVec* vb);

		NVec<5> impulse;
		NVec<5> psuedo_impulse;
		mthz::Vec3 u;
		mthz::Vec3 w;
	private:
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		NMat<3, 5> rotDirA;
		NMat<3, 5> rotDirB;
		NMat<5, 12> jacobian;
		NMat<5, 5> inverse_inertia;
		NVec<5> target_val;
		NVec<5> psuedo_target_val;
	};

}