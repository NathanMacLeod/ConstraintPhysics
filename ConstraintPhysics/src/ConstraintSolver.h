#pragma once
#include "RigidBody.h"

namespace phyz {

	class Constraint;
	class PhysicsEngine;
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, int n_itr_vel = 15, int n_itr_pos = 10);

	template <int dim>
	struct NVec {
		double v[dim];

		bool isZero();
		NVec<dim> operator+(const NVec<dim>& r) const;
		NVec<dim> operator-(const NVec<dim>& r) const;
		NVec<dim> operator-() const;
		void operator+=(const NVec<dim>& r);
	};

	template <int dim>
	struct NMat {
		double v[dim][dim];

		NVec<dim> operator*(const NVec<dim>& v) const;
	};

	class Constraint {
	public:
		Constraint() {}
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

		virtual bool constraintWarmStarted() = 0;
		virtual void warmStartVelocityChange(VelVec* va, VelVec* vb) = 0;
		virtual void performPGSConstraintStep() = 0;
		virtual void performPGSPsuedoConstraintStep() = 0;
	};

	class ContactConstraint : public Constraint {
	public:
		ContactConstraint() {}
		ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, NVec<1> warm_start_impulse=NVec<1>{ 0.0 }, double cutoff_vel=0);

		bool constraintWarmStarted() override { return !impulse.isZero(); }
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
		NMat<1> inverse_inertia;
		NVec<1> target_val;
		NVec<1> psuedo_target_val;
	};

	class FrictionConstraint : public Constraint {
	public:
		FrictionConstraint() : normal_impulse(nullptr) {}
		FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 frictionDir, mthz::Vec3 contact_p, double coeff_friction, int n_contact_points, ContactConstraint* normal, NVec<1> warm_start_impulse=NVec<1>{ 0.0 });

		bool constraintWarmStarted() override { return !impulse.isZero(); }
		void warmStartVelocityChange(VelVec* va, VelVec* vb) override;
		void performPGSConstraintStep() override;
		void performPGSPsuedoConstraintStep() override { return; };

		NVec<1> getConstraintValue(const VelVec& va, const VelVec& vb);
		void addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb);
		bool getStaticReady() { return static_ready; }

		NVec<1> impulse;
	private:
		mthz::Vec3 frictionDir;
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 rotDirA;
		mthz::Vec3 rotDirB;
		NMat<1> inverse_inertia;
		NVec<1> target_val;
		double coeff_friction;
		NVec<1>* normal_impulse;
		bool static_ready;
	};

	class BallSocketConstraint : public Constraint {
	public:
		BallSocketConstraint() {}
		BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_p, mthz::Vec3 error, double pos_correct_hardness, NVec<3> warm_start_impulse=NVec<3>{ 0.0 });
		
		bool constraintWarmStarted() override { return !impulse.isZero(); }
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
		NMat<3> inverse_inertia;
		NVec<3> target_val;
		NVec<3> psuedo_target_val;
		
	};

}