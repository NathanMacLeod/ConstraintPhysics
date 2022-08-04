#pragma once
#include "RigidBody.h"

namespace phyz {

	class Constraint;
	class PhysicsEngine;
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, int n_itr_vel = 15, int n_itr_pos = 10);

	class Constraint {
	public:
		Constraint() {}
		Constraint(RigidBody* a, RigidBody* b, double impulse) : a(a), b(b), impulse(impulse), psuedo_impulse(0), 
			a_velocity_changes(nullptr), a_psuedo_velocity_changes(nullptr), b_velocity_changes(nullptr), b_psuedo_velocity_changes(nullptr) {}
		virtual ~Constraint() {}

		static struct VelVec {
			mthz::Vec3 lin;
			mthz::Vec3 ang;
		};

		RigidBody* a;
		RigidBody* b;
		double impulse;
		double psuedo_impulse;
		VelVec* a_velocity_changes;
		VelVec* a_psuedo_velocity_changes;
		VelVec* b_velocity_changes;
		VelVec* b_psuedo_velocity_changes;

		virtual bool needsPositionalCorrection() = 0;
		virtual void addVelocityChange(double impulse, VelVec* va, VelVec* vb) = 0;
		virtual double getConstraintValue(const VelVec& va, const VelVec& vb) = 0;
		virtual double getInertiaVal() = 0;
		virtual double getTargetValue() = 0;
		virtual double getPsuedoTarget() = 0;
		virtual double projectValidImpulse(double impulse) = 0;
		
	};

	class ContactConstraint : public Constraint {
	public:
		ContactConstraint() {}
		ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, double warm_start_impulse=0, double cutoff_vel=0);

		bool needsPositionalCorrection() override { return true; }
		void addVelocityChange(double impulse, VelVec* va, VelVec* vb) override;
		double getConstraintValue(const VelVec& va, const VelVec& vb) override;
		double getInertiaVal() override;
		double getTargetValue() override;
		double getPsuedoTarget() override;
		double projectValidImpulse(double impulse) override;

	private:
		mthz::Vec3 norm;
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 rotDirA;
		mthz::Vec3 rotDirB;
		double inertia_val;
		double target_val;
		double psuedo_target_val;
	};

	class FrictionConstraint : public Constraint {
	public:
		FrictionConstraint() : normal_impulse(nullptr) {}
		FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 frictionDir, mthz::Vec3 contact_p, double coeff_friction, int n_contact_points, ContactConstraint* normal, double warm_start_impulse=0);

		bool needsPositionalCorrection() override { return false; }
		void addVelocityChange(double impulse, VelVec* va, VelVec* vb) override;
		double getConstraintValue(const VelVec& va, const VelVec& vb) override;
		double getInertiaVal() override;
		double getTargetValue() override;
		double getPsuedoTarget() override { return 0; }
		double projectValidImpulse(double impulse) override;

		bool getStaticReady() { return static_ready; }
	private:
		mthz::Vec3 frictionDir;
		mthz::Vec3 rA;
		mthz::Vec3 rB;
		mthz::Vec3 rotDirA;
		mthz::Vec3 rotDirB;
		double inertia_val;
		double target_val;
		double coeff_friction;
		double* normal_impulse;
		bool static_ready;
	};

}