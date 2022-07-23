#include "ConstraintSolver.h"
#include <unordered_map>

namespace phyz {
	static void PGS(const std::vector<Constraint*>& constraints, std::unordered_map<RigidBody*, Constraint::VelVec>* velocity_changes, int n_itr, bool solve_psuedo_vel) {

		for (int i = 0; i < n_itr; i++) {
			for (Constraint* c : constraints) {
				if (solve_psuedo_vel && !c->needsPositionalCorrection()) {
					continue;
				}

				double target_val = solve_psuedo_vel? c->getPsuedoTarget() : c->getTargetValue();
				double impulse = solve_psuedo_vel? c->psuedo_impulse : c->impulse;

				double val_diff = target_val - c->getConstraintValue((*velocity_changes)[c->a], (*velocity_changes)[c->b]);
				double impulse_new = impulse + val_diff / c->getInertiaVal();
				double impulse_diff = c->projectValidImpulse(impulse_new) - impulse; //apply projection on the total impulse, not the individual contributions. 
				if (impulse_diff != 0) {
					c->addVelocityChange(impulse_diff, &(*velocity_changes)[c->a], &(*velocity_changes)[c->b]);

					if (solve_psuedo_vel) { c->psuedo_impulse += impulse_diff; }
					else                  { c->impulse += impulse_diff; }
				}
			}
		}

	}

	//Projected Gauss-Seidel solver, see Iterative Dynamics with Temporal Coherence by Erin Catto 
	//the first third of this video explains it pretty well I think: https://www.youtube.com/watch?v=P-WP1yMOkc4 (Improving an Iterative Physics Solver Using a Direct Method)
	void PGS_solve(const std::vector<Constraint*>& constraints, int n_itr_vel, int n_itr_pos) {
		std::unordered_map<RigidBody*, Constraint::VelVec> velocity_changes;
		std::unordered_map<RigidBody*, Constraint::VelVec> psuedo_velocities;

		for (Constraint* c : constraints) {
			velocity_changes[c->a] = { mthz::Vec3(), mthz::Vec3() };
			velocity_changes[c->b] = { mthz::Vec3(), mthz::Vec3() };
			psuedo_velocities[c->a] = { mthz::Vec3(), mthz::Vec3() };
			psuedo_velocities[c->b] = { mthz::Vec3(), mthz::Vec3() };
		}

		/*for (int i = 0; i < n_itr; i++) {
			for (Constraint* c : constraints) {
				double val_diff = c->getTargetValue() - c->getConstraintValue(velocity_changes[c->a], velocity_changes[c->b]);
				double impulse_new = c->impulse + val_diff / c->getInertiaVal();
				double impulse_diff = c->projectValidImpulse(impulse_new) - c->impulse; //apply projection on the total impulse, not the individual contributions. 
				if (impulse_diff != 0) {
					c->addVelocityChange(impulse_diff, &velocity_changes[c->a], &velocity_changes[c->b]);
					c->impulse += impulse_diff;
				}
			}
		}*/

		//apply warm starting
		for (Constraint* c : constraints) {
			if (c->impulse != 0) {
				c->addVelocityChange(c->impulse, &velocity_changes[c->a], &velocity_changes[c->b]);
			}
		}

		PGS(constraints, &velocity_changes, n_itr_vel, false);
		PGS(constraints, &psuedo_velocities, n_itr_pos, true);

		for (const auto& kv_pair : velocity_changes) {
			kv_pair.first->vel += kv_pair.second.lin;
			kv_pair.first->ang_vel += kv_pair.second.ang;
		}
		for (const auto& kv_pair : psuedo_velocities) {
			kv_pair.first->psuedo_vel += kv_pair.second.lin;
			kv_pair.first->psuedo_ang_vel += kv_pair.second.ang;
		}
	}

	ContactConstraint::ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, double warm_start_impulse)
		: Constraint(a, b, warm_start_impulse), norm(norm), rA(contact_p - a->com), rB(contact_p - b->com)
	{
		rotDirA = a->getInvTensor() * norm.cross(rA);
		rotDirB = b->getInvTensor() * norm.cross(rB);
		inertia_val = a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(norm) + rB.cross(rotDirB).dot(norm);
		target_val = -(1 + bounce) * getConstraintValue({ a->vel, a->ang_vel }, { b->vel, b->ang_vel });
		psuedo_target_val = pen_depth * pos_correct_hardness;
	}

	void ContactConstraint::addVelocityChange(double impulse, VelVec* va, VelVec* vb) {
		va->lin -= norm * impulse * a->getInvMass();
		vb->lin += norm * impulse * b->getInvMass();
		va->ang += rotDirA * impulse;
		vb->ang -= rotDirB * impulse;
	}

	double ContactConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return vb.lin.dot(norm) - rB.cross(vb.ang).dot(norm) - va.lin.dot(norm) + rA.cross(va.ang).dot(norm);
	}

	double ContactConstraint::getInertiaVal() {
		return inertia_val;
	}

	double ContactConstraint::getTargetValue() {
		return target_val;
	}

	double ContactConstraint::getPsuedoTarget() {
		return psuedo_target_val;
	}

	double ContactConstraint::projectValidImpulse(double impulse) {
		return std::max<double>(impulse, 0);
	}

	FrictionConstraint::FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 frictionDir, mthz::Vec3 contact_p, double coeff_friction, int n_contact_points, ContactConstraint* normal, double warm_start_impulse)
		: Constraint(a, b, warm_start_impulse), frictionDir(frictionDir), rA(contact_p - a->com), rB(contact_p - b->com),
		coeff_friction(coeff_friction / n_contact_points), normal_impulse(&normal->impulse)
	{
		rotDirA = a->getInvTensor() * frictionDir.cross(rA);
		rotDirB = b->getInvTensor() * frictionDir.cross(rB);
		inertia_val = a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(frictionDir) + rB.cross(rotDirB).dot(frictionDir);
		target_val = -getConstraintValue({ a->vel, a->ang_vel }, { b->vel, b->ang_vel });
	}

	void FrictionConstraint::addVelocityChange(double impulse, VelVec* va, VelVec* vb) {
		va->lin -= frictionDir * impulse * a->getInvMass();
		vb->lin += frictionDir * impulse * b->getInvMass();
		va->ang += rotDirA * impulse;
		vb->ang -= rotDirB * impulse;
	}

	double FrictionConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return vb.lin.dot(frictionDir) - rB.cross(vb.ang).dot(frictionDir) - va.lin.dot(frictionDir) + rA.cross(va.ang).dot(frictionDir);
	}

	double FrictionConstraint::getInertiaVal() {
		return inertia_val;
	}

	double FrictionConstraint::getTargetValue() {
		return target_val;
	}

	double FrictionConstraint::projectValidImpulse(double impulse) {
		double max_impulse = coeff_friction * (*normal_impulse);
		if      (impulse < -max_impulse) return -max_impulse; 
		else if (impulse >  max_impulse) return max_impulse; 
		else                             return impulse; 
	}

}