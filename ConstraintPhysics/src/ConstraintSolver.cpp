#include "ConstraintSolver.h"
#include "PhysicsEngine.h"
#include <unordered_map>

namespace phyz {
	static void PGS(const std::vector<Constraint*>& constraints, int n_itr, bool solve_psuedo_vel) {

		for (int i = 0; i < n_itr; i++) {
			for (Constraint* c : constraints) {
				if (solve_psuedo_vel && !c->needsPositionalCorrection()) {
					continue;
				}
				Constraint::VelVec* vel_change_a = solve_psuedo_vel? c->a_psuedo_velocity_changes : c->a_velocity_changes;
				Constraint::VelVec* vel_change_b = solve_psuedo_vel? c->b_psuedo_velocity_changes : c->b_velocity_changes;

				double target_val = solve_psuedo_vel? c->getPsuedoTarget() : c->getTargetValue();
				double impulse = solve_psuedo_vel? c->psuedo_impulse : c->impulse;

				double val_diff = target_val - c->getConstraintValue(*vel_change_a, *vel_change_b);
				double impulse_new = impulse + val_diff / c->getInertiaVal();
				double impulse_diff = c->projectValidImpulse(impulse_new) - impulse; //apply projection on the total impulse, not the individual contributions. 
				if (impulse_diff != 0) {
					c->addVelocityChange(impulse_diff, vel_change_a, vel_change_b);

					if (solve_psuedo_vel) { c->psuedo_impulse += impulse_diff; }
					else                  { c->impulse += impulse_diff; }
				}
			}
		}

	}

	//Projected Gauss-Seidel solver, see Iterative Dynamics with Temporal Coherence by Erin Catto 
	//the first third of this video explains it pretty well I think: https://www.youtube.com/watch?v=P-WP1yMOkc4 (Improving an Iterative Physics Solver Using a Direct Method)
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, int n_itr_vel, int n_itr_pos) {
		struct VelPair {
			VelPair() :velocity({mthz::Vec3(), mthz::Vec3()}), psuedo_vel({ mthz::Vec3(), mthz::Vec3() }) {} //initialize zeroed out
			Constraint::VelVec velocity;
			Constraint::VelVec psuedo_vel;
		};

		std::unordered_map<RigidBody*, VelPair*> velocity_changes;

		for (Constraint* c : constraints) {
			VelPair* vA = nullptr; VelPair* vB = nullptr;
			if (velocity_changes.find(c->a) == velocity_changes.end()) {
				vA = new VelPair();
				velocity_changes[c->a] = vA;
			}
			else {
				vA = velocity_changes[c->a];
			}
			if (velocity_changes.find(c->b) == velocity_changes.end()) {
				vB = new VelPair();
				velocity_changes[c->b] = vB;
			}
			else {
				vB = velocity_changes[c->b];
			}
			c->a_velocity_changes = &vA->velocity;
			c->a_psuedo_velocity_changes = &vA->psuedo_vel;
			c->b_velocity_changes = &vB->velocity;
			c->b_psuedo_velocity_changes = &vB->psuedo_vel;
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
				c->addVelocityChange(c->impulse, c->a_velocity_changes, c->b_velocity_changes);
			}
		}

		PGS(constraints, n_itr_vel, false);
		PGS(constraints, n_itr_pos, true);

		for (const auto& kv_pair : velocity_changes) {
			pEngine->applyVelocityChange(kv_pair.first, kv_pair.second->velocity.lin, kv_pair.second->velocity.ang);
			kv_pair.first->psuedo_vel += kv_pair.second->psuedo_vel.lin;
			kv_pair.first->psuedo_ang_vel += kv_pair.second->psuedo_vel.ang;
			delete kv_pair.second;
		}
	}

	ContactConstraint::ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, double warm_start_impulse, double cutoff_vel)
		: Constraint(a, b, warm_start_impulse), norm(norm), rA(contact_p - a->com), rB(contact_p - b->com)
	{
		rotDirA = a->getInvTensor() * norm.cross(rA);
		rotDirB = b->getInvTensor() * norm.cross(rB);
		inertia_val = a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(norm) + rB.cross(rotDirB).dot(norm);
		double current_val = getConstraintValue({ a->vel, a->ang_vel }, { b->vel, b->ang_vel });
		target_val = (current_val < -cutoff_vel)? - (1 + bounce) * current_val : -current_val;
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
		coeff_friction(coeff_friction / n_contact_points), normal_impulse(&normal->impulse), static_ready(false)
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
		if (impulse < -max_impulse) {
			static_ready = false;
			return -max_impulse;
		}
		else if (impulse > max_impulse) {
			static_ready = false;
			return max_impulse;
		}
		else {
			static_ready = true;
			return impulse;
		}
	}

}