#include "ConstraintSolver.h"
#include "PhysicsEngine.h"
#include <unordered_map>

#include <chrono>
static int CUTOFF = 0.00000001;

namespace phyz {
	
	template<int dim>
	static void PGS_constraint_step(Constraint::VelVec* vel_change_a, Constraint::VelVec* vel_change_b, const NVec<dim>& target_val, NVec<dim>* impulse, 
		const NVec<dim>& current_constraint_value, const NMat<dim>& inverse_inertia_mat, std::function<NVec<dim>(const NVec<dim>& impulse)> projectValidImpulse, 
		std::function<void(const NVec<dim>& impulse, Constraint::VelVec* vel_change_a, Constraint::VelVec* vel_change_b)> addVelocityChange) 
	{
		NVec<dim> val_diff = target_val - current_constraint_value;
		NVec<dim> impulse_new = (*impulse) + inverse_inertia_mat * val_diff;
		NVec<dim> impulse_diff = projectValidImpulse(impulse_new) - (*impulse);

		if (!impulse_diff.isZero()) {
			addVelocityChange(impulse_diff, vel_change_a, vel_change_b);
			*impulse += impulse_diff;
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

		//apply warm starting
		for (Constraint* c : constraints) {
			if (c->constraintWarmStarted()) {
				c->warmStartVelocityChange(c->a_velocity_changes, c->b_velocity_changes);
			}
		}

		for (int i = 0; i < n_itr_vel; i++) {
			for (Constraint* c : constraints) {
				c->performPGSConstraintStep();
			}
		}
		for (int i = 0; i < n_itr_pos; i++) {
			for (Constraint* c : constraints) {
				c->performPGSPsuedoConstraintStep();
			}
		}

		for (const auto& kv_pair : velocity_changes) {
			RigidBody* b = kv_pair.first;
			VelPair* deltaV = kv_pair.second;
			pEngine->applyVelocityChange(b, deltaV->velocity.lin, deltaV->velocity.ang, deltaV->psuedo_vel.lin, deltaV->psuedo_vel.ang);
			delete deltaV;
		}
	}

	ContactConstraint::ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, NVec<1> warm_start_impulse, double cutoff_vel)
		: Constraint(a, b), norm(norm), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<1>{0.0})
	{
		rotDirA = a->getInvTensor() * norm.cross(rA);
		rotDirB = b->getInvTensor() * norm.cross(rB);
		inverse_inertia = NMat<1>{ {1.0 / (a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(norm) + rB.cross(rotDirB).dot(norm))} };
		double current_val = getConstraintValue({ a->getVel(), a->getAngVel()}, {b->getVel(), b->getAngVel()}).v[0];
		target_val = NVec<1>{ (current_val < -cutoff_vel) ? -(1 + bounce) * current_val : -current_val };
		psuedo_target_val = NVec<1>{ pen_depth * pos_correct_hardness };
	}

	void ContactConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void ContactConstraint::performPGSConstraintStep() {
		PGS_constraint_step<1>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[](const NVec<1>& impulse) { return NVec<1>{ std::max<double>(impulse.v[0], 0) }; },
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	void ContactConstraint::performPGSPsuedoConstraintStep() {
		PGS_constraint_step<1>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[](const NVec<1>& impulse) { return NVec<1>{ std::max<double>(impulse.v[0], 0) }; },
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	void ContactConstraint::addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb) {
		va->lin -= norm * impulse.v[0] * a->getInvMass();
		vb->lin += norm * impulse.v[0] * b->getInvMass();
		va->ang += rotDirA * impulse.v[0];
		vb->ang -= rotDirB * impulse.v[0];
	}

	NVec<1> ContactConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return NVec<1> {vb.lin.dot(norm) - rB.cross(vb.ang).dot(norm) - va.lin.dot(norm) + rA.cross(va.ang).dot(norm)};
	}

	FrictionConstraint::FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 frictionDir, mthz::Vec3 contact_p, double coeff_friction, int n_contact_points, ContactConstraint* normal, NVec<1> warm_start_impulse)
		: Constraint(a, b), frictionDir(frictionDir), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM()), impulse(warm_start_impulse),
		coeff_friction(coeff_friction / n_contact_points), normal_impulse(&normal->impulse), static_ready(false)
	{
		rotDirA = a->getInvTensor() * frictionDir.cross(rA);
		rotDirB = b->getInvTensor() * frictionDir.cross(rB);
		inverse_inertia = NMat<1>{ {1.0 / (a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(frictionDir) + rB.cross(rotDirB).dot(frictionDir)) } };
		target_val = -getConstraintValue({ a->getVel(), a->getAngVel()}, {b->getVel(), b->getAngVel()});
	}

	void FrictionConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void FrictionConstraint::performPGSConstraintStep() {

		PGS_constraint_step<1>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[&](const NVec<1>& impulse) { 
				double max_impulse = coeff_friction * normal_impulse->v[0];
				if (impulse.v[0] < -max_impulse) {
					static_ready = false;
					return NVec<1>{-max_impulse};
				}
				else if (impulse.v[0] > max_impulse) {
					static_ready = false;
					return NVec<1>{max_impulse};
				}
				else {
					static_ready = true;
					return impulse;
				}
			},
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	void FrictionConstraint::addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb) {
		va->lin -= frictionDir * impulse.v[0] * a->getInvMass();
		vb->lin += frictionDir * impulse.v[0] * b->getInvMass();
		va->ang += rotDirA * impulse.v[0];
		vb->ang -= rotDirB * impulse.v[0];
	}

	NVec<1> FrictionConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return NVec<1> { vb.lin.dot(frictionDir) - rB.cross(vb.ang).dot(frictionDir) - va.lin.dot(frictionDir) + rA.cross(va.ang).dot(frictionDir) };
	}

	BallSocketConstraint::BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_p, mthz::Vec3 error, double pos_correct_hardness, NVec<3> warm_start_impulse) 
		: Constraint(a, b), rA(socket_p - a->getCOM()), rB(socket_p - b->getCOM()), rotDirA(a->getInvTensor() * mthz::Mat3::cross_mat(rA)), rotDirB(b->getInvTensor() * mthz::Mat3::cross_mat(rB)),
		impulse(warm_start_impulse), psuedo_impulse(NVec<3>{ 0.0, 0.0, 0.0 })
	{
		mthz::Mat3 inverse_inertia_mat3 = (mthz::Mat3::iden()*a->getInvMass() + mthz::Mat3::iden()*b->getInvMass() - mthz::Mat3::cross_mat(rA)*rotDirA - mthz::Mat3::cross_mat(rB)*rotDirB).inverse();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				inverse_inertia.v[i][j] = inverse_inertia_mat3.v[i][j];
			}
		}
		target_val = -getConstraintValue({ a->getVel(), a->getAngVel()}, {b->getVel(), b->getAngVel()});
		psuedo_target_val = NVec<3>{ pos_correct_hardness * error.x, pos_correct_hardness * error.y, pos_correct_hardness * error.z };
	}

	void BallSocketConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void BallSocketConstraint::performPGSConstraintStep() {
		PGS_constraint_step<3>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[](const NVec<3>& impulse) { return impulse; },
			[&](const NVec<3>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	void BallSocketConstraint::performPGSPsuedoConstraintStep() {
		PGS_constraint_step<3>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[](const NVec<3>& impulse) { return impulse; },
			[&](const NVec<3>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	NVec<3> BallSocketConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		mthz::Vec3 value = va.lin - rA.cross(va.ang) - vb.lin + rB.cross(vb.ang);
		return NVec<3>{ value.x, value.y, value.z };
	}

	void BallSocketConstraint::addVelocityChange(const NVec<3>& impulse, VelVec* va, VelVec* vb) {
		mthz::Vec3 impulse_vec(impulse.v[0], impulse.v[1], impulse.v[2]);
		va->lin += impulse_vec / a->getMass();
		vb->lin -= impulse_vec / b->getMass();
		va->ang += rotDirA * impulse_vec;
		vb->ang -= rotDirB * impulse_vec;
	}

	template<int dim>
	bool NVec<dim>::isZero() {
		for (int i = 0; i < dim; i++) {
			if (abs(v[i]) > CUTOFF) {
				return false;
			}
		}
		return true;
	}

	template<int dim>
	NVec<dim> NVec<dim>::operator+(const NVec& r) const {
		NVec<dim> out;
		for (int i = 0; i < dim; i++) {
			out.v[i] = v[i] + r.v[i];
		}
		return out;
	}

	template<int dim>
	NVec<dim> NVec<dim>::operator-(const NVec& r) const {
		NVec<dim> out;
		for (int i = 0; i < dim; i++) {
			out.v[i] = v[i] - r.v[i];
		}
		return out;
	}

	template<int dim>
	NVec<dim> NVec<dim>::operator-() const {
		NVec<dim> out;
		for (int i = 0; i < dim; i++) {
			out.v[i] = -v[i];
		}
		return out;
	};

	template<int dim>
	void NVec<dim>::operator+=(const NVec<dim>& r) {
		for (int i = 0; i < dim; i++) {
			v[i] += r.v[i];
		}
	}

	template<int dim>
	NVec<dim> NMat<dim>::operator*(const NVec<dim>& n_vec) const {
		NVec<dim> out;
		for (int i = 0; i < dim; i++) {
			out.v[i] = 0;
			for (int j = 0; j < dim; j++) {
				out.v[i] += n_vec.v[j] * v[i][j];
			}
		}
		return out;
	}
}