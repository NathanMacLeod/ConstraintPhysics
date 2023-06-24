#include "ConstraintSolver.h"
#include "PhysicsEngine.h"
#include <unordered_map>
#include <cassert>

#include <chrono>
static int CUTOFF = 0.00000001;

namespace phyz {
	
	template<int n>
	static NMat<n, n> idenMat();
	static NMat<3, 3> Mat3toNMat33(const mthz::Mat3& m);
	static NVec<3> Vec3toNVec3(const mthz::Vec3& v);
	static mthz::Vec3 NVec3toVec3(const NVec<3>& v);

	static NVec<12> VelVectoNVec(const phyz::Constraint::VelVec& vel_a, const phyz::Constraint::VelVec& vel_b);

	template<int n, typename ProjectFunc, typename AddVelFunc>
	static void PGS_constraint_step(Constraint::VelVec* vel_change_a, Constraint::VelVec* vel_change_b, const NVec<n>& target_val, NVec<n>* impulse, 
		const NVec<n>& current_constraint_value, const NMat<n,n>& inverse_inertia_mat, ProjectFunc projectValidImpulse, AddVelFunc addVelocityChange) 
	{
		NVec<n> val_diff = target_val - current_constraint_value;
		NVec<n> impulse_new = (*impulse) + inverse_inertia_mat * val_diff;
		NVec<n> impulse_diff = projectValidImpulse(impulse_new) - (*impulse);

		if (!impulse_diff.isZero()) {
			addVelocityChange(impulse_diff, vel_change_a, vel_change_b);
			*impulse += impulse_diff;
		}
	}

	//Projected Gauss-Seidel solver, see Iterative Dynamics with Temporal Coherence by Erin Catto 
	//the first third of this video explains it pretty well I think: https://www.youtube.com/watch?v=P-WP1yMOkc4 (Improving an Iterative Physics Solver Using a Direct Method)
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, int n_itr_vel, int n_itr_pos) {
		struct VelPair {
			VelPair() : velocity({mthz::Vec3(), mthz::Vec3()}), psuedo_vel({ mthz::Vec3(), mthz::Vec3() }) {} //initialize zeroed out
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

				if ((c->a->getID() == 967)) {
					int a = 1 + 2;
				}

				c->warmStartVelocityChange(c->a_velocity_changes, c->b_velocity_changes);
			}
		}

		for (int i = 0; i < n_itr_vel; i++) {
			for (Constraint* c : constraints) {

				phyz::Constraint::VelVec* va = c->a_velocity_changes;

				if ((c->a->getID() == 967)) {
					int a = 1 + 2;
				}

				c->performPGSConstraintStep();

				if ((c->a->getID() == 967)) {
					int a = 1 + 2;
				}
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

	//******************************
	//*****CONTACT CONSTRAINT*******
	//******************************
	ContactConstraint::ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, NVec<1> warm_start_impulse, double cutoff_vel)
		: Constraint(a, b), norm(norm), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<1>{0.0})
	{

		if (a->getID() == 967) {
			//a->setAngVel(mthz::Vec3());
		}

		rotDirA = a->getInvTensor() * norm.cross(rA);
		rotDirB = b->getInvTensor() * norm.cross(rB);
		{
			NMat<3, 3> rA_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rA));
			NMat<3, 3> rB_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rB));
			NMat<1, 3> n_dot = NMat<1, 3>{ {norm.x, norm.y, norm.z} };

			jacobian.copyInto(n_dot, 0, 0);
			jacobian.copyInto(-n_dot * rA_skew, 0, 3);
			jacobian.copyInto(-n_dot, 0, 6);
			jacobian.copyInto(n_dot * rB_skew, 0, 9);
		}
		inverse_inertia = NMat<1,1>{ {1.0 / (a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(norm) + rB.cross(rotDirB).dot(norm))} };
		double current_val = getConstraintValue({ a->getVel(), a->getAngVel()}, {b->getVel(), b->getAngVel()}).v[0];
		target_val = NVec<1>{ (current_val < -cutoff_vel) ? -(1 + bounce) * current_val : -current_val };
		psuedo_target_val = NVec<1>{ pen_depth * pos_correct_hardness };

		if (a->getID() == 64) {
			//int a = 1 + 2;
		}
	}

	void ContactConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		if (a->getID() == 967) {
			//impulse.v[0] = 0;
		}
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

	inline void ContactConstraint::addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb) {
		va->lin -= norm * impulse.v[0] * a->getInvMass();
		vb->lin += norm * impulse.v[0] * b->getInvMass();
		va->ang += rotDirA * impulse.v[0];
		vb->ang -= rotDirB * impulse.v[0];
	}

	inline NVec<1> ContactConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return -jacobian * VelVectoNVec(va, vb);
	}

	//******************************
	//*****FRICTION CONSTRAINT******
	//******************************
	FrictionConstraint::FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double impulse_limit, NVec<2> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w)
		: Constraint(a, b), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM()), impulse(warm_start_impulse),
		static_ready(false), impulse_limit(impulse_limit)
	{

		if (a->getID() == 967) {
			int a = 1 + 2;
			//a->setAngVel(mthz::Vec3());
		}

		norm.getPerpendicularBasis(&u, &w);

		impulse.v[0] = u.dot(source_u) * warm_start_impulse.v[0] + u.dot(source_w) * warm_start_impulse.v[1];
		impulse.v[1] = w.dot(source_u) * warm_start_impulse.v[0] + w.dot(source_w) * warm_start_impulse.v[1];

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();
		mthz::Vec3 rAxU = rA.cross(u);
		mthz::Vec3 rAxW = rA.cross(w);
		mthz::Vec3 rBxU = rB.cross(u);
		mthz::Vec3 rBxW = rB.cross(w);

		{
			mthz::Vec3 l = Ia_inv * rAxU;
			mthz::Vec3 r = Ia_inv * rAxW;
			rotDirA.v[0][0] = l.x; rotDirA.v[0][1] = r.x;
			rotDirA.v[1][0] = l.y; rotDirA.v[1][1] = r.y;
			rotDirA.v[2][0] = l.z; rotDirA.v[2][1] = r.z;
		}
		{
			mthz::Vec3 l = Ib_inv * rBxU;
			mthz::Vec3 r = Ib_inv * rBxW;
			rotDirB.v[0][0] = l.x; rotDirB.v[0][1] = r.x;
			rotDirB.v[1][0] = l.y; rotDirB.v[1][1] = r.y;
			rotDirB.v[2][0] = l.z; rotDirB.v[2][1] = r.z;
		}
		{
			NMat<1, 3> u_dot = { {u.x, u.y, u.z} };
			NMat<1, 3> w_dot = { {w.x, w.y, w.z} };
			NMat<3, 3> rA_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rA));
			NMat<3, 3> rB_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rB));

			jacobian.copyInto(u_dot, 0, 0);
			jacobian.copyInto(-u_dot * rA_skew, 0, 3);
			jacobian.copyInto(-u_dot, 0, 6);
			jacobian.copyInto(u_dot * rB_skew, 0, 9);

			jacobian.copyInto(w_dot, 1, 0);
			jacobian.copyInto(-w_dot * rA_skew, 1, 3);
			jacobian.copyInto(-w_dot, 1, 6);
			jacobian.copyInto(w_dot * rB_skew, 1, 9);
		}

		inverse_inertia.v[0][0] = a->getInvMass() + b->getInvMass() - rA.cross(Ia_inv * rAxU).dot(u) - rB.cross(Ib_inv * rBxU).dot(u);
		inverse_inertia.v[0][1] = -rA.cross(Ia_inv * rAxW).dot(u) - rB.cross(Ib_inv * rBxW).dot(u);
		inverse_inertia.v[1][0] = -rA.cross(Ia_inv * rAxU).dot(w) - rB.cross(Ib_inv * rBxU).dot(w);
		inverse_inertia.v[1][1] = a->getInvMass() + b->getInvMass() - rA.cross(Ia_inv * rAxW).dot(w) - rB.cross(Ib_inv * rBxW).dot(w);

		inverse_inertia = inverse_inertia.inverse();

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() });
	}

	inline void FrictionConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		if (a->getID() == 967) {
			//impulse.v[0] = 0;
			//impulse.v[1] = 0;
		}
		addVelocityChange(impulse, va, vb);
	}

	void FrictionConstraint::performPGSConstraintStep() {

		PGS_constraint_step<2>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[&](const NVec<2>& impulse) {
				double current_impulse_mag2 = impulse.v[0] * impulse.v[0] + impulse.v[1] * impulse.v[1];
				if (current_impulse_mag2 > impulse_limit * impulse_limit) {
					static_ready = false;
					double r = impulse_limit / sqrt(current_impulse_mag2);
					return NVec<2>{ impulse.v[0] * r, impulse.v[1] * r };
				}
				else {
					static_ready = true;
					return impulse;
				}
			},
			[&](const NVec<2>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	inline void FrictionConstraint::addVelocityChange(const NVec<2>& impulse, VelVec* va, VelVec* vb) {
		va->lin += (u * impulse.v[0] + w * impulse.v[1]) * a->getInvMass();
		vb->lin -= (u * impulse.v[0] + w * impulse.v[1]) * b->getInvMass();
		va->ang += NVec3toVec3(rotDirA * impulse);
		vb->ang -= NVec3toVec3(rotDirB * impulse);
	}

	inline NVec<2> FrictionConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return jacobian * VelVectoNVec(va, vb);
	}

	//******************************
	//****BALL SOCKET CONSTRAINT****
	//******************************
	BallSocketConstraint::BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_pos_a, mthz::Vec3 socket_pos_b, double pos_correct_hardness, NVec<3> warm_start_impulse)
		: Constraint(a, b), rA(socket_pos_a - a->getCOM()), rB(socket_pos_a - b->getCOM()), rotDirA(a->getInvTensor() * mthz::Mat3::cross_mat(rA)), rotDirB(b->getInvTensor() * mthz::Mat3::cross_mat(rB)),
		impulse(warm_start_impulse), psuedo_impulse(NVec<3>{ 0.0, 0.0, 0.0 })
	{
		mthz::Mat3 inverse_inertia_mat3 = (mthz::Mat3::iden()*a->getInvMass() + mthz::Mat3::iden()*b->getInvMass() - mthz::Mat3::cross_mat(rA)*rotDirA - mthz::Mat3::cross_mat(rB)*rotDirB).inverse();
		inverse_inertia = Mat3toNMat33(inverse_inertia_mat3);
		
		NMat<3, 3> rA_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rA));
		NMat<3, 3> rB_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rB));

		jacobian.copyInto(idenMat<3>(), 0, 0);
		jacobian.copyInto(-rA_skew, 0, 3);
		jacobian.copyInto(-idenMat<3>(), 0, 6);
		jacobian.copyInto(rB_skew, 0, 9);

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel()}, {b->getVel(), b->getAngVel()});
		mthz::Vec3 error = socket_pos_b - socket_pos_a;
		psuedo_target_val = NVec<3>{ pos_correct_hardness * error.x, pos_correct_hardness * error.y, pos_correct_hardness * error.z };
	}

	inline void BallSocketConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
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

	inline NVec<3> BallSocketConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return jacobian * VelVectoNVec(va, vb);
	}

	inline void BallSocketConstraint::addVelocityChange(const NVec<3>& impulse, VelVec* va, VelVec* vb) {
		mthz::Vec3 impulse_vec(impulse.v[0], impulse.v[1], impulse.v[2]);
		va->lin += impulse_vec * a->getInvMass();
		vb->lin -= impulse_vec * b->getInvMass();
		va->ang += rotDirA * impulse_vec;
		vb->ang -= rotDirB * impulse_vec;
	}

	//******************************
	//*****HINGE CONSTRAINT*********
	//******************************
	HingeConstraint::HingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, NVec<5> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w)
		: Constraint(a, b), rA(hinge_pos_a - a->getCOM()), rB(hinge_pos_a - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 })
	{
		rot_axis_a.getPerpendicularBasis(&u, &w);
		n = rot_axis_b;

		impulse.v[3] = u.dot(source_u) * warm_start_impulse.v[3] + u.dot(source_w) * warm_start_impulse.v[4];
		impulse.v[4] = w.dot(source_u) * warm_start_impulse.v[3] + w.dot(source_w) * warm_start_impulse.v[4];

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();

		mthz::Mat3 rA_skew = mthz::Mat3::cross_mat(rA);
		mthz::Mat3 rB_skew = mthz::Mat3::cross_mat(rB);

		//IMPULSE TO ROTATION MATRICES
		{
			NMat<3, 3> l = Mat3toNMat33(Ia_inv * rA_skew); //left block
			mthz::Vec3 m = Ia_inv * (n.cross(u));          //middle block
			mthz::Vec3 r = Ia_inv * (n.cross(w));          //right block

			rotDirA.copyInto(l, 0, 0);
			rotDirA.v[0][3] = m.x; rotDirA.v[1][3] = m.y; rotDirA.v[2][3] = m.z;
			rotDirA.v[0][4] = r.x; rotDirA.v[1][4] = r.y; rotDirA.v[2][4] = r.z;
		}
		{
			NMat<3, 3> l = Mat3toNMat33(Ib_inv * rB_skew); //left block
			mthz::Vec3 m = Ib_inv * (n.cross(u));          //middle block
			mthz::Vec3 r = Ib_inv * (n.cross(w));          //right block

			rotDirB.copyInto(l, 0, 0);
			rotDirB.v[0][3] = m.x; rotDirB.v[1][3] = m.y; rotDirB.v[2][3] = m.z;
			rotDirB.v[0][4] = r.x; rotDirB.v[1][4] = r.y; rotDirB.v[2][4] = r.z;
		}
		
		//INVERSE INERTIA MATRIX
		{
			NMat<3, 3> ul = Mat3toNMat33(mthz::Mat3::iden() * a->getInvMass() + mthz::Mat3::iden() * b->getInvMass() - rA_skew * Ia_inv * rA_skew - rB_skew * Ib_inv * rB_skew); //upper left
			mthz::Vec3 um = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(u)); //upper middle
			mthz::Vec3 ur = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(w)); //upper right

			NMat<3, 3> rA_skew_mat = Mat3toNMat33(rA_skew);
			NMat<3, 3> rB_skew_mat = Mat3toNMat33(rB_skew);
			NMat<3, 3> u_skew = Mat3toNMat33(mthz::Mat3::cross_mat(u));
			NMat<3, 3> w_skew = Mat3toNMat33(mthz::Mat3::cross_mat(w));
			NMat<3, 3> n_skew = Mat3toNMat33(mthz::Mat3::cross_mat(n));
			NMat<1, 3> u_dot = NMat<1, 3>{ {u.x, u.y, u.z} };
			NMat<1, 3> w_dot = NMat<1, 3>{ {w.x, w.y, w.z} };
			NMat<1, 3> n_dot = NMat<1, 3>{ {n.x, n.y, n.z} };
			NMat<1, 3> zero = NMat<1, 3>{ {0, 0, 0} };

			NMat<1, 5> m = n_dot * u_skew * rotDirA - u_dot * n_skew * rotDirB; //middle row
			NMat<1, 5> b = n_dot * w_skew * rotDirA - w_dot * n_skew * rotDirB; //bottom row

			jacobian.copyInto(idenMat<3>(), 0, 0);
			jacobian.copyInto(-rA_skew_mat, 0, 3);
			jacobian.copyInto(-idenMat<3>(), 0, 6);
			jacobian.copyInto(rB_skew_mat, 0, 9);
			
			jacobian.copyInto(zero, 3, 0);
			jacobian.copyInto(n_dot * u_skew, 3, 3);
			jacobian.copyInto(zero, 3, 6);
			jacobian.copyInto(u_dot * n_skew, 3, 9);
			jacobian.copyInto(zero, 4, 0);
			jacobian.copyInto(n_dot * w_skew, 4, 3);
			jacobian.copyInto(zero, 4, 6);
			jacobian.copyInto(w_dot * n_skew, 4, 9);

			inverse_inertia.copyInto(ul, 0, 0);
			inverse_inertia.v[0][3] = um.x; inverse_inertia.v[1][3] = um.y; inverse_inertia.v[2][3] = um.z;
			inverse_inertia.v[0][4] = ur.x; inverse_inertia.v[1][4] = ur.y; inverse_inertia.v[2][4] = ur.z;
			inverse_inertia.copyInto(m, 3, 0);
			inverse_inertia.copyInto(b, 4, 0);

			inverse_inertia = inverse_inertia.inverse();
		}

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() });
		{
			mthz::Vec3 pos_correct = (hinge_pos_b - hinge_pos_a) * pos_correct_hardness;
			double u_correct = u.dot(n)* rot_correct_hardness;
			double w_correct = w.dot(n)* rot_correct_hardness;
			psuedo_target_val = NVec<5>{ pos_correct.x, pos_correct.y, pos_correct.z, u_correct, w_correct };
		}
	}

	inline void HingeConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void HingeConstraint::performPGSConstraintStep() {
		PGS_constraint_step<5>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[](const NVec<5>& impulse) { return impulse; },
			[&](const NVec<5>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	void HingeConstraint::performPGSPsuedoConstraintStep() {
		PGS_constraint_step<5>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[](const NVec<5>& impulse) { return impulse; },
			[&](const NVec<5>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	inline NVec<5> HingeConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return jacobian * VelVectoNVec(va, vb);
	}

	inline void HingeConstraint::addVelocityChange(const NVec<5>& impulse, VelVec* va, VelVec* vb) {
		mthz::Vec3 linear_impulse_vec(impulse.v[0], impulse.v[1], impulse.v[2]);
		va->lin += linear_impulse_vec * a->getInvMass();
		vb->lin -= linear_impulse_vec * b->getInvMass();
		va->ang += NVec3toVec3(rotDirA * impulse);
		vb->ang -= NVec3toVec3(rotDirB * impulse);
	}

	//******************************
	//*****MOTOR CONSTRAINT*********
	//******************************
	MotorConstraint::MotorConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 motor_axis, double target_velocity, double max_torque_impulse, double current_angle, double min_angle, double max_angle, double rot_correct_hardness, NVec<1> warm_start_impulse)
		: Constraint(a, b), motor_axis(motor_axis), impulse(warm_start_impulse), max_torque_impulse(max_torque_impulse)
	{
		double real_target_velocity;

		if (current_angle > max_angle) {
			psuedo_target_val = NVec<1>{ (max_angle - current_angle) * rot_correct_hardness };
			real_target_velocity = std::min<double>(0, target_velocity);
			rot_limit_status = ABOVE_MAX;
		}
		else if (current_angle < min_angle) {
			psuedo_target_val = NVec<1>{ (min_angle - current_angle) * rot_correct_hardness };
			real_target_velocity = std::max<double>(0, target_velocity);
			rot_limit_status = BELOW_MIN;
		}
		else {
			psuedo_target_val = NVec<1>{ 0 };
			real_target_velocity = target_velocity;
			rot_limit_status = NOT_EXCEEDED;
		}

		rotDirA = a->getInvTensor() * motor_axis;
		rotDirB = b->getInvTensor() * motor_axis;
		inverse_inertia = NMat<1, 1>{ {1.0 / (motor_axis.dot(rotDirA) + motor_axis.dot(rotDirB))} };
		double current_val = getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() }).v[0];
		target_val = NVec<1>{ real_target_velocity - current_val };
	}

	inline void MotorConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void MotorConstraint::performPGSConstraintStep() {
		PGS_constraint_step<1>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[&](const NVec<1>& impulse) {
				switch (rot_limit_status) {
				case NOT_EXCEEDED:
					if (impulse.v[0] > 0) 
						return NVec<1>{ std::min<double>(impulse.v[0], max_torque_impulse) };
					else
						return NVec<1>{ std::max<double>(impulse.v[0], -max_torque_impulse) };
				case ABOVE_MAX:
					return NVec<1>{ std::min<double>(impulse.v[0], 0) };
				case BELOW_MIN:
					return NVec<1>{ std::max<double>(impulse.v[0], 0) };
				}
				
			},
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	inline void MotorConstraint::performPGSPsuedoConstraintStep() {
		if (psuedo_target_val.v[0] == 0) return;

		PGS_constraint_step<1>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[&](const NVec<1>& impulse) { return impulse; },
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	inline void MotorConstraint::addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb) {
		va->ang += rotDirA * impulse.v[0];
		vb->ang -= rotDirB * impulse.v[0];
	}

	inline NVec<1> MotorConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return NVec<1> {motor_axis.dot(va.ang) - motor_axis.dot(vb.ang)};
	}

	////******************************
	////*****SLIDER CONSTRAINT********
	////******************************
	SliderConstraint::SliderConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slider_point_a, mthz::Vec3 slider_point_b, mthz::Vec3 slider_axis_a, double pos_correct_hardness, double rot_correct_hardness, NVec<5> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w)
		: Constraint(a, b), rA(slider_point_a - a->getCOM()), rB(slider_point_b - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 })
	{
		slider_axis_a = slider_axis_a.normalize();
		slider_axis_a.getPerpendicularBasis(&u, &w);

		impulse.v[0] = u.dot(source_u) * warm_start_impulse.v[0] + u.dot(source_w) * warm_start_impulse.v[1];
		impulse.v[1] = w.dot(source_u) * warm_start_impulse.v[0] + w.dot(source_w) * warm_start_impulse.v[1];

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();
		NMat<3, 3> Ia_inv_mat = Mat3toNMat33(Ia_inv);
		NMat<3, 3> Ib_inv_mat = Mat3toNMat33(Ib_inv);

		mthz::Vec3 pos_error = slider_point_a - slider_point_b;
		mthz::Vec3 rAxU = rA.cross(u);
		mthz::Vec3 rAxW = rA.cross(w);
		mthz::Vec3 rBxU = rB.cross(u);
		mthz::Vec3 rBxW = rB.cross(w);
		mthz::Vec3 UxPe = u.cross(pos_error);
		mthz::Vec3 WxPe = w.cross(pos_error);

		//IMPULSE TO ROTATION MATRICES
		{
			mthz::Vec3 l = Ia_inv * (rAxU + UxPe); //left block
			mthz::Vec3 m = Ia_inv * (rAxW + WxPe); //middle block

			rotDirA.v[0][0] = l.x; rotDirA.v[0][1] = m.x;
			rotDirA.v[1][0] = l.y; rotDirA.v[1][1] = m.y;
			rotDirA.v[2][0] = l.z; rotDirA.v[2][1] = m.z;
			rotDirA.copyInto(Ia_inv_mat, 0, 2);
		}
		{
			mthz::Vec3 l = Ib_inv * rBxU; //left block
			mthz::Vec3 m = Ib_inv * rBxW; //middle block

			rotDirB.v[0][0] = l.x; rotDirB.v[0][1] = m.x;
			rotDirB.v[1][0] = l.y; rotDirB.v[1][1] = m.y;
			rotDirB.v[2][0] = l.z; rotDirB.v[2][1] = m.z;
			rotDirB.copyInto(Ib_inv_mat, 0, 2);
		}

		//INVERSE INERTIA MATRIX
		{
			NMat<1, 3> u_dot = { u.x, u.y, u.z };
			NMat<1, 3> w_dot = { w.x, w.y, w.z };
			NMat<1, 3> Pe_dot = { pos_error.x, pos_error.y, pos_error.z };
			NMat<3, 3> rA_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rA));
			NMat<3, 3> rB_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rB));
			NMat<3, 3> u_skew = Mat3toNMat33(mthz::Mat3::cross_mat(u));
			NMat<3, 3> w_skew = Mat3toNMat33(mthz::Mat3::cross_mat(w));
			NMat<3, 3> zero = Mat3toNMat33(mthz::Mat3::zero());

			jacobian.copyInto(u_dot, 0, 0);
			jacobian.copyInto(-u_dot * rA_skew - Pe_dot * u_skew, 0, 3);
			jacobian.copyInto(-u_dot, 0, 6);
			jacobian.copyInto(u_dot * rB_skew, 0, 9);

			jacobian.copyInto(w_dot, 1, 0);
			jacobian.copyInto(-w_dot * rA_skew - Pe_dot * w_skew, 1, 3);
			jacobian.copyInto(-w_dot, 1, 6);
			jacobian.copyInto(w_dot * rB_skew, 1, 9);

			jacobian.copyInto(zero, 2, 0);
			jacobian.copyInto(idenMat<3>(), 2, 3);
			jacobian.copyInto(zero, 2, 6);
			jacobian.copyInto(-idenMat<3>(), 2, 9);

			////top row
			inverse_inertia.v[0][0] = a->getInvMass() + b->getInvMass() - u.dot(rA.cross(Ia_inv * (rAxU + UxPe))) - pos_error.dot(u.cross(Ia_inv * (rAxU + UxPe))) - u.dot(rB.cross(Ib_inv * (rBxU)));
			inverse_inertia.v[0][1] = -u.dot(rA.cross(Ia_inv * (rAxW + WxPe))) - pos_error.dot(u.cross(Ia_inv * (rAxW))) - u.dot(rB.cross(Ib_inv * (rBxW)));
			NMat<1, 3> ur = -(u_dot * rA_skew + Pe_dot * u_skew) * Ia_inv_mat - u_dot * rB_skew * Ib_inv_mat;
			inverse_inertia.v[0][2] = ur.v[0][0]; inverse_inertia.v[0][3] = ur.v[0][1]; inverse_inertia.v[0][4] = ur.v[0][2];

			//middle row
			inverse_inertia.v[1][0] = -w.dot(rA.cross(Ia_inv * (rAxU + UxPe))) - pos_error.dot(w.cross(Ia_inv * (rAxU + UxPe))) - w.dot(rB.cross(Ib_inv * (rBxU)));
			inverse_inertia.v[1][1] = a->getInvMass() + b->getInvMass() - w.dot(rA.cross(Ia_inv * (rAxW))) - pos_error.dot(w.cross(Ia_inv * (rAxW + WxPe))) - w.dot(rB.cross(Ib_inv * (rBxW)));
			NMat<1, 3> mr = -(w_dot * rA_skew + Pe_dot * w_skew) * Ia_inv_mat - w_dot * rB_skew * Ib_inv_mat;
			inverse_inertia.v[1][2] = mr.v[0][0]; inverse_inertia.v[1][3] = mr.v[0][1]; inverse_inertia.v[1][4] = mr.v[0][2];

			mthz::Vec3 ll = Ia_inv * (rAxU + UxPe) + Ib_inv * rBxU;
			mthz::Vec3 lm = Ia_inv * (rAxW + WxPe) + Ib_inv * rBxW;
			NMat<3, 3> lr = Mat3toNMat33(Ia_inv + Ib_inv);

			inverse_inertia.v[2][0] = ll.x; inverse_inertia.v[2][1] = lm.x;
			inverse_inertia.v[3][0] = ll.y; inverse_inertia.v[3][1] = lm.y;
			inverse_inertia.v[4][0] = ll.z; inverse_inertia.v[4][1] = lm.z;
			inverse_inertia.copyInto(lr, 2, 2);

			inverse_inertia = inverse_inertia.inverse();
			
		}

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() });
		{
			double u_correct = -pos_error.dot(u) * pos_correct_hardness;
			double w_correct = -pos_error.dot(w) * pos_correct_hardness;

			mthz::Quaternion orientation_diff = b->getOrientation() * a->getOrientation().conjugate();
			mthz::Vec3 rot_correct = mthz::Vec3(orientation_diff.i, orientation_diff.j, orientation_diff.k) * rot_correct_hardness;
			psuedo_target_val = NVec<5>{ u_correct, w_correct, rot_correct.x, rot_correct.y, rot_correct.z };
		}
	}

	inline void SliderConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void SliderConstraint::performPGSConstraintStep() {
		PGS_constraint_step<5>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[](const NVec<5>& impulse) { return impulse; },
			[&](const NVec<5>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	inline void SliderConstraint::performPGSPsuedoConstraintStep() {
		PGS_constraint_step<5>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[](const NVec<5>& impulse) { return impulse; },
			[&](const NVec<5>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	inline NVec<5> SliderConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return jacobian * VelVectoNVec(va, vb);
	}

	inline void SliderConstraint::addVelocityChange(const NVec<5>& impulse, VelVec* va, VelVec* vb) {
		va->lin += (impulse.v[0] * u + impulse.v[1] * w) * a->getInvMass();
		vb->lin -= (impulse.v[0] * u + impulse.v[1] * w) * b->getInvMass();
		va->ang += NVec3toVec3(rotDirA * impulse);
		vb->ang -= NVec3toVec3(rotDirB * impulse);
	}


	//******************************
	//******PISTON CONSTRAINT*******
	//******************************
	PistonConstraint::PistonConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis_a, double target_velocity, double max_impulse, NVec<1> warm_start_impulse)
		: Constraint(a, b), slide_axis(slide_axis_a), impulse(warm_start_impulse), max_impulse(max_impulse)
	{
		
		mthz::Mat3 Ia_inv = a->getInvTensor();

		pos_diff = b->getCOM() - a->getCOM();
		rot_dir = Ia_inv * (pos_diff.cross(slide_axis_a));

		inverse_inertia = NMat<1, 1>{ 1.0 / (a->getInvMass() + b->getInvMass() + pos_diff.dot(slide_axis_a.cross(rot_dir))) };

		double current_val = getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() }).v[0];
		target_val = NVec<1>{ target_velocity - current_val };
	}

	inline void PistonConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void PistonConstraint::performPGSConstraintStep() {
		PGS_constraint_step<1>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[&](const NVec<1>& impulse) {
				if (impulse.v[0] > 0)
					return NVec<1>{ std::min<double>(impulse.v[0], max_impulse) };
				else
					return NVec<1>{ std::max<double>(impulse.v[0], -max_impulse) };
			},
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	inline void PistonConstraint::addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb) {
		va->lin -= a->getInvMass() * slide_axis * impulse.v[0];
		vb->lin += b->getInvMass() * slide_axis * impulse.v[0];
		va->ang -= rot_dir * impulse.v[0];
	}

	inline NVec<1> PistonConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return NVec<1>{ slide_axis.dot(vb.lin) - slide_axis.dot(va.lin) - pos_diff.dot(slide_axis.cross(va.ang)) };
	}

	//******************************
	//****Slide Limit Constraint****
	//******************************
	SlideLimitConstraint::SlideLimitConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis_a, double slide_position, double positive_slide_limit, double negative_slide_limit, double pos_correct_hardness, NVec<1> warm_start_impulse)
		: Constraint(a, b), slide_axis(slide_axis_a), impulse(warm_start_impulse)
	{

		if (slide_position > positive_slide_limit) {
			psuedo_target_val = NVec<1>{ (positive_slide_limit - slide_position) * pos_correct_hardness };
			slide_limit_status = ABOVE_MAX;
		}
		else if (slide_position < negative_slide_limit) {
			psuedo_target_val = NVec<1>{ (negative_slide_limit - slide_position) * pos_correct_hardness };
			slide_limit_status = BELOW_MIN;
		}

		mthz::Mat3 Ia_inv = a->getInvTensor();

		pos_diff = b->getCOM() - a->getCOM();
		rot_dir = Ia_inv * (pos_diff.cross(slide_axis_a));

		inverse_inertia = NMat<1, 1>{ 1.0 / (a->getInvMass() + b->getInvMass() + pos_diff.dot(slide_axis_a.cross(rot_dir))) };

		double current_val = getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() }).v[0];
		target_val = NVec<1>{ -current_val };
	}

	inline void SlideLimitConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void SlideLimitConstraint::performPGSConstraintStep() {
		PGS_constraint_step<1>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[&](const NVec<1>& impulse) {
				switch (slide_limit_status) {
				case ABOVE_MAX:
					return NVec<1>{ std::min<double>(impulse.v[0], 0)};
				case BELOW_MIN:
					return NVec<1>{ std::max<double>(impulse.v[0], 0)};
				}
			},
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	inline void SlideLimitConstraint::performPGSPsuedoConstraintStep() {
		if (psuedo_target_val.v[0] == 0) return;

		PGS_constraint_step<1>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[&](const NVec<1>& impulse) { return impulse; },
			[&](const NVec<1>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	};

	inline void SlideLimitConstraint::addVelocityChange(const NVec<1>& impulse, VelVec* va, VelVec* vb) {
		va->lin -= a->getInvMass() * slide_axis * impulse.v[0];
		vb->lin += b->getInvMass() * slide_axis * impulse.v[0];
		va->ang -= rot_dir * impulse.v[0];
	}

	inline NVec<1> SlideLimitConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return NVec<1>{ slide_axis.dot(vb.lin) - slide_axis.dot(va.lin) - pos_diff.dot(slide_axis.cross(va.ang)) };
	}

	//******************************
	//***SLIDING HINGE CONSTRAINT***
	//******************************
	SlidingHingeConstraint::SlidingHingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_pos_a, mthz::Vec3 slide_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, NVec<4> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w)
		: Constraint(a, b), rA(slide_pos_a - a->getCOM()), rB(slide_pos_b - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<4>{ 0.0, 0.0, 0.0, 0.0 })
	{
		rot_axis_b.getPerpendicularBasis(&u, &w);
		n = rot_axis_a;

		impulse.v[2] = u.dot(source_u) * warm_start_impulse.v[2] + u.dot(source_w) * warm_start_impulse.v[3];
		impulse.v[3] = w.dot(source_u) * warm_start_impulse.v[2] + w.dot(source_w) * warm_start_impulse.v[3];

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();

		mthz::Mat3 rA_skew = mthz::Mat3::cross_mat(rA);
		mthz::Mat3 rB_skew = mthz::Mat3::cross_mat(rB);

		mthz::Vec3 pos_diff = slide_pos_a - slide_pos_b;

		//IMPULSE TO ROTATION MATRICES
		{
			mthz::Vec3 l = -Ia_inv * (u.cross(rA));		//left block
			mthz::Vec3 lm = -Ia_inv * (w.cross(rA));    //left middle block
			mthz::Vec3 rm = -Ia_inv * (u.cross(n));     //left middle block
			mthz::Vec3 r = -Ia_inv * (w.cross(n));      //right block

			rotDirA.v[0][0] = l.x; rotDirA.v[0][1] = lm.x; rotDirA.v[0][2] = rm.x; rotDirA.v[0][3] = r.x;
			rotDirA.v[1][0] = l.y; rotDirA.v[1][1] = lm.y; rotDirA.v[1][2] = rm.y; rotDirA.v[1][3] = r.y;
			rotDirA.v[2][0] = l.z; rotDirA.v[2][1] = lm.z; rotDirA.v[2][2] = rm.z; rotDirA.v[2][3] = r.z;
		}
		{
			mthz::Vec3 l = Ib_inv * (u.cross(rB) - pos_diff.cross(u));  //left block
			mthz::Vec3 lm = Ib_inv * (w.cross(rB) - pos_diff.cross(w)); //left middle block
			mthz::Vec3 rm = Ib_inv * (u.cross(n));                      //left middle block
			mthz::Vec3 r = Ib_inv * (w.cross(n));                       //right block

			rotDirB.v[0][0] = l.x; rotDirB.v[0][1] = lm.x; rotDirB.v[0][2] = rm.x; rotDirB.v[0][3] = r.x;
			rotDirB.v[1][0] = l.y; rotDirB.v[1][1] = lm.y; rotDirB.v[1][2] = rm.y; rotDirB.v[1][3] = r.y;
			rotDirB.v[2][0] = l.z; rotDirB.v[2][1] = lm.z; rotDirB.v[2][2] = rm.z; rotDirB.v[2][3] = r.z;
		}

		//INVERSE INERTIA MATRIX
		{
			NMat<3, 3> ul = Mat3toNMat33(mthz::Mat3::iden() * a->getInvMass() + mthz::Mat3::iden() * b->getInvMass() - rA_skew * Ia_inv * rA_skew - rB_skew * Ib_inv * rB_skew); //upper left
			mthz::Vec3 um = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(u)); //upper middle
			mthz::Vec3 ur = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(w)); //upper right

			NMat<3, 3> rA_skew_mat = Mat3toNMat33(rA_skew);
			NMat<3, 3> rB_skew_mat = Mat3toNMat33(rB_skew);
			NMat<3, 3> u_skew = Mat3toNMat33(mthz::Mat3::cross_mat(u));
			NMat<3, 3> w_skew = Mat3toNMat33(mthz::Mat3::cross_mat(w));
			NMat<3, 3> n_skew = Mat3toNMat33(mthz::Mat3::cross_mat(n));
			NMat<1, 3> u_dot = NMat<1, 3>{ {u.x, u.y, u.z} };
			NMat<1, 3> w_dot = NMat<1, 3>{ {w.x, w.y, w.z} };
			NMat<1, 3> n_dot = NMat<1, 3>{ {n.x, n.y, n.z} };
			NMat<1, 3> pd_dot = NMat<1, 3>{ {pos_diff.x, pos_diff.y, pos_diff.z} };

			jacobian.copyInto(u_dot, 0, 0); jacobian.copyInto(-u_dot * rA_skew_mat, 0, 3); jacobian.copyInto(-u_dot, 0, 6); jacobian.copyInto(u_dot * rB_skew_mat - pd_dot * u_skew, 0, 9);
			jacobian.copyInto(w_dot, 1, 0); jacobian.copyInto(-w_dot * rA_skew_mat, 1, 3); jacobian.copyInto(-w_dot, 1, 6); jacobian.copyInto(w_dot * rB_skew_mat - pd_dot * w_skew, 1, 9);
											jacobian.copyInto(-u_dot * n_skew, 2, 3);										jacobian.copyInto(-n_dot * u_skew, 2, 9);
											jacobian.copyInto(-w_dot * n_skew, 3, 3);										jacobian.copyInto(-n_dot * w_skew, 3, 9);

			NMat<12, 4> imp_to_vel;

			mthz::Vec3 uma = u * a->getInvMass();
			mthz::Vec3 wma = w * a->getInvMass();

			imp_to_vel.v[0][0] = uma.x; imp_to_vel.v[0][1] = wma.x;
			imp_to_vel.v[1][0] = uma.y; imp_to_vel.v[1][1] = wma.y;
			imp_to_vel.v[2][0] = uma.z; imp_to_vel.v[2][1] = wma.z;

			imp_to_vel.copyInto(rotDirA, 3, 0);

			mthz::Vec3 umb = u * b->getInvMass();
			mthz::Vec3 wmb = w * b->getInvMass();

			imp_to_vel.v[6][0] = -umb.x; imp_to_vel.v[6][1] = -wmb.x;
			imp_to_vel.v[7][0] = -umb.y; imp_to_vel.v[7][1] = -wmb.y;
			imp_to_vel.v[8][0] = -umb.z; imp_to_vel.v[8][1] = -wmb.z;

			imp_to_vel.copyInto(rotDirB, 9, 0);

			inverse_inertia = (jacobian * imp_to_vel).inverse(); //not effient, could expand out eventually
		}

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() });
		{
			double u_correct = -u.dot(pos_diff) * pos_correct_hardness;
			double w_correct = -w.dot(pos_diff) * pos_correct_hardness;
			double u_rot_correct = -u.dot(n) * rot_correct_hardness;
			double w_rot_correct = -w.dot(n) * rot_correct_hardness;
			psuedo_target_val = NVec<4>{ u_correct, w_correct, u_rot_correct, w_rot_correct };
		}
	}

	inline void SlidingHingeConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void SlidingHingeConstraint::performPGSConstraintStep() {
		PGS_constraint_step<4>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[](const NVec<4>& impulse) { return impulse; },
			[&](const NVec<4>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	void SlidingHingeConstraint::performPGSPsuedoConstraintStep() {
		PGS_constraint_step<4>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[](const NVec<4>& impulse) { return impulse; },
			[&](const NVec<4>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	inline NVec<4> SlidingHingeConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		return jacobian * VelVectoNVec(va, vb);
	}

	inline void SlidingHingeConstraint::addVelocityChange(const NVec<4>& impulse, VelVec* va, VelVec* vb) {
		va->lin += (impulse.v[0] * u + impulse.v[1] * w) * a->getInvMass();
		vb->lin -= (impulse.v[0] * u + impulse.v[1] * w) * b->getInvMass();
		va->ang += NVec3toVec3(rotDirA * impulse);
		vb->ang += NVec3toVec3(rotDirB * impulse);
	}

	////******************************
	////*******WELD CONSTRAINT********
	////******************************
	WeldConstraint::WeldConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 pos_a, mthz::Vec3 pos_b, double pos_correct_hardness, double rot_correct_hardness, NVec<6> warm_start_impulse)
		: Constraint(a, b), rA(pos_a - a->getCOM()), rB(pos_b - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<6>{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 })
	{

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();
		NMat<3, 3> Ia_inv_mat = Mat3toNMat33(Ia_inv);
		NMat<3, 3> Ib_inv_mat = Mat3toNMat33(Ib_inv);
		NMat<3, 3> rA_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rA));
		NMat<3, 3> rB_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rB));
		NMat<3, 3> iden_mat = idenMat<3>();

		rotDirA.copyInto(Ia_inv_mat * rA_skew, 0, 0); rotDirA.copyInto(Ia_inv_mat, 0, 3);

		rotDirB.copyInto(Ib_inv_mat * rB_skew, 0, 0); rotDirB.copyInto(Ib_inv_mat, 0, 3);

		inverse_inertia.copyInto(iden_mat * a->getInvMass() + iden_mat * b->getInvMass() - rA_skew * Ia_inv_mat * rA_skew - rB_skew * Ib_inv_mat * rB_skew, 0, 0);
		inverse_inertia.copyInto(-rA_skew * Ia_inv_mat - rB_skew * Ib_inv_mat, 0, 3);
		inverse_inertia.copyInto(Ia_inv_mat * rA_skew + Ib_inv_mat * rB_skew, 3, 0);
		inverse_inertia.copyInto(Ia_inv_mat + Ib_inv_mat, 3, 3);

		inverse_inertia = inverse_inertia.inverse();

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() });
		{
			mthz::Vec3 pos_correct = (pos_b - pos_a) * pos_correct_hardness;

			mthz::Quaternion orientation_diff = b->getOrientation() * a->getOrientation().conjugate();
			mthz::Vec3 rot_correct =  mthz::Vec3(orientation_diff.i, orientation_diff.j, orientation_diff.k)* rot_correct_hardness;
			psuedo_target_val = NVec<6>{ pos_correct.x, pos_correct.y, pos_correct.z, rot_correct.x, rot_correct.y, rot_correct.z };
		}
	}

	inline void WeldConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
		addVelocityChange(impulse, va, vb);
	}

	void WeldConstraint::performPGSConstraintStep() {
		PGS_constraint_step<6>(a_velocity_changes, b_velocity_changes, target_val, &impulse,
			getConstraintValue(*a_velocity_changes, *b_velocity_changes), inverse_inertia,
			[](const NVec<6>& impulse) { return impulse; },
			[&](const NVec<6>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	inline void WeldConstraint::performPGSPsuedoConstraintStep() {
		PGS_constraint_step<6>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[](const NVec<6>& impulse) { return impulse; },
			[&](const NVec<6>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	inline NVec<6> WeldConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		mthz::Vec3 pos_value = va.lin - rA.cross(va.ang) - vb.lin + rB.cross(vb.ang);
		mthz::Vec3 rot_value = va.ang - vb.ang;

		return NVec<6>{ pos_value.x, pos_value.y, pos_value.z, rot_value.x, rot_value.y, rot_value.z };
	}

	inline void WeldConstraint::addVelocityChange(const NVec<6>& impulse, VelVec* va, VelVec* vb) {
		va->lin += mthz::Vec3(impulse.v[0], impulse.v[1], impulse.v[2]) * a->getInvMass();
		vb->lin -= mthz::Vec3(impulse.v[0], impulse.v[1], impulse.v[2]) * b->getInvMass();
		va->ang += NVec3toVec3(rotDirA * impulse);
		vb->ang -= NVec3toVec3(rotDirB * impulse);
	}

	//******************************
	//******** NVEC / NMAT *********
	//******************************
	template<int n>
	bool NVec<n>::isZero() {
		for (int i = 0; i < n; i++) {
			if (abs(v[i]) > CUTOFF) {
				return false;
			}
		}
		return true;
	}

	template<int n>
	NVec<n> NVec<n>::operator+(const NVec& r) const {
		NVec<n> out;
		for (int i = 0; i < n; i++) {
			out.v[i] = this->v[i] + r.v[i];
		}
		return out;
	}

	template<int n>
	NVec<n> NVec<n>::operator-(const NVec& r) const {
		NVec<n> out;
		for (int i = 0; i < n; i++) {
			out.v[i] = this->v[i] - r.v[i];
		}
		return out;
	}

	template<int n>
	NVec<n> NVec<n>::operator-() const {
		NVec<n> out;
		for (int i = 0; i < n; i++) {
			out.v[i] = -this->v[i];
		}
		return out;
	};

	template<int n>
	void NVec<n>::operator+=(const NVec<n>& r) {
		for (int i = 0; i < n; i++) {
			v[i] += r.v[i];
		}
	}

	template <int n_row, int n_col>
	NVec<n_row> NMat<n_row, n_col>::operator*(const NVec<n_col>& n_vec) const {
		NVec<n_row> out;
		for (int i = 0; i < n_row; i++) {
			out.v[i] = 0;
			for (int j = 0; j < n_col; j++) {
				out.v[i] += n_vec.v[j] * this->v[i][j];
			}
		}
		return out;
	}

	template <int n_row, int n_col>
	NMat<n_row, n_col> NMat<n_row, n_col>::operator+(const NMat<n_row, n_col>& r) const {
		NMat<n_row, n_col> out;
		for (int i = 0; i < n_row; i++) {
			for (int j = 0; j < n_col; j++) {
				out.v[i][j] = this->v[i][j] + r.v[i][j];
			}
		}
		return out;
	}

	template <int n_row, int n_col>
	NMat<n_row, n_col> NMat<n_row, n_col>::operator-(const NMat<n_row, n_col>& r) const {
		NMat<n_row, n_col> out;
		for (int i = 0; i < n_row; i++) {
			for (int j = 0; j < n_col; j++) {
				out.v[i][j] = this->v[i][j] - r.v[i][j];
			}
		}
		return out;
	}

	template <int n_row, int n_col>
	NMat<n_row, n_col> NMat<n_row, n_col>::operator-() const {
		NMat<n_row, n_col> out;
		for (int i = 0; i < n_row; i++) {
			for (int j = 0; j < n_col; j++) {
				out.v[i][j] = -this->v[i][j];
			}
		}
		return out;
	}

	template<int n>
	static NMat<n, n> idenMat() {
		NMat<n, n> out;
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				out.v[i][j] = 0;
			}
			out.v[i][i] = 1;
		}
		return out;
	}

	static NMat<3, 3> Mat3toNMat33(const mthz::Mat3& m) {
		NMat<3, 3> out;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				out.v[i][j] = m.v[i][j];
			}
		}
		return out;
	}

	static NVec<3> Vec3toNVec3(const mthz::Vec3& v) {
		return NVec<3> {v.x, v.y, v.z};
	}

	static mthz::Vec3 NVec3toVec3(const NVec<3>& v) {
		return mthz::Vec3(v.v[0], v.v[1], v.v[2]);
	}

	template <int n_row, int n_col>
	NMat<n_row, n_col> NMat<n_row, n_col>::inverse() const {
		assert(n_row == n_col);

		NMat<n_row, n_row> copy = *this;
		NMat<n_row, n_row> out = idenMat<n_row>();

		for (int piv = 0; piv < n_row; piv++) {
			for (int target_row = 0; target_row < n_row; target_row++) {
				if (piv == target_row) continue;

				double ratio = copy.v[target_row][piv] / copy.v[piv][piv];
				for (int target_col = 0; target_col < n_row; target_col++) {
					copy.v[target_row][target_col] -= ratio * copy.v[piv][target_col];
					out.v[target_row][target_col] -= ratio * out.v[piv][target_col];
				}

			}
		}
		for (int piv = 0; piv < n_row; piv++) {
			for (int col = 0; col < n_row; col++) {
				out.v[piv][col] /= copy.v[piv][piv];
			}
		}

		return out;
	}

	static NVec<12> VelVectoNVec(const phyz::Constraint::VelVec& vel_a, const phyz::Constraint::VelVec& vel_b) {
		return NVec<12> {
			vel_a.lin.x, vel_a.lin.y, vel_a.lin.z,
			vel_a.ang.x, vel_a.ang.y, vel_a.ang.z,
			vel_b.lin.x, vel_b.lin.y, vel_b.lin.z,
			vel_b.ang.x, vel_b.ang.y, vel_b.ang.z,
		};
	}
};