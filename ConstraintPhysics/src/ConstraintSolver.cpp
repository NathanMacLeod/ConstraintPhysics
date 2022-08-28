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

	template<int n>
	static void PGS_constraint_step(Constraint::VelVec* vel_change_a, Constraint::VelVec* vel_change_b, const NVec<n>& target_val, NVec<n>* impulse, 
		const NVec<n>& current_constraint_value, const NMat<n,n>& inverse_inertia_mat, std::function<NVec<n>(const NVec<n>& impulse)> projectValidImpulse, 
		std::function<void(const NVec<n>& impulse, Constraint::VelVec* vel_change_a, Constraint::VelVec* vel_change_b)> addVelocityChange) 
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

	//******************************
	//*****CONTACT CONSTRAINT*******
	//******************************
	ContactConstraint::ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, NVec<1> warm_start_impulse, double cutoff_vel)
		: Constraint(a, b), norm(norm), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<1>{0.0})
	{
		rotDirA = a->getInvTensor() * norm.cross(rA);
		rotDirB = b->getInvTensor() * norm.cross(rB);
		inverse_inertia = NMat<1,1>{ {1.0 / (a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(norm) + rB.cross(rotDirB).dot(norm))} };
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

	//******************************
	//*****FRICTION CONSTRAINT******
	//******************************
	FrictionConstraint::FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 frictionDir, mthz::Vec3 contact_p, double coeff_friction, int n_contact_points, ContactConstraint* normal, NVec<1> warm_start_impulse)
		: Constraint(a, b), frictionDir(frictionDir), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM()), impulse(warm_start_impulse),
		coeff_friction(coeff_friction / n_contact_points), normal_impulse(&normal->impulse), static_ready(false)
	{
		rotDirA = a->getInvTensor() * frictionDir.cross(rA);
		rotDirB = b->getInvTensor() * frictionDir.cross(rB);
		inverse_inertia = NMat<1,1>{ {1.0 / (a->getInvMass() + b->getInvMass() + rA.cross(rotDirA).dot(frictionDir) + rB.cross(rotDirB).dot(frictionDir)) } };
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

	//******************************
	//****BALL SOCKET CONSTRAINT****
	//******************************
	BallSocketConstraint::BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_pos_a, mthz::Vec3 socket_pos_b, double pos_correct_hardness, NVec<3> warm_start_impulse)
		: Constraint(a, b), rA(socket_pos_a - a->getCOM()), rB(socket_pos_a - b->getCOM()), rotDirA(a->getInvTensor() * mthz::Mat3::cross_mat(rA)), rotDirB(b->getInvTensor() * mthz::Mat3::cross_mat(rB)),
		impulse(warm_start_impulse), psuedo_impulse(NVec<3>{ 0.0, 0.0, 0.0 })
	{
		mthz::Mat3 inverse_inertia_mat3 = (mthz::Mat3::iden()*a->getInvMass() + mthz::Mat3::iden()*b->getInvMass() - mthz::Mat3::cross_mat(rA)*rotDirA - mthz::Mat3::cross_mat(rB)*rotDirB).inverse();
		inverse_inertia = Mat3toNMat33(inverse_inertia_mat3);

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel()}, {b->getVel(), b->getAngVel()});
		mthz::Vec3 error = socket_pos_b - socket_pos_a;
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
		va->lin += impulse_vec * a->getInvMass();
		vb->lin -= impulse_vec * b->getInvMass();
		va->ang += rotDirA * impulse_vec;
		vb->ang -= rotDirB * impulse_vec;
	}

	//******************************
	//*****HINGE CONSTRAINT*********
	//******************************
	HingeConstraint::HingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, NVec<5> warm_start_impulse)
		: Constraint(a, b), rA(hinge_pos_a - a->getCOM()), rB(hinge_pos_a - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 })
	{
		rot_axis_a.getPerpendicularBasis(&u, &w);
		n = rot_axis_b;

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();

		mthz::Mat3 rA_skew = mthz::Mat3::cross_mat(rA);
		mthz::Mat3 rB_skew = mthz::Mat3::cross_mat(rB);

		//IMPULSE TO ROTATION MATRICES
		{
			mthz::Mat3 l = Ia_inv * rA_skew;	  //left block
			mthz::Vec3 m = Ia_inv * (n.cross(u)); //middle block
			mthz::Vec3 r = Ia_inv * (n.cross(w)); //right block

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					rotDirA.v[i][j] = l.v[i][j];
				}
			}
			rotDirA.v[0][3] = m.x; rotDirA.v[1][3] = m.y; rotDirA.v[2][3] = m.z;
			rotDirA.v[0][4] = r.x; rotDirA.v[1][4] = r.y; rotDirA.v[2][4] = r.z;
		}
		{
			mthz::Mat3 l = Ib_inv * rB_skew;      //left block
			mthz::Vec3 m = Ib_inv * (n.cross(u)); //middle block
			mthz::Vec3 r = Ib_inv * (n.cross(w)); //right block

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					rotDirB.v[i][j] = l.v[i][j];
				}
			}
			rotDirB.v[0][3] = m.x; rotDirB.v[1][3] = m.y; rotDirB.v[2][3] = m.z;
			rotDirB.v[0][4] = r.x; rotDirB.v[1][4] = r.y; rotDirB.v[2][4] = r.z;
		}
		
		//INVERSE INERTIA MATRIX
		{
			mthz::Mat3 ul = mthz::Mat3::iden() * a->getInvMass() + mthz::Mat3::iden() * b->getInvMass() - rA_skew * Ia_inv * rA_skew - rB_skew * Ib_inv * rB_skew; //upper left
			mthz::Vec3 um = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(u)); //upper middle
			mthz::Vec3 ur = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(w)); //upper right

			NMat<3, 3> u_skew = Mat3toNMat33(mthz::Mat3::cross_mat(u));
			NMat<3, 3> w_skew = Mat3toNMat33(mthz::Mat3::cross_mat(w));
			NMat<3, 3> n_skew = Mat3toNMat33(mthz::Mat3::cross_mat(n));
			NMat<1, 3> u_dot = NMat<1, 3>{ {u.x, u.y, u.z} };
			NMat<1, 3> w_dot = NMat<1, 3>{ {w.x, w.y, w.z} };
			NMat<1, 3> n_dot = NMat<1, 3>{ {n.x, n.y, n.z} };

			NMat<1, 5> m = n_dot * u_skew * rotDirA - u_dot * n_skew * rotDirB; //middle row
			NMat<1, 5> b = n_dot * w_skew * rotDirA - w_dot * n_skew * rotDirB; //bottom row

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					inverse_inertia.v[i][j] = ul.v[i][j];
				}
			}
			inverse_inertia.v[0][3] = um.x; inverse_inertia.v[1][3] = um.y; inverse_inertia.v[2][3] = um.z;
			inverse_inertia.v[0][4] = ur.x; inverse_inertia.v[1][4] = ur.y; inverse_inertia.v[2][4] = ur.z;
			
			for (int i = 0; i < 5; i++) {
				inverse_inertia.v[3][i] = m.v[0][i];
				inverse_inertia.v[4][i] = b.v[0][i];
			}

			inverse_inertia = inverse_inertia.inverse();
		}

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() });
		{
			mthz::Vec3 pos_correct = (hinge_pos_b - hinge_pos_a) * pos_correct_hardness;
			double u_correct = u.dot(n) * rot_correct_hardness;
			double w_correct = w.dot(n) * rot_correct_hardness;
			psuedo_target_val = NVec<5>{ pos_correct.x, pos_correct.y, pos_correct.z, u_correct, w_correct };
		}
	}

	void HingeConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
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

	NVec<5> HingeConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		mthz::Vec3 pos_value = va.lin - rA.cross(va.ang) - vb.lin + rB.cross(vb.ang);
		double u_value = n.dot(u.cross(va.ang)) + u.dot(n.cross(vb.ang));
		double w_value = n.dot(w.cross(va.ang)) + w.dot(n.cross(vb.ang));
		return NVec<5>{ pos_value.x, pos_value.y, pos_value.z, u_value, w_value };
	}

	void HingeConstraint::addVelocityChange(const NVec<5>& impulse, VelVec* va, VelVec* vb) {
		mthz::Vec3 linear_impulse_vec(impulse.v[0], impulse.v[1], impulse.v[2]);
		va->lin += linear_impulse_vec * a->getInvMass();
		vb->lin -= linear_impulse_vec * b->getInvMass();
		va->ang += NVec3toVec3(rotDirA * impulse);
		vb->ang -= NVec3toVec3(rotDirB * impulse);
	}

	//******************************
	//*****SLIDER CONSTRAINT********
	//******************************
	SliderConstraint::SliderConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slider_point_a, mthz::Vec3 slider_point_b, mthz::Vec3 slider_axis_a, mthz::Vec3 slider_axis_b, double pos_correct_hardness, double rot_correct_hardness, NVec<5> warm_start_impulse)
		: Constraint(a, b), rA(slider_point_b - a->getCOM()), rB(slider_point_b - b->getCOM()), impulse(warm_start_impulse), psuedo_impulse(NVec<5>{ 0.0, 0.0, 0.0, 0.0, 0.0 })
	{
		slider_axis_a = slider_axis_a.normalize();
		slider_axis_b = slider_axis_b.normalize();
		slider_axis_a.getPerpendicularBasis(&u, &w);

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();

		mthz::Vec3 rAxU = rA.cross(u);
		mthz::Vec3 rAxW = rA.cross(w);
		mthz::Vec3 rBxU = rB.cross(u);
		mthz::Vec3 rBxW = rB.cross(w);

		//IMPULSE TO ROTATION MATRICES
		{
			mthz::Vec3 l = Ia_inv * rAxU; //left block
			mthz::Vec3 m = Ia_inv * rAxW; //middle block
			mthz::Mat3 r = Ia_inv;        //right block

			rotDirA.v[0][0] = l.x; rotDirA.v[0][1] = m.x;
			rotDirA.v[1][0] = l.y; rotDirA.v[1][1] = m.y;
			rotDirA.v[2][0] = l.z; rotDirA.v[2][1] = m.z;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					rotDirA.v[i][j + 2] = r.v[i][j];
				}
			}
		}
		{
			mthz::Vec3 l = Ib_inv * rBxU; //left block
			mthz::Vec3 m = Ib_inv * rBxW; //middle block
			mthz::Mat3 r = Ib_inv;        //right block

			rotDirB.v[0][0] = l.x; rotDirB.v[0][1] = m.x;
			rotDirB.v[1][0] = l.y; rotDirB.v[1][1] = m.y;
			rotDirB.v[2][0] = l.z; rotDirB.v[2][1] = m.z;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					rotDirB.v[i][j + 2] = r.v[i][j];
				}
			}
		}

		//INVERSE INERTIA MATRIX
		{
			NMat<1, 3> u_dot = { u.x, u.y, u.z };
			NMat<1, 3> w_dot = { w.x, w.y, w.z };
			NMat<3, 3> rA_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rA));
			NMat<3, 3> rB_skew = Mat3toNMat33(mthz::Mat3::cross_mat(rB));
			NMat<3, 3> Ia_inv_mat = Mat3toNMat33(Ia_inv);
			NMat<3, 3> Ib_inv_mat = Mat3toNMat33(Ib_inv);

			//top row
			inverse_inertia.v[0][0] = a->getInvMass() + b->getInvMass() - u.dot(rA.cross(Ia_inv * (rAxU))) - u.dot(rB.cross(Ib_inv * (rBxU)));
			inverse_inertia.v[0][1] = -u.dot(rA.cross(Ia_inv * (rAxW))) - u.dot(rB.cross(Ib_inv * (rBxW)));
			NMat<1, 3> ur = -u_dot * rA_skew * Ia_inv_mat - u_dot * rB_skew * Ib_inv_mat;
			inverse_inertia.v[0][2] = ur.v[0][0]; inverse_inertia.v[0][3] = ur.v[0][1]; inverse_inertia.v[0][4] = ur.v[0][2];

			//middle row
			inverse_inertia.v[1][0] = -w.dot(rA.cross(Ia_inv * (rAxU))) - w.dot(rB.cross(Ib_inv * (rBxU)));
			inverse_inertia.v[1][1] = a->getInvMass() + b->getInvMass() - w.dot(rA.cross(Ia_inv * (rAxW))) - w.dot(rB.cross(Ib_inv * (rBxW)));
			NMat<1, 3> mr = -w_dot * rA_skew * Ia_inv_mat - w_dot * rB_skew * Ib_inv_mat;
			inverse_inertia.v[1][2] = mr.v[0][0]; inverse_inertia.v[1][3] = mr.v[0][1]; inverse_inertia.v[1][4] = mr.v[0][2];

			mthz::Vec3 ll = Ia_inv * rAxU + Ib_inv * rBxU;
			mthz::Vec3 lm = Ia_inv * rAxW + Ib_inv * rBxW;
			mthz::Mat3 lr = Ia_inv + Ib_inv;

			inverse_inertia.v[2][0] = ll.x; inverse_inertia.v[2][1] = lm.x;
			inverse_inertia.v[3][0] = ll.y; inverse_inertia.v[3][1] = lm.y;
			inverse_inertia.v[4][0] = ll.z; inverse_inertia.v[4][1] = lm.z;

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					inverse_inertia.v[i + 2][j + 2] = lr.v[i][j];
				}
			}

			inverse_inertia = inverse_inertia.inverse();
		}

		target_val = -getConstraintValue({ a->getVel(), a->getAngVel() }, { b->getVel(), b->getAngVel() });
		{
			double u_correct = (slider_point_b - slider_point_a).dot(u) * pos_correct_hardness;
			double w_correct = (slider_point_b - slider_point_a).dot(w) * pos_correct_hardness;
			mthz::Vec3 rot_correct = slider_axis_a.cross(slider_axis_b) * rot_correct_hardness;
			psuedo_target_val = NVec<5>{ u_correct, w_correct, rot_correct.x, rot_correct.y, rot_correct.z };
		}
	}

	void SliderConstraint::warmStartVelocityChange(VelVec* va, VelVec* vb) {
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

	void SliderConstraint::performPGSPsuedoConstraintStep() {
		PGS_constraint_step<5>(a_psuedo_velocity_changes, b_psuedo_velocity_changes, psuedo_target_val, &psuedo_impulse,
			getConstraintValue(*a_psuedo_velocity_changes, *b_psuedo_velocity_changes), inverse_inertia,
			[](const NVec<5>& impulse) { return impulse; },
			[&](const NVec<5>& impulse, Constraint::VelVec* va, Constraint::VelVec* vb) {
				this->addVelocityChange(impulse, va, vb);
			});
	}

	NVec<5> SliderConstraint::getConstraintValue(const VelVec& va, const VelVec& vb) {
		double u_value = u.dot(va.lin) - u.dot(rA.cross(va.ang)) - u.dot(vb.lin) + u.dot(rB.cross(vb.ang));
		double w_value = w.dot(va.lin) - w.dot(rA.cross(va.ang)) - w.dot(vb.lin) + w.dot(rB.cross(vb.ang));
		mthz::Vec3 rot_value = va.ang - vb.ang;
		return NVec<5>{ u_value, w_value, rot_value.x, rot_value.y, rot_value.z };
	}

	void SliderConstraint::addVelocityChange(const NVec<5>& impulse, VelVec* va, VelVec* vb) {
		va->lin += (impulse.v[0] * u + impulse.v[1] * w) * a->getInvMass();
		vb->lin -= (impulse.v[0] * u + impulse.v[1] * w) * b->getInvMass();
		va->ang += NVec3toVec3(rotDirA * impulse);
		vb->ang -= NVec3toVec3(rotDirB * impulse);

		NVec<5> v = getConstraintValue(*va, *vb);
		int a = 1 + 2;
	}

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
};