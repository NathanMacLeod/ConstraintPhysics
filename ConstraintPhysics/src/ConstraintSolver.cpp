#include "ConstraintSolver.h"
#include "PhysicsEngine.h"
#include <unordered_map>
#include <cassert>

#define NDEBUG

#include <chrono>

namespace phyz {

	template<int n>
	static mthz::NMat<n, n> applyCFM(const mthz::NMat<n, n>& mat, double cfm);

	static mthz::NVec<6> velAngToNVec(mthz::Vec3 vel, mthz::Vec3 ang_vel);

	template<int n>
	static void PGSConstraintStep(DegreedConstraint<n>* constraint, bool use_psuedo_values) {
		mthz::NVec<6>* vel_a_change, *vel_b_change;
		mthz::NVec<n>* accumulated_impulse;
		mthz::NVec<n> target_val;
		if (use_psuedo_values) {
			vel_a_change = constraint->a_psuedo_velocity_change;
			vel_b_change = constraint->b_psuedo_velocity_change;
			accumulated_impulse = &constraint->psuedo_impulse;
			target_val = constraint->psuedo_target_val;
		}
		else {
			vel_a_change = constraint->a_velocity_change;
			vel_b_change = constraint->b_velocity_change;
			accumulated_impulse = &constraint->impulse;
			target_val = constraint->target_val;
		}

		mthz::NVec<n> current_val = constraint->getConstraintValue(*vel_a_change, *vel_b_change);
		mthz::NVec<n> delta = target_val - current_val;
		//printf("%f\n", delta.mag());
		//Apply projection on the accumulation, not the delta, to allow reversing overcorrection.
		mthz::NVec<n> impulse_new = (*accumulated_impulse) + constraint->impulse_to_value_inverse * delta;
		mthz::NVec<n> impulse_diff = constraint->isInequalityConstraint()? constraint->projectValidImpulse(impulse_new) - *accumulated_impulse
																   : impulse_new - *accumulated_impulse;
		if (!impulse_diff.isZero()) {
			constraint->computeAndApplyVelocityChange(impulse_diff, vel_a_change, vel_b_change);
			*accumulated_impulse += impulse_diff;
		}
	}

	static void constraintStep(Constraint* c, bool pos_correct) {
		switch (c->getDegree()) {
		case 1: PGSConstraintStep<1>((DegreedConstraint<1>*)c, pos_correct); break;
		case 2: PGSConstraintStep<2>((DegreedConstraint<2>*)c, pos_correct); break;
		case 3: PGSConstraintStep<3>((DegreedConstraint<3>*)c, pos_correct); break;
		case 4: PGSConstraintStep<4>((DegreedConstraint<4>*)c, pos_correct); break;
		case 5: PGSConstraintStep<5>((DegreedConstraint<5>*)c, pos_correct); break;
		case 6: PGSConstraintStep<6>((DegreedConstraint<6>*)c, pos_correct); break;
		}
	}

	static int get_total_island_degree(const std::vector<Constraint*>& constraints) {
		int sum = 0;
		for (Constraint* c : constraints) sum += c->getDegree();
		return sum;
	}

	template<int n>
	static void add_impulses(std::vector<double>* add_to, DegreedConstraint<n>* c, int indx, bool psuedo_velocity) {
		for (int i = 0; i < n; i++) {
			if (psuedo_velocity) add_to->at(indx + i) = c->psuedo_impulse.v[i];
			else				 add_to->at(indx + i) = c->impulse.v[i];
		}
	}

	static void write_all_impulses(const std::vector<Constraint*>& constraints, std::vector<double>* out, bool psuedo_velocity) {
		assert(out->size() == get_total_island_degree(constraints));
		int indx = 0;

		for (Constraint* c : constraints) {
			switch (c->getDegree()) {
			case 1: add_impulses(out, (DegreedConstraint<1>*)c, indx, psuedo_velocity); indx += 1; break;
			case 2: add_impulses(out, (DegreedConstraint<2>*)c, indx, psuedo_velocity); indx += 2; break;
			case 3: add_impulses(out, (DegreedConstraint<3>*)c, indx, psuedo_velocity); indx += 3; break;
			case 4: add_impulses(out, (DegreedConstraint<4>*)c, indx, psuedo_velocity); indx += 4; break;
			case 5: add_impulses(out, (DegreedConstraint<5>*)c, indx, psuedo_velocity); indx += 5; break;
			case 6: add_impulses(out, (DegreedConstraint<6>*)c, indx, psuedo_velocity); indx += 6; break;
			}
		}
	}

	static bool checkIfConverged(const std::vector<Constraint*>& constraints, const std::vector<double>& old_impulses, std::vector<double>* new_impulses, bool psuedo_velocity) {
		write_all_impulses(constraints, new_impulses, psuedo_velocity);

		double total_delta = 0;
		for (int i = 0; i < old_impulses.size(); i++) {
			double di = new_impulses->at(i) - old_impulses[i];
			total_delta += di * di;
		}
		return total_delta == 0;
	}

	//Projected Gauss-Seidel solver, see Iterative Dynamics with Temporal Coherence by Erin Catto 
	//the first third of this video explains it pretty well: https://www.youtube.com/watch?v=P-WP1yMOkc4 (Improving an Iterative Physics Solver Using a Direct Method)
	void PGS_solve(PhysicsEngine* pEngine, const std::vector<Constraint*>& constraints, const std::vector<HolonomicSystem*>& holonomic_systems, double holonomic_block_solver_CFM, int n_itr_vel, int n_itr_pos, int n_itr_holonomic, ThreadManager::JobStatus* compute_inverse_status) {
		auto t0 = std::chrono::system_clock::now();
		
		struct VelPair {
			VelPair() : velocity_change({0.0}), psuedo_vel_change({0.0}) {} //initialize zeroed out
			mthz::NVec<6> velocity_change;
			mthz::NVec<6> psuedo_vel_change;
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
			c->a_velocity_change = &vA->velocity_change;
			c->a_psuedo_velocity_change = &vA->psuedo_vel_change;
			c->b_velocity_change = &vB->velocity_change;
			c->b_psuedo_velocity_change = &vB->psuedo_vel_change;
		}

		//apply warm starting
		for (Constraint* c : constraints) {
			if (c->constraintWarmStarted()) {
				c->applyWarmStartVelocityChange();
			}
		}

		//really need to refactor this into a class
		int system_degree = get_total_island_degree(constraints);
		std::vector<double> impulse_val_buff1(system_degree);
		std::vector<double> impulse_val_buff2(system_degree);
		std::vector<double> psuedo_impulse_val_buff1(system_degree);
		std::vector<double> psuedo_impulse_val_buff2(system_degree);

		std::vector<double>* old_impulse_val_buff = &impulse_val_buff1;
		std::vector<double>* new_impulse_val_buff = &impulse_val_buff2;
		std::vector<double>* old_psuedo_impulse_val_buff = &psuedo_impulse_val_buff1;
		std::vector<double>* new_psuedo_impulse_val_buff = &psuedo_impulse_val_buff2;

		write_all_impulses(constraints, old_impulse_val_buff, false);
		write_all_impulses(constraints, old_psuedo_impulse_val_buff, true);

		for (int i = 0; i < n_itr_vel; i++) {
			for (Constraint* c : constraints) {
				constraintStep(c, false);
			}

			bool converged = checkIfConverged(constraints, *old_impulse_val_buff, new_impulse_val_buff, false);
			std::vector<double>* tmp = old_impulse_val_buff;
			old_impulse_val_buff = new_impulse_val_buff;
			new_impulse_val_buff = tmp;

			if (converged) break;
		}

		for (int i = 0; i < n_itr_pos; i++) {
			for (Constraint* c : constraints) {
				if (c->needsPosCorrect()) constraintStep(c, true);
			}

			bool converged = checkIfConverged(constraints, *old_psuedo_impulse_val_buff, new_psuedo_impulse_val_buff, true);
			std::vector<double>* tmp = old_psuedo_impulse_val_buff;
			old_psuedo_impulse_val_buff = new_psuedo_impulse_val_buff;
			new_psuedo_impulse_val_buff = tmp;

			if (converged) break;
		}

		auto t1 = std::chrono::system_clock::now();
		//printf("PGS done. Took %f milliseconds\n", 1000 * std::chrono::duration<float>(t1 - t0).count());

		if (holonomic_systems.size() > 0) {
			bool holonomic_inverse_computed_in_parallel = compute_inverse_status != nullptr;
			//If holonomic inverse is being computed in another thread, don't continue until it has finished
			if (holonomic_inverse_computed_in_parallel) {
				auto wait0 = std::chrono::system_clock::now();
				compute_inverse_status->waitUntilDone();
				auto wait1 = std::chrono::system_clock::now();
				//printf("Waited %f milliseconds\n", 1000 * std::chrono::duration<float>(wait1 - wait0).count());
			}
			else {
				for (HolonomicSystem* h : holonomic_systems) {
					h->computeInverse(holonomic_block_solver_CFM);
				}
			}

			//apply block solver solutions for holonomic systems
			for (int i = 0; i < n_itr_holonomic; i++) {
				for (HolonomicSystem* h : holonomic_systems) {
					h->computeAndApplyImpulses(true);
					h->computeAndApplyImpulses(false);
				}

				for (Constraint* c : constraints) {
					if (c->is_in_holonomic_system) continue;

					if (c->needsPosCorrect()) constraintStep(c, true);
					constraintStep(c, false);
				}
			}
		}

		for (const auto& kv_pair : velocity_changes) {
			RigidBody* b = kv_pair.first;
			VelPair* deltaV = kv_pair.second;

			mthz::Vec3 linear_change(deltaV->velocity_change.v[0], deltaV->velocity_change.v[1], deltaV->velocity_change.v[2]);
			mthz::Vec3 angular_change(deltaV->velocity_change.v[3], deltaV->velocity_change.v[4], deltaV->velocity_change.v[5]);
			mthz::Vec3 psuedo_linear_change(deltaV->psuedo_vel_change.v[0], deltaV->psuedo_vel_change.v[1], deltaV->psuedo_vel_change.v[2]);
			mthz::Vec3 psuedo_angular_change(deltaV->psuedo_vel_change.v[3], deltaV->psuedo_vel_change.v[4], deltaV->psuedo_vel_change.v[5]);

			pEngine->applyVelocityChange(b, linear_change, angular_change, psuedo_linear_change, psuedo_angular_change);
			delete deltaV;
		}
	}
	
	
	mthz::NMat<6, 6> Constraint::aInvMass() {
		mthz::NMat<6, 6> out;
		mthz::NMat<3, 3> force_inv = mthz::idenMat<3>() * a->getInvMass();
		mthz::NMat<3, 3> torque_inv = a->getInvTensor().toNMat();

		out.copyInto(force_inv, 0, 0);
		out.copyInto(torque_inv, 3, 3);

		return out;
	}
	mthz::NMat<6, 6> Constraint::bInvMass() {
		mthz::NMat<6, 6> out;
		mthz::NMat<3, 3> force_inv = mthz::idenMat<3>() * b->getInvMass();
		mthz::NMat<3, 3> torque_inv = b->getInvTensor().toNMat();

		out.copyInto(force_inv, 0, 0);
		out.copyInto(torque_inv, 3, 3);

		return out;
	}

	//******************************
	//*****CONTACT CONSTRAINT*******
	//******************************
	ContactConstraint::ContactConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double bounce, double pen_depth, double pos_correct_hardness, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse, double cutoff_vel)
		: DegreedConstraint<1>(a, b, warm_start_impulse), norm(norm), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM())
	{
		rotDirA = a->getInvTensor() * norm.cross(rA);
		rotDirB = b->getInvTensor() * norm.cross(rB);
		{
			mthz::NMat<3, 3> rA_skew = mthz::crossMat(rA);
			mthz::NMat<3, 3> rB_skew = mthz::crossMat(rB);
			mthz::NMat<1, 3> n_dot = mthz::NMat<1, 3>{ {norm.x, norm.y, norm.z} };

			a_jacobian.copyInto(-n_dot, 0, 0);
			a_jacobian.copyInto(n_dot * rA_skew, 0, 3);
			b_jacobian.copyInto(n_dot, 0, 0);
			b_jacobian.copyInto(-n_dot * rB_skew, 0, 3);

			impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
			impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();
		}
		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		double current_val = getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel())).v[0];
		target_val = mthz::NVec<1>{ (current_val < -cutoff_vel) ? -(1 + bounce) * current_val : -current_val };
		psuedo_target_val = mthz::NVec<1>{ pen_depth * pos_correct_hardness };
	}

	mthz::NVec<1> ContactConstraint::projectValidImpulse(mthz::NVec<1> impulse) {
		return mthz::NVec<1>{ std::max<double>(impulse.v[0], 0) };
	}

	//******************************
	//*****FRICTION CONSTRAINT******
	//******************************
	FrictionConstraint::FrictionConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 norm, mthz::Vec3 contact_p, double coeff_friction, ContactConstraint* normal, double constraint_force_mixing, mthz::NVec<2> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w, double normal_impulse_limit)
		: DegreedConstraint<2>(a, b, warm_start_impulse), rA(contact_p - a->getCOM()), rB(contact_p - b->getCOM()),
		coeff_friction(coeff_friction), normal_impulse(&normal->impulse), static_ready(false), normal_impulse_limit(normal_impulse_limit)
	{
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
			mthz::NMat<1, 3> u_dot = { {u.x, u.y, u.z} };
			mthz::NMat<1, 3> w_dot = { {w.x, w.y, w.z} };
			mthz::NMat<3, 3> rA_skew = mthz::crossMat(rA);
			mthz::NMat<3, 3> rB_skew = mthz::crossMat(rB);

			a_jacobian.copyInto(u_dot, 0, 0);
			a_jacobian.copyInto(-u_dot * rA_skew, 0, 3);
			b_jacobian.copyInto(-u_dot, 0, 0);
			b_jacobian.copyInto(u_dot * rB_skew, 0, 3);

			a_jacobian.copyInto(w_dot, 1, 0);
			a_jacobian.copyInto(-w_dot * rA_skew, 1, 3);
			b_jacobian.copyInto(-w_dot, 1, 0);
			b_jacobian.copyInto(w_dot * rB_skew, 1, 3);

			impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
			impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();
		}

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		target_val = -getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel()));
	}

	mthz::NVec<2> FrictionConstraint::projectValidImpulse(mthz::NVec<2> impulse) {
		double max_impulse_mag = coeff_friction * std::min<double>(normal_impulse_limit, normal_impulse->v[0]);
		double current_impulse_mag = sqrt(impulse.v[0] * impulse.v[0] + impulse.v[1] * impulse.v[1]);
		if (current_impulse_mag > max_impulse_mag) {
			static_ready = false;
			double r = max_impulse_mag / current_impulse_mag;
			return mthz::NVec<2>{ impulse.v[0] * r, impulse.v[1] * r };
		}
		else {
			static_ready = true;
			return impulse;
		}
	}

	//******************************
	//*****DISTANCE CONSTRAINT*****
	//******************************
	DistanceConstraint::DistanceConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 attach_pos_a, mthz::Vec3 attach_pos_b, double target_distance, double pos_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<1> warm_start_impulse)
		: DegreedConstraint<1>(a, b, warm_start_impulse), rA(attach_pos_a - a->getCOM()), rB(attach_pos_b - b->getCOM())
	{
		this->is_in_holonomic_system = is_in_holonomic_system;
		//mthz::Mat3 inverse_inertia_mat3 = (mthz::Mat3::iden()*a->getInvMass() + mthz::Mat3::iden()*b->getInvMass() - mthz::Mat3::cross_mat(rA)*rotDirA - mthz::Mat3::cross_mat(rB)*rotDirB);
		//inverse_inertia = applyCFM(Mat3tomthz::NMat33(inverse_inertia_mat3), constraint_force_mixing).inverse();

		mthz::Vec3 diff = attach_pos_a - attach_pos_b;
		mthz::NMat<1, 3> diff_dot = { diff.x, diff.y, diff.z };
		mthz::NMat<3, 3> rA_skew = mthz::crossMat(rA);
		mthz::NMat<3, 3> rB_skew = mthz::crossMat(rB);

		a_jacobian.copyInto(diff_dot, 0, 0);
		a_jacobian.copyInto(-diff_dot * rA_skew, 0, 3);
		b_jacobian.copyInto(-diff_dot, 0, 0);
		b_jacobian.copyInto(diff_dot * rB_skew, 0, 3);

		impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
		impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();
		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;

		target_val = -getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel()));
		double error = diff.magSqrd() < 0.00001? target_distance : target_distance - diff.normalize().dot(diff);
		psuedo_target_val = mthz::NVec<1>{ pos_correct_hardness * error };
	}

	//******************************
	//****BALL SOCKET CONSTRAINT****
	//******************************
	BallSocketConstraint::BallSocketConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 socket_pos_a, mthz::Vec3 socket_pos_b, double pos_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<3> warm_start_impulse)
		: DegreedConstraint<3>(a, b, warm_start_impulse), rA(socket_pos_a - a->getCOM()), rB(socket_pos_a - b->getCOM()), rotDirA(a->getInvTensor() * mthz::Mat3::cross_mat(rA)), rotDirB(b->getInvTensor() * mthz::Mat3::cross_mat(rB))
	{
		this->is_in_holonomic_system = is_in_holonomic_system;
		//mthz::Mat3 inverse_inertia_mat3 = (mthz::Mat3::iden()*a->getInvMass() + mthz::Mat3::iden()*b->getInvMass() - mthz::Mat3::cross_mat(rA)*rotDirA - mthz::Mat3::cross_mat(rB)*rotDirB);
		//inverse_inertia = applyCFM(Mat3tomthz::NMat33(inverse_inertia_mat3), constraint_force_mixing).inverse();

		mthz::NMat<3, 3> rA_skew = mthz::crossMat(rA);
		mthz::NMat<3, 3> rB_skew = mthz::crossMat(rB);

		a_jacobian.copyInto(mthz::idenMat<3>(), 0, 0);
		a_jacobian.copyInto(-rA_skew, 0, 3);
		b_jacobian.copyInto(-mthz::idenMat<3>(), 0, 0);
		b_jacobian.copyInto(rB_skew, 0, 3);
		
		impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
		impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		mthz::Vec3 ap_vel = a->getVelOfPoint(socket_pos_a);
		mthz::Vec3 bp_vel = b->getVelOfPoint(socket_pos_a);
		mthz::Vec3 vel_diff = ap_vel - bp_vel;

		target_val = -getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel()));
		mthz::Vec3 error = socket_pos_b - socket_pos_a;
		psuedo_target_val = mthz::NVec<3>{ pos_correct_hardness * error.x, pos_correct_hardness * error.y, pos_correct_hardness * error.z };
	}

	

	//******************************
	//*****HINGE CONSTRAINT*********
	//******************************
	HingeConstraint::HingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 hinge_pos_a, mthz::Vec3 hinge_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<5> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w)
		: DegreedConstraint<5>(a, b, warm_start_impulse), rA(hinge_pos_a - a->getCOM()), rB(hinge_pos_a - b->getCOM())
	{
		this->is_in_holonomic_system = is_in_holonomic_system;
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
			mthz::NMat<3, 3> l = (Ia_inv * rA_skew).toNMat(); //left block
			mthz::Vec3 m = Ia_inv * (n.cross(u));          //middle block
			mthz::Vec3 r = Ia_inv * (n.cross(w));          //right block

			rotDirA.copyInto(l, 0, 0);
			rotDirA.v[0][3] = m.x; rotDirA.v[1][3] = m.y; rotDirA.v[2][3] = m.z;
			rotDirA.v[0][4] = r.x; rotDirA.v[1][4] = r.y; rotDirA.v[2][4] = r.z;
		}
		{
			mthz::NMat<3, 3> l = (Ib_inv * rB_skew).toNMat(); //left block
			mthz::Vec3 m = Ib_inv * (n.cross(u));          //middle block
			mthz::Vec3 r = Ib_inv * (n.cross(w));          //right block

			rotDirB.copyInto(l, 0, 0);
			rotDirB.v[0][3] = m.x; rotDirB.v[1][3] = m.y; rotDirB.v[2][3] = m.z;
			rotDirB.v[0][4] = r.x; rotDirB.v[1][4] = r.y; rotDirB.v[2][4] = r.z;
		}
		
		//INVERSE INERTIA MATRIX
		{
			mthz::NMat<3, 3> ul = (mthz::Mat3::iden() * a->getInvMass() + mthz::Mat3::iden() * b->getInvMass() - rA_skew * Ia_inv * rA_skew - rB_skew * Ib_inv * rB_skew).toNMat(); //upper left
			mthz::Vec3 um = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(u)); //upper middle
			mthz::Vec3 ur = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(w)); //upper right

			mthz::NMat<3, 3> rA_skew_mat = rA_skew.toNMat();
			mthz::NMat<3, 3> rB_skew_mat = rB_skew.toNMat();
			mthz::NMat<3, 3> u_skew = mthz::crossMat(u);
			mthz::NMat<3, 3> w_skew = mthz::crossMat(w);
			mthz::NMat<3, 3> n_skew = mthz::crossMat(n);
			mthz::NMat<1, 3> u_dot = mthz::NMat<1, 3>{ {u.x, u.y, u.z} };
			mthz::NMat<1, 3> w_dot = mthz::NMat<1, 3>{ {w.x, w.y, w.z} };
			mthz::NMat<1, 3> n_dot = mthz::NMat<1, 3>{ {n.x, n.y, n.z} };
			mthz::NMat<1, 3> zero = mthz::NMat<1, 3>{ {0, 0, 0} };

			mthz::NMat<1, 5> m = n_dot * u_skew * rotDirA - u_dot * n_skew * rotDirB; //middle row
			mthz::NMat<1, 5> b = n_dot * w_skew * rotDirA - w_dot * n_skew * rotDirB; //bottom row

			a_jacobian.copyInto(mthz::idenMat<3>(), 0, 0);
			a_jacobian.copyInto(-rA_skew_mat, 0, 3);
			b_jacobian.copyInto(-mthz::idenMat<3>(), 0, 0);
			b_jacobian.copyInto(rB_skew_mat, 0, 3);
			
			a_jacobian.copyInto(zero, 3, 0);
			a_jacobian.copyInto(n_dot * u_skew, 3, 3);
			b_jacobian.copyInto(zero, 3, 0);
			b_jacobian.copyInto(u_dot * n_skew, 3, 3);
			a_jacobian.copyInto(zero, 4, 0);
			a_jacobian.copyInto(n_dot * w_skew, 4, 3);
			b_jacobian.copyInto(zero, 4, 0);
			b_jacobian.copyInto(w_dot * n_skew, 4, 3);
		}

		impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
		impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		target_val = -getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel()));
		{
			mthz::Vec3 pos_correct = (hinge_pos_b - hinge_pos_a) * pos_correct_hardness;
			double u_correct = u.dot(n)* rot_correct_hardness;
			double w_correct = w.dot(n)* rot_correct_hardness;
			psuedo_target_val = mthz::NVec<5>{ pos_correct.x, pos_correct.y, pos_correct.z, u_correct, w_correct };
		}
	}

	//******************************
	//*****MOTOR CONSTRAINT*********
	//******************************
	MotorConstraint::MotorConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 motor_axis, double target_velocity, double max_torque_impulse, double current_angle, double min_angle, double max_angle, double rot_correct_hardness, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse)
		: DegreedConstraint<1>(a, b, warm_start_impulse), motor_axis(motor_axis), max_torque_impulse(max_torque_impulse)
	{
		double real_target_velocity;

		if (current_angle > max_angle) {
			psuedo_target_val = mthz::NVec<1>{ (max_angle - current_angle) * rot_correct_hardness };
			real_target_velocity = std::min<double>(0, target_velocity);
			rot_limit_status = ABOVE_MAX;
		}
		else if (current_angle < min_angle) {
			psuedo_target_val = mthz::NVec<1>{ (min_angle - current_angle) * rot_correct_hardness };
			real_target_velocity = std::max<double>(0, target_velocity);
			rot_limit_status = BELOW_MIN;
		}
		else {
			psuedo_target_val = mthz::NVec<1>{ 0 };
			real_target_velocity = target_velocity;
			rot_limit_status = NOT_EXCEEDED;
		}

		mthz::NMat<1, 3> motor_axis_dot = { motor_axis.x, motor_axis.y, motor_axis.z };

		a_jacobian.copyInto(motor_axis_dot, 0, 3);
		b_jacobian.copyInto(-motor_axis_dot, 0, 3);

		impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
		impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		double current_val = getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel())).v[0];

		target_val = mthz::NVec<1>{ real_target_velocity - current_val };
	}

	mthz::NVec<1> MotorConstraint::projectValidImpulse(mthz::NVec<1> impulse) {
		switch (rot_limit_status) {
		case NOT_EXCEEDED:
			if (impulse.v[0] > 0)
				return mthz::NVec<1>{ std::min<double>(impulse.v[0], max_torque_impulse) };
			else
				return mthz::NVec<1>{ std::max<double>(impulse.v[0], -max_torque_impulse) };
		case ABOVE_MAX:
			return mthz::NVec<1>{ std::min<double>(impulse.v[0], 0) };
		case BELOW_MIN:
			return mthz::NVec<1>{ std::max<double>(impulse.v[0], 0) };
		}
	}

	////******************************
	////*****SLIDER CONSTRAINT********
	////******************************
	SliderConstraint::SliderConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slider_point_a, mthz::Vec3 slider_point_b, mthz::Vec3 slider_axis_a, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<5> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w)
		: DegreedConstraint<5>(a, b, warm_start_impulse), rA(slider_point_a - a->getCOM()), rB(slider_point_b - b->getCOM())
	{
		this->is_in_holonomic_system = is_in_holonomic_system;
		slider_axis_a = slider_axis_a.normalize();
		slider_axis_a.getPerpendicularBasis(&u, &w);

		impulse.v[0] = u.dot(source_u) * warm_start_impulse.v[0] + u.dot(source_w) * warm_start_impulse.v[1];
		impulse.v[1] = w.dot(source_u) * warm_start_impulse.v[0] + w.dot(source_w) * warm_start_impulse.v[1];

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();
		mthz::NMat<3, 3> Ia_inv_mat = Ia_inv.toNMat();
		mthz::NMat<3, 3> Ib_inv_mat = Ib_inv.toNMat();

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
			mthz::NMat<1, 3> u_dot = { u.x, u.y, u.z };
			mthz::NMat<1, 3> w_dot = { w.x, w.y, w.z };
			mthz::NMat<1, 3> Pe_dot = { pos_error.x, pos_error.y, pos_error.z };
			mthz::NMat<3, 3> rA_skew = mthz::crossMat(rA);
			mthz::NMat<3, 3> rB_skew = mthz::crossMat(rB);
			mthz::NMat<3, 3> u_skew = mthz::crossMat(u);
			mthz::NMat<3, 3> w_skew = mthz::crossMat(w);

			a_jacobian.copyInto(u_dot, 0, 0);
			a_jacobian.copyInto(-u_dot * rA_skew - Pe_dot * u_skew, 0, 3);
			b_jacobian.copyInto(-u_dot, 0, 0);
			b_jacobian.copyInto(u_dot * rB_skew, 0, 3);

			a_jacobian.copyInto(w_dot, 1, 0);
			a_jacobian.copyInto(-w_dot * rA_skew - Pe_dot * w_skew, 1, 3);
			b_jacobian.copyInto(-w_dot, 1, 0);
			b_jacobian.copyInto(w_dot * rB_skew, 1, 3);

			a_jacobian.copyInto(mthz::idenMat<3>(), 2, 3);
			b_jacobian.copyInto(-mthz::idenMat<3>(), 2, 3);

			impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
			impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();
			
			impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
			impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();
		}

		target_val = -getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel()));
		{
			double u_correct = -pos_error.dot(u) * pos_correct_hardness;
			double w_correct = -pos_error.dot(w) * pos_correct_hardness;

			mthz::Quaternion a_orient = a->getOrientation();
			mthz::Quaternion b_orient = b->getOrientation();
			mthz::Quaternion orientation_diff = b_orient * a_orient.conjugate();
			double angle = a_orient.angleTo(b_orient);
			mthz::Vec3 axis = angle == 0 ? mthz::Vec3() : orientation_diff.getRotAxis() * rot_correct_hardness;
			mthz::Vec3 rot_correct = angle * axis;
			psuedo_target_val = mthz::NVec<5>{ u_correct, w_correct, rot_correct.x, rot_correct.y, rot_correct.z };
		}
	}

	//******************************
	//******PISTON CONSTRAINT*******
	//******************************
	PistonConstraint::PistonConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis_a, double target_velocity, double max_impulse, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse)
		: DegreedConstraint<1>(a, b, warm_start_impulse), slide_axis(slide_axis_a), max_impulse(max_impulse)
	{
		mthz::Mat3 Ia_inv = a->getInvTensor();

		pos_diff = b->getCOM() - a->getCOM();
		rot_dir = Ia_inv * (pos_diff.cross(slide_axis_a));

		mthz::NMat<1, 3> slide_axis_dot = { slide_axis.x, slide_axis.y, slide_axis.z };
		mthz::NMat<3, 3> slide_axis_cross = mthz::crossMat(slide_axis);

		a_jacobian.copyInto(-slide_axis_dot, 0, 0);
		a_jacobian.copyInto(-slide_axis_dot * slide_axis_cross, 0, 3);
		b_jacobian.copyInto(slide_axis_dot, 0, 0);

		impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
		impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		double current_val = getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel())).v[0];
		target_val = mthz::NVec<1>{ target_velocity - current_val };
	}

	mthz::NVec<1> PistonConstraint::projectValidImpulse(mthz::NVec<1> impulse) {
		if (impulse.v[0] > 0)	return mthz::NVec<1>{ std::min<double>(impulse.v[0], max_impulse) };
		else					return mthz::NVec<1>{ std::max<double>(impulse.v[0], -max_impulse) };
	}

	//******************************
	//****Slide Limit Constraint****
	//******************************
	SlideLimitConstraint::SlideLimitConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_axis_a, double slide_position, double positive_slide_limit, double negative_slide_limit, double pos_correct_hardness, double constraint_force_mixing, mthz::NVec<1> warm_start_impulse)
		: DegreedConstraint<1>(a, b, warm_start_impulse), slide_axis(slide_axis_a)
	{

		if (slide_position > positive_slide_limit) {
			psuedo_target_val = mthz::NVec<1>{ (positive_slide_limit - slide_position) * pos_correct_hardness };
			slide_limit_status = ABOVE_MAX;
		}
		else if (slide_position < negative_slide_limit) {
			psuedo_target_val = mthz::NVec<1>{ (negative_slide_limit - slide_position) * pos_correct_hardness };
			slide_limit_status = BELOW_MIN;
		}

		mthz::Mat3 Ia_inv = a->getInvTensor();

		pos_diff = b->getCOM() - a->getCOM();
		rot_dir = Ia_inv * (pos_diff.cross(slide_axis_a));

		mthz::NMat<1, 3> slide_axis_dot = { slide_axis.x, slide_axis.y, slide_axis.z };
		mthz::NMat<3, 3> slide_axis_cross = mthz::crossMat(slide_axis);

		a_jacobian.copyInto(-slide_axis_dot, 0, 0);
		a_jacobian.copyInto(-slide_axis_dot * slide_axis_cross, 0, 3);
		b_jacobian.copyInto(slide_axis_dot, 0, 0);

		impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
		impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		double current_val = getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel())).v[0];
		target_val = mthz::NVec<1>{ -current_val };
	}

	mthz::NVec<1> SlideLimitConstraint::projectValidImpulse(mthz::NVec<1> impulse) {
		switch (slide_limit_status) {
		case ABOVE_MAX: return mthz::NVec<1>{ std::min<double>(impulse.v[0], 0)};
		case BELOW_MIN: return mthz::NVec<1>{ std::max<double>(impulse.v[0], 0)};
		}
	}

	//******************************
	//***SLIDING HINGE CONSTRAINT***
	//******************************
	SlidingHingeConstraint::SlidingHingeConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 slide_pos_a, mthz::Vec3 slide_pos_b, mthz::Vec3 rot_axis_a, mthz::Vec3 rot_axis_b, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<4> warm_start_impulse, mthz::Vec3 source_u, mthz::Vec3 source_w)
		: DegreedConstraint<4>(a, b, warm_start_impulse), rA(slide_pos_a - a->getCOM()), rB(slide_pos_b - b->getCOM())
	{
		this->is_in_holonomic_system = is_in_holonomic_system;
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
			mthz::NMat<3, 3> ul = (mthz::Mat3::iden() * a->getInvMass() + mthz::Mat3::iden() * b->getInvMass() - rA_skew * Ia_inv * rA_skew - rB_skew * Ib_inv * rB_skew).toNMat(); //upper left
			mthz::Vec3 um = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(u)); //upper middle
			mthz::Vec3 ur = -(rA_skew * Ia_inv + rB_skew * Ib_inv) * (n.cross(w)); //upper right

			mthz::NMat<3, 3> rA_skew_mat = rA_skew.toNMat();
			mthz::NMat<3, 3> rB_skew_mat = rB_skew.toNMat();
			mthz::NMat<3, 3> u_skew = mthz::crossMat(u);
			mthz::NMat<3, 3> w_skew = mthz::crossMat(w);
			mthz::NMat<3, 3> n_skew = mthz::crossMat(n);
			mthz::NMat<1, 3> u_dot = mthz::NMat<1, 3>{ {u.x, u.y, u.z} };
			mthz::NMat<1, 3> w_dot = mthz::NMat<1, 3>{ {w.x, w.y, w.z} };
			mthz::NMat<1, 3> n_dot = mthz::NMat<1, 3>{ {n.x, n.y, n.z} };
			mthz::NMat<1, 3> pd_dot = mthz::NMat<1, 3>{ {pos_diff.x, pos_diff.y, pos_diff.z} };

			a_jacobian.copyInto(u_dot, 0, 0); a_jacobian.copyInto(-u_dot * rA_skew_mat, 0, 3); b_jacobian.copyInto(-u_dot, 0, 0); b_jacobian.copyInto(u_dot * rB_skew_mat - pd_dot * u_skew, 0, 3);
			a_jacobian.copyInto(w_dot, 1, 0); a_jacobian.copyInto(-w_dot * rA_skew_mat, 1, 3); b_jacobian.copyInto(-w_dot, 1, 0); b_jacobian.copyInto(w_dot * rB_skew_mat - pd_dot * w_skew, 1, 3);
											  a_jacobian.copyInto(-u_dot * n_skew, 2, 3);										  b_jacobian.copyInto(-n_dot * u_skew, 2, 3);
											  a_jacobian.copyInto(-w_dot * n_skew, 3, 3);										  b_jacobian.copyInto(-n_dot * w_skew, 3, 3);

			impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
			impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

			impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
			impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();
		}

		target_val = -getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel()));
		{
			double u_correct = -u.dot(pos_diff) * pos_correct_hardness;
			double w_correct = -w.dot(pos_diff) * pos_correct_hardness;
			double u_rot_correct = -u.dot(n) * rot_correct_hardness;
			double w_rot_correct = -w.dot(n) * rot_correct_hardness;
			psuedo_target_val = mthz::NVec<4>{ u_correct, w_correct, u_rot_correct, w_rot_correct };
		}
	}

	////******************************
	////*******WELD CONSTRAINT********
	////******************************
	WeldConstraint::WeldConstraint(RigidBody* a, RigidBody* b, mthz::Vec3 pos_a, mthz::Vec3 pos_b, double pos_correct_hardness, double rot_correct_hardness, double constraint_force_mixing, bool is_in_holonomic_system, mthz::NVec<6> warm_start_impulse)
		: DegreedConstraint<6>(a, b, warm_start_impulse), rA(pos_a - a->getCOM()), rB(pos_b - b->getCOM())
	{
		this->is_in_holonomic_system = is_in_holonomic_system;

		mthz::Mat3 Ia_inv = a->getInvTensor();
		mthz::Mat3 Ib_inv = b->getInvTensor();
		mthz::NMat<3, 3> Ia_inv_mat = Ia_inv.toNMat();
		mthz::NMat<3, 3> Ib_inv_mat = Ib_inv.toNMat();
		mthz::NMat<3, 3> rA_skew = mthz::crossMat(rA);
		mthz::NMat<3, 3> rB_skew = mthz::crossMat(rB);
		mthz::NMat<3, 3> iden_mat = mthz::idenMat<3>();

		rotDirA.copyInto(Ia_inv_mat * rA_skew, 0, 0); rotDirA.copyInto(Ia_inv_mat, 0, 3);

		rotDirB.copyInto(Ib_inv_mat * rB_skew, 0, 0); rotDirB.copyInto(Ib_inv_mat, 0, 3);

		a_jacobian.copyInto(mthz::idenMat<3>(), 0, 0); a_jacobian.copyInto(-rA_skew, 0, 3);
												 a_jacobian.copyInto(mthz::idenMat<3>(), 3, 3);
		b_jacobian.copyInto(-mthz::idenMat<3>(), 0, 0); b_jacobian.copyInto(rB_skew, 0, 3);
												  b_jacobian.copyInto(-mthz::idenMat<3>(), 3, 3);

		impulse_to_a_velocity = aInvMass() * a_jacobian.transpose();
		impulse_to_b_velocity = bInvMass() * b_jacobian.transpose();

		impulse_to_value = a_jacobian * impulse_to_a_velocity + b_jacobian * impulse_to_b_velocity;
		impulse_to_value_inverse = applyCFM(impulse_to_value, constraint_force_mixing).inverse();

		target_val = -getConstraintValue(velAngToNVec(a->getVel(), a->getAngVel()), velAngToNVec(b->getVel(), b->getAngVel()));
		{
			mthz::Vec3 pos_correct = (pos_b - pos_a) * pos_correct_hardness;

			mthz::Quaternion a_orient = a->getOrientation();
			mthz::Quaternion b_orient = b->getOrientation();
			mthz::Quaternion orientation_diff = b_orient * a_orient.conjugate();
			double angle = a_orient.angleTo(b_orient);
			mthz::Vec3 axis = angle == 0 ? mthz::Vec3() : orientation_diff.getRotAxis() * rot_correct_hardness;
			mthz::Vec3 rot_correct = angle * axis;
			psuedo_target_val = mthz::NVec<6>{ pos_correct.x, pos_correct.y, pos_correct.z, rot_correct.x, rot_correct.y, rot_correct.z };
		}
	}

	TestConstraint::TestConstraint(RigidBody* a, RigidBody* b)
		: DegreedConstraint(a, b, mthz::NVec<6>())
	{
		a_jacobian = mthz::idenMat<6>();
		b_jacobian = mthz::idenMat<6>();

		impulse_to_a_velocity = mthz::idenMat<6>();
		impulse_to_b_velocity = mthz::idenMat<6>();

		impulse_to_value = 2 * mthz::idenMat<6>();
		impulse_to_value_inverse = 0.5 * mthz::idenMat<6>();
	}

	mthz::NVec<6> velAngToNVec(mthz::Vec3 vel, mthz::Vec3 ang_vel) {
		return mthz::NVec<6> { vel.x, vel.y, vel.z, ang_vel.x, ang_vel.y, ang_vel.z };
	}

	template<int n>
	static mthz::NMat<n, n> applyCFM(const mthz::NMat<n, n>& mat, double cfm) {
		mthz::NMat<n, n> copy(mat);
		for (int i = 0; i < n; i++) {
			copy.v[i][i] *= (1 + cfm);
		}
		return copy;
	}
};