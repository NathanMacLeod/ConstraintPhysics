#pragma once
#include "RigidBody.h"
#include "CollisionDetect.h"
#include "ConstraintSolver.h"
#include "ThreadManager.h"
#include "ContactCache.h"
#include <set>
#include <functional>

namespace phyz {

	class PhysicsEngine {
	public:
		PhysicsEngine() {
			thread_manager.init(4);
		}


		void timeStep();
		RigidBody* createRigidBody(const std::vector<ConvexPoly>& geometry, bool fixed=false, double density=1.0);
		void applyVelocityChange(RigidBody* b, const mthz::Vec3& delta_vel, const mthz::Vec3& delta_ang_vel);

		int getNumBodies() { return bodies.size(); }

		mthz::Vec3 getGravity();
		void setGravity(const mthz::Vec3& v);

		double getStep_time();
		void setStep_time(double d);
		
	private:

		struct ConstraintGraphNode; 
		
		void addContact(RigidBody* b1, RigidBody* b2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness);
		void cleanExpiredConstraintsFromGraph();
		void dfsVisitAll(ConstraintGraphNode* curr, std::set<ConstraintGraphNode*>* visited, void* in, std::function<void(ConstraintGraphNode* curr, void* in)> action);
		//std::vector<ConstraintGraphNode*> getAwakeIslandFootholds();
		std::vector<std::vector<Constraint*>> sleepOrSolveIslands();
		//bool resolve_collision(RigidBody* a, RigidBody* b, Manifold manifold, double restitution);
		//void resolve_penetration(RigidBody* a, RigidBody* b, const Manifold& manifold, double slack = 1);
		double getCutoffVel(double step_time, const mthz::Vec3& gravity) { return 2 * gravity.mag() * step_time; }
		bool bodySleepy(const std::vector<RigidBody::MovementState>& body_history);
		bool readyToSleep(RigidBody* b);
		void wakeupIsland(ConstraintGraphNode* foothold);

		int next_ID = 0;
		std::vector<RigidBody*> bodies;
		std::unordered_map<int, ConstraintGraphNode*> constraint_graph_nodes;
		mthz::Vec3 gravity;
		double step_time;
		double cutoff_vel;
		int contact_life = 6;
		bool sleeping_enabled = false;
		double sleep_delay = 0.33;

		double vel_sleep_coeff = 0.1;
		double accel_sleep_coeff = 0.022;

		ThreadManager thread_manager;

		struct Contact {
			ContactConstraint contact;
			FrictionConstraint friction1;
			FrictionConstraint friction2;
			MagicID magic;
			int memory_life; //how many frames can be referenced for warm starting before is deleted
			bool is_live_contact;
		};

		struct SharedConstraintsEdge {
			SharedConstraintsEdge(ConstraintGraphNode* n1, ConstraintGraphNode* n2) : n1(n1), n2(n2), contactConstraints(std::vector<Contact*>()) {}

			ConstraintGraphNode* n1;
			ConstraintGraphNode* n2;
			std::vector<Contact*> contactConstraints;

			ConstraintGraphNode* other(ConstraintGraphNode* c) { return (c->b == n1->b ? n2 : n1); }
		};

		struct ConstraintGraphNode {
			ConstraintGraphNode(RigidBody* b) : b(b), constraints(std::vector<SharedConstraintsEdge*>()) {}

			RigidBody* b;
			std::vector<SharedConstraintsEdge*> constraints;

			SharedConstraintsEdge* getOrCreateEdgeTo(ConstraintGraphNode* n2);
			//void removeEdgeTo(ConstraintGraphNode* n2);
		};
	};

}