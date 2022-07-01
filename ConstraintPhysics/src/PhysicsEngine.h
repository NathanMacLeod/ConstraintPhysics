#pragma once
#include "RigidBody.h"
#include "CollisionDetect.h"

namespace phyz {

	class PhysicsEngine {
	public:
		//PhysicsEngine() { init_multithreaded_sat(); }

		void timeStep();
		RigidBody* createRigidBody(const std::vector<ConvexPoly>& geometry, bool fixed=false, double density=1.0);

		mthz::Vec3 gravity;
		double step_time;
	private:
		bool resolve_collision(RigidBody* a, RigidBody* b, Manifold manifold, double restitution);
		void resolve_penetration(RigidBody* a, RigidBody* b, const Manifold& manifold, double slack = 1);

		int next_ID = 0;
		std::vector<RigidBody*> bodies;
	};

}