#pragma once
#include "RigidBody.h"

namespace phyz {

	struct Manifold {
		std::vector<mthz::Vec3> points;
		mthz::Vec3 normal;
		double pen_depth;
	};

	Manifold merge_manifold(const Manifold& m1, const Manifold& m2);
	Manifold cull_manifold(const Manifold& m, int new_size);

	class PhysicsEngine {
	public:
		void timeStep();
		RigidBody* createRigidBody(const std::vector<ConvexPoly>& geometry, bool fixed=false, double density=1.0);

		mthz::Vec3 gravity;
		double step_time;
	private:
		bool resolve_collision(RigidBody* a, RigidBody* b, Manifold manifold, double restitution);
		void resolve_penetration(RigidBody* a, RigidBody* b, const Manifold& manifold, double slack = 1);
		Manifold SAT(const ConvexPoly& a, const ConvexPoly& b) const;

		int next_ID = 0;
		std::vector<RigidBody*> bodies;
	};

}