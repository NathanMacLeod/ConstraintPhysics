#pragma once
#include "RigidBody.h"

namespace phyz {

	struct Manifold {
		std::vector<mthz::Vec3> points;
		mthz::Vec3 normal;
		double pen_depth;
	};

	class PhysicsEngine {
	public:
		void timeStep();
		RigidBody* createRigidBody(const std::vector<ConvexPoly>& geometry, bool fixed=false, double density=1.0);

		mthz::Vec3 gravity;
		double step_time;
	private:
		void resolve_collision(RigidBody* a, RigidBody* b, Manifold manifold, double restitution);
		Manifold SAT(const ConvexPoly& a, const ConvexPoly& b, int max_man_size) const;

		int next_ID = 0;
		std::vector<RigidBody*> bodies;
	};

}