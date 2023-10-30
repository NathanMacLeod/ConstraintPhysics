#pragma once
#include "ConstraintSolver.h"

namespace phyz {

	class HolonomicSystem {
	public:

		class HolonomicSystem(std::vector<Constraint*> constraints);
		void computeInverse(double cfm);

	}


	};

};