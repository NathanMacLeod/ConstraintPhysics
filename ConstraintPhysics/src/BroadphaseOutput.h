#pragma once
#include "RigidBody.h"

namespace phyz {

	class Pair {
	public:
		Pair(RigidBody* t1, RigidBody* t2) {
			//keep ordering consistent to avoid {x, y}; {y, x} duplicates
			if ((int)t1 < (int)t2) {
				this->t1 = t1;
				this->t2 = t2;
			}
			else {
				this->t1 = t2;
				this->t2 = t1;
			}
		}

		RigidBody* t1 = nullptr;
		RigidBody* t2 = nullptr;

		//for use in std::set
		bool operator<(const Pair& p) const {
			if ((int)t1 < (int)p.t1) {
				return true;
			}
			else if ((int)t1 > (int)p.t1) {
				return false;
			}
			else {
				return (int)t2 < (int)p.t2;
			}
		}
	};

}