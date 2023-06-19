#pragma once
#include "RigidBody.h"

namespace phyz {

	class Pair {
	public:
		Pair(RigidBody* t1, RigidBody* t2) {
			//keep ordering consistent to avoid {x, y}; {y, x} duplicates
			if (t1->getID() < t2->getID()) {
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
			if (t1->getID() < p.t1->getID()) {
				return true;
			}
			else if (t1->getID() > p.t1->getID()) {
				return false;
			}
			else {
				return t2->getID() < p.t2->getID();
			}
		}

		bool operator==(const Pair& p) const {
			return t1 == p.t1 && t2 == p.t2;
		}
	};

}