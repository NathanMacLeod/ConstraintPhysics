#pragma once
#include <cinttypes>

namespace phyz {

	template <typename T>
	class Pair {
	public:
		Pair(T* t1, T* t2) {
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

		T* t1 = nullptr;
		T* t2 = nullptr;

		//for use in std::set, used by octree class
		//use of pointer values probably makes octree non-determenistic, but octree is obsolete now that I have AABB tree and thus is only kept around for fun, so whatever
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

		bool operator==(const Pair& p) const {
			return t1 == p.t1 && t2 == p.t2;
		}
	};

}