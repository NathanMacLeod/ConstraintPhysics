#pragma once
#include <cinttypes>

namespace phyz {

	template <typename T>
	class Pair {
	public:
		Pair(T t1, int t1_id, T t2, int t2_id) {
			//keep ordering consistent to avoid {x, y}; {y, x} duplicates
			if (t1_id < t2_id) {
				this->t1 = t1;
				this->t2 = t2;
			}
			else {
				this->t1 = t2;
				this->t2 = t1;
			}
		}

		T t1 = nullptr;
		T t2 = nullptr;
		int t1_id = -1;
		int t2_id = -1;

		//for use in std::set, used by octree class
		bool operator<(const Pair& p) const {
			if (t1_id < p.t1_id) {
				return true;
			}
			else if (t1_id > p.t1_id) {
				return false;
			}
			else {
				return t2_id < p.t2_id;
			}
		}

		bool operator==(const Pair& p) const {
			return t1_id == p.t1_id && t2_id == p.t2_id;
		}
	};

}