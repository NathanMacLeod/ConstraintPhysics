#pragma once
#include "CollisionDetect.h"
#include "ConstraintSolver.h"
#include <unordered_map>

namespace phyz {

	class ContactCache {
	public:

		struct ContactImpulses {
			double c_imp;
			double f1_imp;
			double f2_imp;
		};

		struct CacheQuery {
			ContactImpulses impulses;
			bool cache_hit;
		};

		CacheQuery checkCache(const MagicID& magic) {
			CacheQuery out;
			if (cache.find(magic) != cache.end()) {
				out.impulses = cache[magic];
				out.cache_hit = true;
			}
			else {
				out.cache_hit = false;
			}
			return out;
		}

		void CacheImpulse(const MagicID& magic, double c_imp, double f1_imp, double f2_imp) {
			cache[magic] = ContactImpulses{ c_imp, f1_imp, f2_imp };
		}

		void clear() {
			cache.clear();
		}

		int n_cached() {
			return cache.size();
		}

	private:

		std::unordered_map<MagicID, ContactImpulses> cache;
	};

}