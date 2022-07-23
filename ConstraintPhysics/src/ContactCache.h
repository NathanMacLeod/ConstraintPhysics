#pragma once
#include "CollisionDetect.h"
#include "ConstraintSolver.h"
#include <unordered_map>

namespace phyz {

	class ContactCache {
	public:

		std::vector<Constraint*> getWarmedConstraints(RigidBody* a, RigidBody* b, const Manifold& m, double friction, double bounce, double pos_correct_hardness) {
			
			ContactSurface* c = nullptr;
			if (cache.find(m.magicID) != cache.end()) {
				c = cache[m.magicID];
				if (c->m.points.size() == m.points.size()) {
					c->visited = true;
					
					mthz::Vec3 u, w;
					m.normal.getPerpendicularBasis(&u, &w);
					std::vector<bool> matched(c->m.points.size(), false);
					for (int i = 0; i < m.points.size(); i++) {
						double min_dist = std::numeric_limits<double>::infinity();
						int match_i;
						for (int j = 0; j < c->m.points.size(); j++) {
							double d = (c->m.points[j] - m.points[i]).magSqrd();
							if (!matched[j] && d < min_dist) {
								min_dist = d;
								match_i = j;
							}
						}

						//printf("WARMSTART! %f %f %f\n", c->points[i].c->impulse, c->points[i].f1->impulse, c->points[i].f2->impulse);

						matched[match_i] = true;
						c->points[i].c = new ContactConstraint(a, b, m.normal, m.points[i], bounce, m.pen_depth, pos_correct_hardness, c->points[i].c->impulse);
						c->points[i].f1 = new FrictionConstraint(a, b, u, m.points[i], friction, m.points.size(), c->points[i].c, c->points[i].f1->impulse);
						c->points[i].f2 = new FrictionConstraint(a, b, w, m.points[i], friction, m.points.size(), c->points[i].c, c->points[i].f2->impulse);
					}

					c->m = m;

				}
				else {
					c = new ContactSurface(a, b, m, friction, bounce, pos_correct_hardness);
				}
			}
			else {
				//printf("NEW\n");
				c = new ContactSurface(a, b, m, friction, bounce, pos_correct_hardness);
				cache[m.magicID] = c;
			}

			std::vector<Constraint*> out(c->points.size() * 3);
			for (int i = 0; i < c->points.size(); i++) {
				out[i*3 + 0] = c->points[i].c;
				out[i*3 + 1] = c->points[i].f1;
				out[i*3 + 2] = c->points[i].f2;
			}
			return out;
		}

		void cleanUnusedContacts() {
			for (auto it = cache.begin(); it != cache.end();)
			{
				if (!it->second->visited) {
					delete it->second;
					it = cache.erase(it);
				}
				else {
					it->second->visited = false;
					it++;
				}
			}
		}

		int n_cached() {
			return cache.size();
		}

	private:
		
		struct ContactP {
			~ContactP() {
				if (c != nullptr) delete c;
				if (f1 != nullptr) delete f1;
				if (f2 != nullptr) delete f2;
			}

			ContactConstraint* c = nullptr;
			FrictionConstraint* f1 = nullptr;
			FrictionConstraint* f2 = nullptr;
		};

		struct ContactSurface {
			ContactSurface() {}
			ContactSurface(RigidBody* a, RigidBody* b, const Manifold& m, double friction, double bounce, double pos_correct_hardness)
				: m(m), points(std::vector<ContactP>(m.points.size()))
			{
				mthz::Vec3 u, w;
				m.normal.getPerpendicularBasis(&u, &w);
				for (int i = 0; i < m.points.size(); i++) {
					points[i].c = new ContactConstraint(a, b, m.normal, m.points[i], bounce, m.pen_depth, pos_correct_hardness);
					points[i].f1 = new FrictionConstraint(a, b, u, m.points[i], friction, m.points.size(), points[i].c);
					points[i].f2 = new FrictionConstraint(a, b, w, m.points[i], friction, m.points.size(), points[i].c);
				}
			}

			Manifold m;
			std::vector<ContactP> points;
			bool visited = true;
		};

		std::unordered_map<uint64_t, ContactSurface*> cache;
	};

}