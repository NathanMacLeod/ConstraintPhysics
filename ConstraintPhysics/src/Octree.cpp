#include "Octree.h"

namespace phyz {

	void Octree::insert_r(OctreeNode* curr, OctElem e) {
		//for n < 15, sorting n elems into children nodes requires more AABB checks then simply checking elems against each other
		const int LEAF_IDEAL_CAPACITY = 15;
		if (curr->is_leaf) {
			curr->elem.push_back(e);
			if (curr->elem.size() > LEAF_IDEAL_CAPACITY && curr->size() / 2.0 > min_size) {
				curr->is_leaf = false;
				for (int i = 0; i < 8; i++) {
					curr->children[i] = new OctreeNode(getSubAABB(*curr, (Oct)i));
				}
				for (OctElem elem : curr->elem) {
					insert_r(curr, elem);
				}
			}

		}
		else {
			bool intersects[8] = { true, true, true, true, true, true, true, true };
			if (e.aabb->min.x > curr->center.x) {
				intersects[LLL] = false;
				intersects[LLG] = false;
				intersects[LGL] = false;
				intersects[LGG] = false;
			}
			else if (e.aabb->max.x < curr->center.x) {
				intersects[GLL] = false;
				intersects[GLG] = false;
				intersects[GGL] = false;
				intersects[GGG] = false;
			}
			if (e.aabb->min.y > curr->center.y) {
				intersects[LLL] = false;
				intersects[LLG] = false;
				intersects[GLL] = false;
				intersects[GLG] = false;
			}
			else if (e.aabb->max.y < curr->center.y) {
				intersects[LGL] = false;
				intersects[LGG] = false;
				intersects[GGL] = false;
				intersects[GGG] = false;
			}
			if (e.aabb->min.z > curr->center.z) {
				intersects[LLL] = false;
				intersects[LGL] = false;
				intersects[GLL] = false;
				intersects[GGL] = false;
			}
			else if (e.aabb->max.z < curr->center.z) {
				intersects[LLG] = false;
				intersects[LGG] = false;
				intersects[GLG] = false;
				intersects[GGG] = false;
			}

			for (int i = 0; i < 8; i++) {
				if (intersects[i]) {
					insert_r(curr->children[i], e);
				}
			}
		}
	}

	void Octree::getAllIntersections_r(const OctreeNode* curr, std::set<Pair<RigidBody*>>* out) const {
		if (curr->is_leaf) {
			for (int i = 0; i < curr->elem.size(); i++) {
				for (int j = i + 1; j < curr->elem.size(); j++) {
					if (AABB::intersects(*curr->elem[i].aabb, *curr->elem[j].aabb)) {
						Pair<RigidBody*> p((RigidBody*)curr->elem[i].t, curr->elem[i].t->getID(), (RigidBody*)curr->elem[j].t, curr->elem[j].t->getID());
						out->insert(p);
					}
				}
			}
		}
		else {
			for (int i = 0; i < 8; i++) {
				getAllIntersections_r(curr->children[i], out);
			}
		}
	}

	AABB Octree::getSubAABB(const OctreeNode& node, Oct whichSub) const {
		mthz::Vec3 min = node.aabb.min;
		mthz::Vec3 max = node.aabb.max;
		mthz::Vec3 mid = (min + max) / 2.0;
		switch (whichSub) {
		case LLL:
			return { mthz::Vec3(min.x, min.y, min.z), mthz::Vec3(mid.x, mid.y, mid.z) };
		case GLL:
			return { mthz::Vec3(mid.x, min.y, min.z), mthz::Vec3(max.x, mid.y, mid.z) };
		case LGL:
			return { mthz::Vec3(min.x, mid.y, min.z), mthz::Vec3(mid.x, max.y, mid.z) };
		case GGL:
			return { mthz::Vec3(mid.x, mid.y, min.z), mthz::Vec3(max.x, max.y, mid.z) };
		case LLG:
			return { mthz::Vec3(min.x, min.y, mid.z), mthz::Vec3(mid.x, mid.y, max.z) };
		case GLG:
			return { mthz::Vec3(mid.x, min.y, mid.z), mthz::Vec3(max.x, mid.y, max.z) };
		case LGG:
			return { mthz::Vec3(min.x, mid.y, mid.z), mthz::Vec3(mid.x, max.y, max.z) };
		case GGG:
			return { mthz::Vec3(mid.x, mid.y, mid.z), mthz::Vec3(max.x, max.y, max.z) };
		}
	}

}