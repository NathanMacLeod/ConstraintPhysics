#pragma once
#include "AABB.h"
#include <vector>
#include <set>

template <typename T>
class Octree {
public:
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

	Octree(mthz::Vec3 origin, double size, double min_size)
		: min_size(min_size), root()
	{
		root.aabb = { origin - mthz::Vec3(size / 2.0, size / 2.0, size / 2.0), origin + mthz::Vec3(size / 2.0, size / 2.0, size / 2.0) };
	}

	void insert(const T& object, const AABB& object_bounds) {
		OctElem e = { &object, &object_bounds };
		insert_r(&root, e);
	}

	std::set<Pair> getAllIntersections() const {
		std::set<Pair> pairs;
		getAllIntersections_r(&root, &pairs);
		return pairs;
	}

private:
	struct OctElem {
		const T* t;
		const AABB* aabb;
	};

	struct OctreeNode {
		AABB aabb;
		std::vector<OctElem> elem;

		bool is_leaf = true;
		OctreeNode* children[8] = { nullptr };

		inline double size() { return aabb.max.x - aabb.min.x; };
		~OctreeNode() {
			for (int i = 0; i < 8; i++) {
				if (children[i] != nullptr) {
					delete children[i];
				}
			}
		}
	};

	void insert_r(OctreeNode* curr, OctElem e) {
		//for n < 15, sorting n elems into children nodes requires more AABB checks then simply checking elems against each other
		const int LEAF_IDEAL_CAPACITY = 15;
		if (curr->is_leaf) {
			curr->elem.push_back(e);
			if (curr->elem.size() > LEAF_IDEAL_CAPACITY && curr->size() / 2.0 > min_size) {
				curr->is_leaf = false;
				for (int i = 0; i < 8; i++) {
					curr->children[i] = new OctreeNode();
					curr->children[i]->aabb = getSubAABB(*curr, (Oct)i);
				}
				for (OctElem elem : curr->elem) {
					insert_r(curr, elem);
				}
			}

		}
		else {
			for (OctreeNode* child : curr->children) {
				if (AABB::intersects(child->aabb, *e.aabb)) {
					insert_r(child, e);
				}
			}
		}
	}

	void getAllIntersections_r(const OctreeNode* curr, std::set<Pair>* out) const {
		if (curr->is_leaf) {
			for (int i = 0; i < curr->elem.size(); i++) {
				for (int j = i + 1; j < curr->elem.size(); j++) {
					if (AABB::intersects(*curr->elem[i].aabb, *curr->elem[j].aabb)) {
						Pair p((T*) curr->elem[i].t, (T*) curr->elem[j].t);
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
	//map between the 8 subsections and indexes 0-7. L means lesser, G greater, masking XYZ
	enum Oct { LLL = 0, GLL, LGL, GGL, LLG, GLG, LGG, GGG }; 
	AABB getSubAABB(const OctreeNode& node, Oct whichSub) const {
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

	OctreeNode root;
	double min_size;
};