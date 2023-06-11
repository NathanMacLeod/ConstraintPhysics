#pragma once
#include "AABB.h"
#include "BroadphaseOutput.h"
#include <vector>
#include <set>

namespace phyz {
	class Octree {
	public:

		Octree(mthz::Vec3 origin, double size, double min_size)
			: min_size(min_size), root({ origin - mthz::Vec3(size / 2.0, size / 2.0, size / 2.0), origin + mthz::Vec3(size / 2.0, size / 2.0, size / 2.0) }) {}

		void insert(const RigidBody* object, const AABB& object_bounds) {
			OctElem e = { object, &object_bounds };
			insert_r(&root, e);
		}

		std::vector<Pair> getAllIntersections() const {
			std::set<Pair> pairs;
			getAllIntersections_r(&root, &pairs);
			return std::vector<Pair>(pairs.begin(), pairs.end());
		}

	private:
		struct OctElem {
			const RigidBody* t;
			const AABB* aabb;
		};

		struct OctreeNode {
			OctreeNode(AABB aabb) : aabb(aabb), center((aabb.min + aabb.max) / 2.0), is_leaf(true) {}

			~OctreeNode() {
				for (int i = 0; i < 8; i++) {
					if (children[i] != nullptr) {
						delete children[i];
					}
				}
			}

			AABB aabb;
			mthz::Vec3 center;
			std::vector<OctElem> elem;

			bool is_leaf;
			OctreeNode* children[8] = { nullptr };

			inline double size() { return aabb.max.x - aabb.min.x; };

		};

		void insert_r(OctreeNode* curr, OctElem e);

		void getAllIntersections_r(const OctreeNode* curr, std::set<Pair>* out) const;
	
		//map between the 8 subsections and indexes 0-7. L means lesser, G greater, masking XYZ
		enum Oct { LLL = 0, GLL, LGL, GGL, LLG, GLG, LGG, GGG };
		AABB getSubAABB(const OctreeNode& node, Oct whichSub) const;

		OctreeNode root;
		double min_size;
	};

}