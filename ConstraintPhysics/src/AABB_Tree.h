#pragma once
#include "AABB.h"
#include "BroadphaseOutput.h"
#include <vector>
#include <unordered_map>
#include <cassert>
#include <stack>

namespace phyz {
	class AABBTree {
	public:

		AABBTree(double aabb_margin_size) : aabb_margin_size(aabb_margin_size), root(nullptr) {}

		~AABBTree() {
			if (root == nullptr) return;

			std::vector<Node*> delete_queue{ root };

			while (!delete_queue.empty()) {
				Node* to_delete = delete_queue.back();
				delete_queue.pop_back();

				if (!to_delete->is_leaf) {
					assert(to_delete->left != nullptr && to_delete->right != nullptr);

					delete_queue.push_back(to_delete->left);
					delete_queue.push_back(to_delete->right);
				}

				delete to_delete;
			}
		}

		void add(const RigidBody* object, const AABB& object_bounds);
		void remove(const RigidBody* object);
		void update(const RigidBody* object, const AABB& updated_object_bounds);

		std::vector<Pair> getAllCollisionCandidates() const;
		std::vector<RigidBody*> raycastHitCandidates(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const;

	private:

		class Node {
		public:
			Node()
				: parent(nullptr), left(nullptr), right(nullptr), is_leaf(false), leaf_object(nullptr), volume(0) {}

			Node(const RigidBody* object, const AABB& object_aabb, double aabb_margin_size);

			void setAABB(const AABB& aabb, double aabb_volume = -1) {
				node_aabb = aabb;
				if (aabb_volume == -1) {
					volume = AABB::volume(aabb);
				}
				else {
					volume = aabb_volume;
					assert(aabb_volume == AABB::volume(aabb));
				}
			}

			void calculateAndUpdateAABB() {
				//there is no reason this function should be used with leaf nodes. Probably implies a bug
				assert(!is_leaf);
				//non-leaf nodes should always have two children
				assert(left != nullptr && right != nullptr);

				setAABB(AABB::combine(left->node_aabb, right->node_aabb));
			}

			static void swpNodes(Node* n1, Node* n2) {
				Node* n1_og_parent = n1->parent;
				Node* n2_og_parent = n2->parent;
				assert(n1_og_parent != nullptr && n2_og_parent != nullptr);

				Node** n1_child_slot = (n1_og_parent->left == n1) ? &n1_og_parent->left : &n1_og_parent->right;
				Node** n2_child_slot = (n2_og_parent->left == n2) ? &n2_og_parent->left : &n2_og_parent->right;

				*n1_child_slot = n2;
				*n2_child_slot = n1;
				n1->parent = n2_og_parent;
				n2->parent = n1_og_parent;
			}

			Node* parent;
			Node* left;
			Node* right;

			//in case of non-leaf is the merged bounding box of the two children nodes
			//in case of leaf node, is the enlarged bounding box of the object
			AABB node_aabb;
			double volume;

			//for use in checking all collisions
			bool internal_collision_checked_flag = false;

			bool is_leaf;
			const RigidBody* leaf_object;
			AABB leaf_object_true_aabb;
		};

		//structs used in some functions
		struct InsertionCandidate {
			Node* node;
			double inherited_cost;
		};

		struct TreePair {
			Node* n1;
			Node* n2;
		};

		double aabb_margin_size;
		std::unordered_map<unsigned int, Node*> object_node_map;
		Node* root;

		void haveAncestorsRecalculateAABB(Node* altered_node) {
			Node* current = altered_node->parent;
			while (current != nullptr) {
				current->calculateAndUpdateAABB();
				current = current->parent;
			}
		}

		void rebalanceAncestors(Node* altered_leaf);
	};
}