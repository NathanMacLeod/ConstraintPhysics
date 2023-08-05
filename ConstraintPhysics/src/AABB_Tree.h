#pragma once
#include "AABB.h"
#include "BroadphaseOutput.h"
#include <vector>
#include <unordered_map>
#include <cassert>
#include <stack>

namespace phyz {

	template <typename T>
	class AABBTree {
	public:

		enum CostFunction { VOLUME, SURFACE_AREA };

		AABBTree(double aabb_margin_size, CostFunction cost_type=VOLUME) : aabb_margin_size(aabb_margin_size), root(nullptr), cost_type(cost_type) {}

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

		void add(T object, int object_id, const AABB& object_bounds) {
			Node* new_leaf = new Node(object, object_id, object_bounds, aabb_margin_size, cost_type);
			object_node_map[object_id] = new_leaf;

			if (root == nullptr) {
				root = new_leaf;
				return;
			}

			Node* best_candidate = nullptr;
			AABB best_candidate_aabb;
			double best_candidate_cost = std::numeric_limits<double>::infinity();

			std::vector<InsertionCandidate> candidates = { InsertionCandidate{root, 0} };
			while (!candidates.empty()) {
				InsertionCandidate c = candidates.back();
				candidates.pop_back();

				//impossible to be better
				if (c.inherited_cost >= best_candidate_cost) continue;

				AABB candidate_aabb = AABB::combine(new_leaf->node_aabb, c.node->node_aabb);
				double direct_cost = cost_type == VOLUME ? AABB::volume(candidate_aabb) : AABB::surfaceArea(candidate_aabb);
				double candidate_cost = c.inherited_cost + direct_cost;

				if (candidate_cost < best_candidate_cost) {
					best_candidate = c.node;
					best_candidate_aabb = candidate_aabb;
					best_candidate_cost = candidate_cost;
				}

				if (!c.node->is_leaf) {
					assert(c.node->left != nullptr && c.node->right != nullptr);

					double child_inherited_cost = c.inherited_cost + direct_cost - c.node->cost_value;
					candidates.push_back({ c.node->left, child_inherited_cost });
					candidates.push_back({ c.node->right, child_inherited_cost });
				}
			}

			assert(best_candidate != nullptr);

			Node* new_parent = new Node();
			if (best_candidate == root) {
				root = new_parent;
			}
			else {
				Node* old_parent = best_candidate->parent;
				Node** old_parent_child_slot = (old_parent->left == best_candidate) ? &old_parent->left : &old_parent->right;

				//replace new_parent in slot where best_canditate was
				new_parent->parent = old_parent;
				*old_parent_child_slot = new_parent;
			}

			//attach new_leaf and best_candidate as children of new_parent
			new_parent->left = best_candidate;
			best_candidate->parent = new_parent;
			new_parent->right = new_leaf;
			new_leaf->parent = new_parent;

			//propogate AABB changes that result from this
			haveAncestorsRecalculateAABB(new_leaf);

			//rebalance tree as necessary
			rebalanceAncestors(new_leaf);
		}

		void remove(unsigned int object_id) {
			assert(object_node_map.find(object_id) != object_node_map.end());
			Node* to_remove_leaf = object_node_map[object_id];
			object_node_map.erase(object_id);


			if (to_remove_leaf == root) {
				root = nullptr; //edge case 1
			}
			else {
				Node* parent = to_remove_leaf->parent;
				Node* sibling = (parent->left == to_remove_leaf) ? parent->right : parent->left;

				if (parent == root) {
					root = sibling;
					sibling->parent = nullptr; //edge case 2
				}
				else {
					Node* grandparent = parent->parent;
					Node** grandparent_child_slot = (grandparent->left == parent) ? &grandparent->left : &grandparent->right;

					sibling->parent = grandparent;
					*grandparent_child_slot = sibling;

					haveAncestorsRecalculateAABB(sibling); //general case
				}

				delete parent;
			}

			delete to_remove_leaf;
		}

		void update(T object, int object_id, const AABB& updated_object_bounds) {
			assert(object_node_map.find(object_id) != object_node_map.end());
			Node* object_leaf = object_node_map[object_id];

			if (!AABB::isAABBContained(updated_object_bounds, object_leaf->node_aabb)) {
				remove(object_id);
				add(object, object_id, updated_object_bounds);
			}
			else {
				object_leaf->leaf_object_true_aabb = updated_object_bounds;
			}
		}

		std::vector<Pair<T>> getAllCollisionCandidates() const {
			static int prev_colpair_size = 0;
			if (root == nullptr || root->is_leaf) return std::vector<Pair<T>>();

			std::vector<Pair<T>> col_pairs;
			col_pairs.reserve(prev_colpair_size);

			std::vector<TreePair> search_candidates = { {root->left, root->right} };
			while (!search_candidates.empty()) {
				TreePair tp = search_candidates.back();
				search_candidates.pop_back();

				if (!tp.n1->is_leaf && !tp.n1->internal_collision_checked_flag) {
					search_candidates.push_back({ tp.n1->left, tp.n1->right });
					tp.n1->internal_collision_checked_flag = true;
				}
				if (!tp.n2->is_leaf && !tp.n2->internal_collision_checked_flag) {
					search_candidates.push_back({ tp.n2->left, tp.n2->right });
					tp.n2->internal_collision_checked_flag = true;
				}

				if (!tp.n1->is_leaf && !tp.n2->is_leaf) {
					//non-leaf x non-leaf
					if (AABB::intersects(tp.n1->node_aabb, tp.n2->node_aabb)) {
						search_candidates.push_back({ tp.n1->left, tp.n2->left });
						search_candidates.push_back({ tp.n1->left, tp.n2->right });
						search_candidates.push_back({ tp.n1->right, tp.n2->left });
						search_candidates.push_back({ tp.n1->right, tp.n2->right });
					}
				}
				else if (tp.n1->is_leaf && !tp.n2->is_leaf) {
					//leaf x non-leaf
					if (AABB::intersects(tp.n1->leaf_object_true_aabb, tp.n2->node_aabb)) {
						search_candidates.push_back({ tp.n1, tp.n2->left });
						search_candidates.push_back({ tp.n1, tp.n2->right });
					}
				}
				else if (!tp.n1->is_leaf && tp.n2->is_leaf) {
					//non-leaf x leaf
					if (AABB::intersects(tp.n1->node_aabb, tp.n2->leaf_object_true_aabb)) {
						search_candidates.push_back({ tp.n1->left, tp.n2 });
						search_candidates.push_back({ tp.n1->right, tp.n2 });
					}
				}
				else {
					//leaf x leaf
					if (AABB::intersects(tp.n1->leaf_object_true_aabb, tp.n2->leaf_object_true_aabb)) {
						col_pairs.push_back(Pair<T>(tp.n1->leaf_object, tp.n1->leaf_object_id, tp.n2->leaf_object, tp.n2->leaf_object_id));
					}
				}
			}

			//reset internal_collision_checked_flags
			if (root != nullptr && !root->is_leaf) {

				std::vector<Node*> queue{ root };

				while (!queue.empty()) {
					Node* n = queue.back();
					queue.pop_back();

					assert(!n->is_leaf);
					n->internal_collision_checked_flag = false;

					if (!n->left->is_leaf) queue.push_back(n->left);
					if (!n->right->is_leaf) queue.push_back(n->right);
				}
			}

			prev_colpair_size = col_pairs.size();
			return col_pairs;
		}

		std::vector<T> getCollisionCandidatesWith(AABB target) const {
			if (root == nullptr) return std::vector<T>();

			std::vector<T> out;
			std::vector<Node*> node_candidates = { root };
			while (!node_candidates.empty()) {
				Node* n = node_candidates.back();
				node_candidates.pop_back();

				if (n->is_leaf) {
					if (AABB::intersects(n->leaf_object_true_aabb, target)) out.push_back(n->leaf_object);
				}
				else {
					if (AABB::intersects(n->node_aabb, target)) {
						node_candidates.push_back(n->left);
						node_candidates.push_back(n->right);
					}
				}
			}

			return out;
		}

		std::vector<T> raycastHitCandidates(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const {
			if (root == nullptr) return std::vector<T>();

			std::vector<T> out;
			std::vector<Node*> node_candidates = { root };
			while (!node_candidates.empty()) {
				Node* n = node_candidates.back();
				node_candidates.pop_back();

				if (n->is_leaf) {
					if (AABB::rayIntersectsAABB(n->leaf_object_true_aabb, ray_origin, ray_dir)) out.push_back(n->leaf_object);
				}
				else {
					if (AABB::rayIntersectsAABB(n->node_aabb, ray_origin, ray_dir)) {
						node_candidates.push_back(n->left);
						node_candidates.push_back(n->right);
					}
				}
			}

			return out;
		}

	private:

		class Node {
		public:
			Node()
				: parent(nullptr), left(nullptr), right(nullptr), is_leaf(false), leaf_object_id(-1), cost_value(0) {}

			Node(T object, int object_id, const AABB& object_aabb, double aabb_margin_size, CostFunction cost_type)
				: parent(nullptr), left(nullptr), right(nullptr), is_leaf(true), leaf_object(object), leaf_object_id(object_id), leaf_object_true_aabb(object_aabb)
			{
				setAABB(AABB{
					object_aabb.min - mthz::Vec3(aabb_margin_size, aabb_margin_size, aabb_margin_size),
					object_aabb.max + mthz::Vec3(aabb_margin_size, aabb_margin_size, aabb_margin_size)
					}, cost_type);
			}

			void setAABB(const AABB& aabb, CostFunction cost_type, double aabb_cost = -1) {
				node_aabb = aabb;
				if (aabb_cost == -1) {
					cost_value = cost_type == VOLUME ? AABB::volume(aabb) : AABB::surfaceArea(aabb);
				}
				else {
					cost_value = aabb_cost;
				}
			}

			void calculateAndUpdateAABB(CostFunction cost_type) {
				//there is no reason this function should be used with leaf nodes. Probably implies a bug
				assert(!is_leaf);
				//non-leaf nodes should always have two children
				assert(left != nullptr && right != nullptr);

				setAABB(AABB::combine(left->node_aabb, right->node_aabb), cost_type);
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
			double cost_value;

			//for use in checking all collisions
			bool internal_collision_checked_flag = false;

			bool is_leaf;
			T leaf_object;
			int leaf_object_id;
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

		CostFunction cost_type;
		double aabb_margin_size;
		std::unordered_map<unsigned int, Node*> object_node_map;
		Node* root;

		void haveAncestorsRecalculateAABB(Node* altered_node) {
			Node* current = altered_node->parent;
			while (current != nullptr) {
				current->calculateAndUpdateAABB(cost_type);
				current = current->parent;
			}
		}

		void rebalanceAncestors(Node* altered_leaf) {
			assert(altered_leaf->is_leaf);

			Node* current = altered_leaf->parent;
			while (current != nullptr && current->parent != nullptr) {

				Node* sibling = (current->parent->left == current) ? current->parent->right : current->parent->left;
				if (sibling != nullptr) {
					//cl: swap current with sibling->left
					//sr: swap sibling with current->right
					AABB cl_aabb, cr_aabb, sl_aabb, sr_aabb;
					double cl_cost, cr_cost, sl_cost, sr_cost;

					//current cannot be a leaf as we are following parent pointers
					sl_aabb = AABB::combine(current->right->node_aabb, sibling->node_aabb);
					sr_aabb = AABB::combine(current->left->node_aabb, sibling->node_aabb);
					sl_cost = cost_type == VOLUME ? AABB::volume(sl_aabb) : AABB::surfaceArea(sl_aabb);
					sr_cost = cost_type == VOLUME ? AABB::volume(sr_aabb) : AABB::surfaceArea(sr_aabb);

					if (sibling->is_leaf) {
						//inf worst possible, disqualifies the nodes
						cl_cost = std::numeric_limits<double>::infinity();
						cr_cost = std::numeric_limits<double>::infinity();
					}
					else {
						cl_aabb = AABB::combine(sibling->right->node_aabb, current->node_aabb);
						cr_aabb = AABB::combine(sibling->left->node_aabb, current->node_aabb);
						cl_cost = cost_type == VOLUME ? AABB::volume(cl_aabb) : AABB::surfaceArea(cl_aabb);
						cr_cost = cost_type == VOLUME ? AABB::volume(cr_aabb) : AABB::surfaceArea(cr_aabb);
					}

					double cl_cost_change = cl_cost - sibling->cost_value;
					double cr_cost_change = cr_cost - sibling->cost_value;
					double sl_cost_change = sl_cost - current->cost_value;
					double sr_cost_change = sr_cost - current->cost_value;

					double min = std::min<double>(cl_cost_change, std::min<double>(cr_cost_change, std::min<double>(sl_cost_change, sr_cost_change)));

					if (min >= 0) { /*no swap is best*/ }
					else if (min == cl_cost_change) {
						Node::swpNodes(current, sibling->left);
						sibling->setAABB(cl_aabb, cost_type, cl_cost);
					}
					else if (min == cr_cost_change) {
						Node::swpNodes(current, sibling->right);
						sibling->setAABB(cr_aabb, cost_type, cr_cost);
					}
					else if (min == sl_cost_change) {
						Node::swpNodes(sibling, current->left);
						current->setAABB(sl_aabb, cost_type, sl_cost);
					}
					else {
						Node::swpNodes(sibling, current->right);
						current->setAABB(sr_aabb, cost_type, sr_cost);
					}
				}

				current = current->parent;
			}
		}
	};
}