#pragma once
#include "AABB.h"
#include <vector>
#include <unordered_map>
#include <cassert>
#include <stack>

namespace phyz {
	template <typename T>
	class AABBTree {
	private:
		//structs used in some functions
		struct InsertionCandidate {
			Node* node;
			double inherited_cost;
		};

		struct TreePair {
			Node* n1;
			Node* n2;
		};

	public:

		struct Pair {
			T* t1;
			T* t2;
		};

		AABBTree(double aabb_margin_size)
			: aabb_margin_size(aabb_margin_size), root(nullptr) {}

		~AABBTree() {
			std::vector<Node*> delete_queue { root };

			while (!delete_queue.empty()) {
				Node* to_delete = delete_queue.back();
				delete_queue.pop_back()

				if (!to_delete->is_leaf) {
					assert(to_delete->left != nullptr && to_delete->right != nullptr);

					delete_queue.push_back(to_delete->left);
					delete_queue.push_back(to_delete->right);
				}

				delete to_delete;
			}
		}

		void add(const T& object, const AABB& object_bounds) {
			Node* new_leaf = new Node(&object, object_bounds);
			object_node_map[object] = new_leaf;

			if (root == nullptr) {
				root = new_leaf
				return;
			}

			Node* best_candidate = nullptr;
			Node* best_candidate_aabb;
			Node* best_candidate_cost = std::numeric_limits<double>::infinity();

			std::vector<InsertionCandidate> candidates = { InsertionCandidate{root, 0} };
			while (!candidates.empty) {
				InsertionCandidate c = candidates.back();
				candidates.pop_back();

				//impossible to be better
				if (c.inherited_cost >= best_candidate_cost) continue;

				AABB candidate_aabb = AABB::combine(new_leaf->node_aabb, InsertionCandidate.node->node_aabb);
				double direct_cost = AABB:volume(candidate_aabb);
				double candidate_cost = c.inherited_cost + direct_cost;

				if (candidate_cost < best_candidate_cost) {
					best_candidate = InsertionCandidate.node;
					best_candidate_aabb = candidate_aabb;
					best_candidate_cost = candidate_cost;
				}

				if (!c.node->is_leaf) {
					assert(c.node->left != nullptr && c.node->right != nullptr);

					double child_inherited_cost = c.inherited_cost + direct_cost - c.node->volume;
					candidates.push_back({ c.node->left, child_inherited_cost });
					candidates.push_back({ c.node->right, child_inherited_cost });
				}
			}

			assert(best_candidate != nullptr);

			Node* new_parent = new Node();
			if (best_candidate == root) {
				root = new_parent;
			}
			if (best_candidate != root) {
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

		void remove(const T& object) {
			assert(object_node_map.find(object) != object_node_map.end());
			Node* to_remove_leaf = object_node_map[object];
			object_node_map.erase(object);


			if (to_remove_leaf == root) {
				root = nullptr; //edge case 1
			}
			else  {
				Node* parent = to_remove_leaf->parent;
				Node* sibling = (parent->left == to_remove_leaf;) ? parent->right : parent->left;

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

		void update(const T& object, const AABB& updated_object_bounds) {
			assert(object_node_map.find(object) != object_node_map.end());
			Node* object_leaf = object_node_map[object];

			if (!AABB::isAABBContained(updated_object_bounds, object_leaf->node_aabb)) {
				remove(object);
				add(object, updated_object_bounds);
			}
		}

		std::vector<Pair> getAllCollisionCandidates() const {
			static int prev_colpair_size = 0;
			if (root == nullptr || root->is_leaf) return std::vector<Pair>();

			std::vector<Pair> col_pairs();
			col_pairs.reserve(prev_colpair_size);

			std::vector<TreePair> search_candidates = { {root->left, root->right} };
			while (!search_candidates.empty()) {
				TreePair tp = search_candidates.back();
				search_candidates.pop_back();

				if (!tp.n1->is_leaf && !tp.n2->is_leaf) {
					//non-leaf x non-leaf
					if (AABB::intersects(tp.n1->node_aabb, tp.n2->node_aabb)) {
						search_candidates.push_back({ tp.n1->left, tp.n2->left });
						search_candidates.push_back({ tp.n1->left, tp.n2->right });
						search_candidates.push_back({ tp.n1->right, tp.n2->left });
						search_candidates.push_back({ tp.n1->right, tp->n2->right });
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
						col_pairs.push_back({ tp.n1->leaf_object, tp.n2->leaf_object });
					}
				}
			}

			prev_colpair_size = col_pairs.size();
			return col_pairs;
		}

	private:

		double aabb_margin_size;
		std::unordered_map<T, Node*> object_node_map;
		Node* root;


		class Node {
		public:
			Node()
				: parent(nullptr), left(nullptr), right(nullptr), is_leaf(false), leaf_object(nullptr), volume(0) {}

			Node(T* object, const AABB& object_aabb)
				: parent(nullptr), left(nullptr), right(nullptr), is_leaf(true), leaf_object(object), leaf_object_true_aabb(object_aabb)
			{
				setAABB(AABB{
					object_aabb.min - mthz::Vec3(aabb_margin_size, aabb_margin_size, aabb_margin_size),
					object_aabb.max + mthz::Vec3(aabb_margin_size, aabb_margin_size, aabb_margin_size)
				});
			}

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

				setAABB(AABB::combine(left->node_aabb, right->node_abb));
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

			bool is_leaf;
			T* leaf_object;
			AABB leaf_object_true_aabb;
		};

		void haveAncestorsRecalculateAABB(Node* altered_node) {
			Node* current = altered_leaf->parent;
			while (current != nullptr) {
				current->calculateAndUpdateAABB();
				current = current->parent;
			}
		}

		void rebalanceAncestors(Node* altered_leaf) {
			assert(altered_leaf->is_leaf);

			Node* current = altered_leaf->parent;
			while (current != nullptr && current->parent != nullptr) {
				
				Node* sibling = (current->parent->left == current;) ? current->parent->right : current->parent->left;
				if (sibling != nullptr) {
					//cl: swap current with sibling->left
					//sr: swap sibling with current->right
					AABB cl_aabb, cr_aabb, sl_aabb, sr_aabb;
					double cl_vol, cr_vol, sl_vol, sr_vol;

					//current cannot be a leaf as we are following parent pointers
					sl_aabb = AABB::combine(current->right->aabb, sibling->node_aabb);
					sr_aabb = AABB::combine(current->left->aabb, sibling->node_aabb);
					sl_vol = AABB::volume(sl_aabb);
					sr_vol = AABB::volume(sr_aabb);

					if (sibling->is_leaf) {
						//inf worst possible, disqualifies the nodes
						cl_vol = std::numeric_limits<double>::infinity();
						cr_vol = std::numeric_limits<double>::infinity();
					}
					else {
						cl_aabb = AABB::combine(sibling->right->aabb, current->node_aabb);
						cr_aabb = AABB::combine(sibling->left->aabb, current->node_aabb);
						cl_vol = AABB::volume(cl_aabb);
						cr_vol = AABB::volume(cr_aabb);
					}

					double cl_vol_change = cl_vol - sibling->volume;
					double cr_vol_change = cr_vol - sibling->volume;
					double sl_vol_change = sl_vol - current->volume;
					double sr_vol_change = sr_vol - current->volume;

					double min = std::min<double>({ cl_vol_change, cr_vol_change, sl_vol_change, sr_vol_change });

					if (min >= 0)					{ /*no swap is best*/ }
					else if (min == cl_vol_change) {
						Node::swpNodes(current, sibling->left);
						sibling->setAABB(cl_aabb, cl_vol);
					}
					else if (min == cr_vol_change) {
						Node::swpNodes(current, sibling->right);
						sibling->setAABB(cr_aabb, cr_vol);
					}
					else if (min == sl_vol_change) {
						Node::swpNodes(sibling, current->left);
						current->setAABB(sl_aabb, sl_vol);
					}
					else {
						Node::swpNodes(sibling, current->right);
						current->setAABB(sr_aabb, sr_vol);
					}
				}

				current = current->parent;
			}
		}
	};

}