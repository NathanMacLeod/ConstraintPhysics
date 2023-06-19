#include "AABB_Tree.h"

namespace phyz {

	AABBTree::Node::Node(const RigidBody* object, const AABB& object_aabb, double aabb_margin_size)
		: parent(nullptr), left(nullptr), right(nullptr), is_leaf(true), leaf_object(object), leaf_object_true_aabb(object_aabb)
	{
		setAABB(AABB{
			object_aabb.min - mthz::Vec3(aabb_margin_size, aabb_margin_size, aabb_margin_size),
			object_aabb.max + mthz::Vec3(aabb_margin_size, aabb_margin_size, aabb_margin_size)
			});
	}

	void AABBTree::add(const RigidBody* object, const AABB& object_bounds) {
		Node* new_leaf = new Node(object, object_bounds, aabb_margin_size);
		object_node_map[object] = new_leaf;

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
			double direct_cost = AABB::volume(candidate_aabb);
			double candidate_cost = c.inherited_cost + direct_cost;

			if (candidate_cost < best_candidate_cost) {
				best_candidate = c.node;
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

	void AABBTree::remove(const RigidBody* object) {
		assert(object_node_map.find(object) != object_node_map.end());
		Node* to_remove_leaf = object_node_map[object];
		object_node_map.erase(object);


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

	void AABBTree::update(const RigidBody* object, const AABB& updated_object_bounds) {
		assert(object_node_map.find(object) != object_node_map.end());
		Node* object_leaf = object_node_map[object];

		if (!AABB::isAABBContained(updated_object_bounds, object_leaf->node_aabb)) {
			remove(object);
			add(object, updated_object_bounds);
		}
		else {
			object_leaf->leaf_object_true_aabb = updated_object_bounds;
		}
	}

	std::vector<Pair> AABBTree::getAllCollisionCandidates() const {
		static int prev_colpair_size = 0;
		if (root == nullptr || root->is_leaf) return std::vector<Pair>();

		std::vector<Pair> col_pairs;
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
					col_pairs.push_back(Pair((RigidBody*)tp.n1->leaf_object, (RigidBody*)tp.n2->leaf_object));
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

	std::vector<RigidBody*> AABBTree::raycastHitCandidates(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const {
		if (root == nullptr) return std::vector<RigidBody*>();

		std::vector<RigidBody*> out;
		std::vector<Node*> node_candidates = { root };
		while (!node_candidates.empty()) {
			Node* n = node_candidates.back();
			node_candidates.pop_back();

			if (n->is_leaf) {
				if (AABB::rayIntersectsAABB(n->leaf_object_true_aabb, ray_origin, ray_dir)) out.push_back((RigidBody*)n->leaf_object);
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

	void AABBTree::rebalanceAncestors(AABBTree::Node* altered_leaf) {
		assert(altered_leaf->is_leaf);

		Node* current = altered_leaf->parent;
		while (current != nullptr && current->parent != nullptr) {

			Node* sibling = (current->parent->left == current) ? current->parent->right : current->parent->left;
			if (sibling != nullptr) {
				//cl: swap current with sibling->left
				//sr: swap sibling with current->right
				AABB cl_aabb, cr_aabb, sl_aabb, sr_aabb;
				double cl_vol, cr_vol, sl_vol, sr_vol;

				//current cannot be a leaf as we are following parent pointers
				sl_aabb = AABB::combine(current->right->node_aabb, sibling->node_aabb);
				sr_aabb = AABB::combine(current->left->node_aabb, sibling->node_aabb);
				sl_vol = AABB::volume(sl_aabb);
				sr_vol = AABB::volume(sr_aabb);

				if (sibling->is_leaf) {
					//inf worst possible, disqualifies the nodes
					cl_vol = std::numeric_limits<double>::infinity();
					cr_vol = std::numeric_limits<double>::infinity();
				}
				else {
					cl_aabb = AABB::combine(sibling->right->node_aabb, current->node_aabb);
					cr_aabb = AABB::combine(sibling->left->node_aabb, current->node_aabb);
					cl_vol = AABB::volume(cl_aabb);
					cr_vol = AABB::volume(cr_aabb);
				}

				double cl_vol_change = cl_vol - sibling->volume;
				double cr_vol_change = cr_vol - sibling->volume;
				double sl_vol_change = sl_vol - current->volume;
				double sr_vol_change = sr_vol - current->volume;

				double min = std::min<double>(cl_vol_change, std::min<double>(cr_vol_change, std::min<double>(sl_vol_change, sr_vol_change)));

				if (min >= 0) { /*no swap is best*/ }
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
}