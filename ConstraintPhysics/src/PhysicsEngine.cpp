#include "PhysicsEngine.h"
#include "RigidBody.h"
#include "Octree.h"
#include "ThreadManager.h"
#include <vector>
#include <algorithm>
#include <cassert>
//debug
#include <chrono>

static const double FLATTENING_BIAS_MAG = 0;
static const double FLAT_ENOUGH = cos(PI * 0.1 / 180.0);

namespace phyz {

	int PhysicsEngine::n_threads = 0;
	ThreadManager PhysicsEngine::thread_manager;
	bool PhysicsEngine::use_multithread = false;
	bool PhysicsEngine::print_performance_data = false;

	void PhysicsEngine::enableMultithreading(int n_threads) {
		assert(n_threads >= 1);
		PhysicsEngine::n_threads = n_threads;
		thread_manager.init(n_threads);
		use_multithread = true;
	}

	void PhysicsEngine::disableMultithreading() {
		thread_manager.terminate_threads();
		use_multithread = false;
	}

	void PhysicsEngine::setPrintPerformanceData(bool print_data) {
		print_performance_data = print_data;
	}

	PhysicsEngine::~PhysicsEngine() {
		for (RigidBody* b : bodies) {
			delete constraint_graph_nodes[b->getID()];
			delete b;
		}
	}

	static float octree_time = 0;
	static float aabb_time = 0;
	static float none_time = 0;

	static float maintain_time = 0;
	static float broadphase_time = 0;
	static float sat_time = 0;
	static float dfs_time = 0;
	static float pgs_time = 0;
	static float update_time = 0;
	static int i = 0;
	void PhysicsEngine::timeStep() {

		//remove deleted bodies from relevant structures
		while (!bodies_to_delete.empty()) {
			RigidBody* to_delete = bodies_to_delete.back();
			bodies_to_delete.pop_back();

			deleteRigidBody(to_delete);
		}

		auto t1 = std::chrono::system_clock::now();

		//determine which bodies are active, apply gravity and pass them to broad-phase collision detection
		for (RigidBody* b : bodies) {
			if (b->recievedWakingAction) {
				wakeupIsland(constraint_graph_nodes[b->getID()]);
			}
			b->recievedWakingAction = false;

			if (b->movement_type == RigidBody::DYNAMIC && !b->getAsleep()) {
				b->recordMovementState(sleep_delay / step_time);

				if (bodySleepy(b)) {
					b->sleep_ready_counter += step_time;
				}
				else {
					b->non_sleepy_tick_count++;
					if (b->non_sleepy_tick_count >= non_sleepy_tick_threshold) {
						b->sleep_ready_counter = 0.0;
						b->non_sleepy_tick_count = 0;
					}
				}

				b->vel += gravity * step_time;
			}
		}

		maintainConstraintGraphApplyPoweredConstraints();

		auto t2 = std::chrono::system_clock::now();

		std::vector<Pair<RigidBody*>> possible_intersections;
		switch (broadphase) {
		case OCTREE:
		{
			Octree octree(octree_center, octree_size, octree_minsize);
			for (RigidBody* b : bodies) {
				octree.insert(b, b->aabb);
			}
			possible_intersections = octree.getAllIntersections();
		}
			break;
		case AABB_TREE:
			for (RigidBody* b : bodies) {
				aabb_tree.update(b, b->getMovementType() == RigidBody::DYNAMIC, b->getID(), b->aabb);
			}
			possible_intersections = aabb_tree.getAllCollisionCandidates();
			break;
		case BroadPhaseStructure::NONE:
			for (int i = 0; i < bodies.size(); i++) {
				for (int j = i + 1; j < bodies.size(); j++) {
					if (AABB::intersects(bodies[i]->aabb, bodies[j]->aabb)) {
						possible_intersections.push_back(Pair<RigidBody*>(bodies[i], bodies[i]->getID(), bodies[j], bodies[j]->getID()));
					}
				}
			}
			break;
		case TEST_COMPARE:
		{
			auto bt1 = std::chrono::system_clock::now();

			for (int i = 0; i < bodies.size(); i++) {
				for (int j = i + 1; j < bodies.size(); j++) {
					if (AABB::intersects(bodies[i]->aabb, bodies[j]->aabb)) {
						possible_intersections.push_back(Pair<RigidBody*>(bodies[i], bodies[i]->getID(), bodies[j], bodies[j]->getID()));
					}
				}
			}

			auto bt2 = std::chrono::system_clock::now();

			std::vector<Pair<RigidBody*>> octree_pairs;
			Octree octree(octree_center, octree_size, octree_minsize);
			for (RigidBody* b : bodies) {
				octree.insert(b, b->aabb);
			}
			octree_pairs = octree.getAllIntersections();

			auto bt3 = std::chrono::system_clock::now();

			std::vector<Pair<RigidBody*>> aabb_pairs;
			for (RigidBody* b : bodies) {
				aabb_tree.update(b, b->getMovementType() == RigidBody::DYNAMIC, b->getID(), b->aabb);
			}
			aabb_pairs = aabb_tree.getAllCollisionCandidates();

			auto bt4 = std::chrono::system_clock::now();

			none_time += std::chrono::duration<float>(bt2 - bt1).count();
			octree_time += std::chrono::duration<float>(bt3 - bt2).count();
			aabb_time += std::chrono::duration<float>(bt4 - bt3).count();

			assert(octree_pairs.size() == aabb_pairs.size() && aabb_pairs.size() == possible_intersections.size());
		}
		break;
		}

		auto t3 = std::chrono::system_clock::now();

		std::mutex action_mutex;
		struct TriggeredActionPair {
			RigidBody* b1;
			RigidBody* b2;
			std::vector<Manifold> manifolds;
			std::vector<ColAction> triggered_actions;
		};
		std::vector<TriggeredActionPair> triggered_actions;
		auto collision_detect = [&](const Pair<RigidBody*>& broadphase_pair) {
			OrderedBodyPair p(broadphase_pair.t1, broadphase_pair.t2); //OrderedBodyPair enforces which body is t1/t2 in a determenstic manner. Pair does not.

			RigidBody* b1 = p.t1;
			RigidBody* b2 = p.t2;


			bool collision_possible = (b1->getMovementType() == RigidBody::DYNAMIC || b2->getMovementType() == RigidBody::DYNAMIC) && (!b1->getAsleep() || !b2->getAsleep());

			if (collision_possible && collisionAllowed(b1, b2)) {

				std::vector<Manifold> manifolds;

				if (b1->getGeometryType() == RigidBody::STATIC_MESH && b2->getGeometryType() == RigidBody::STATIC_MESH) return;

				if (b1->getGeometryType() == RigidBody::CONVEX_UNION && b2->getGeometryType() == RigidBody::CONVEX_UNION) {
					for (int i = 0; i < b1->geometry.size(); i++) {
						for (int j = 0; j < b2->geometry.size(); j++) {
							const ConvexPrimitive& c1 = b1->geometry[i];
							const ConvexPrimitive& c2 = b2->geometry[j];
							bool checking_AABB_redundant = b1->geometry.size() == 1 && b2->geometry.size() == 1;
							if (!checking_AABB_redundant && !AABB::intersects(b1->geometry_AABB[i], b2->geometry_AABB[j])) {
								continue;
							}

							Manifold man = detectCollision(c1, c2);

							//no collision signified by pen_depth <= 0
							if (man.max_pen_depth > 0) {
								manifolds.push_back(man);
							}
						}
					}
				}
				else if (b1->getGeometryType() == RigidBody::CONVEX_UNION) {
					for (int i = 0; i < b1->geometry.size(); i++) {
						bool use_local_transformation_on_mesh = b2->getMovementType() == RigidBody::KINEMATIC;

						std::vector<Manifold> detected_manifolds;
						if (use_local_transformation_on_mesh) {
							detected_manifolds = detectCollision(b1->geometry[i], b1->geometry_AABB[i], b2->reference_mesh, b2->getCOM(), b2->getOrientation());
						}
						else {
							detected_manifolds = detectCollision(b1->geometry[i], b1->geometry_AABB[i], b2->mesh, mthz::Vec3(), mthz::Quaternion());
						}

						for (const Manifold& m : detected_manifolds) {
							manifolds.push_back(m);
						}
					}
				}
				else {
					for (int i = 0; i < b2->geometry.size(); i++) {
						bool use_local_transformation_on_mesh = b1->getMovementType() == RigidBody::KINEMATIC;

						std::vector<Manifold> detected_manifolds;
						if (use_local_transformation_on_mesh) {
							detected_manifolds = detectCollision(b1->reference_mesh, b1->getCOM(), b1->getOrientation(), b2->geometry[i], b2->geometry_AABB[i]);
						}
						else {
							detected_manifolds = detectCollision(b1->mesh, mthz::Vec3(), mthz::Quaternion(), b2->geometry[i], b2->geometry_AABB[i]);
						}
						for (const Manifold& m : detected_manifolds) {
							manifolds.push_back(m);
						}
					}
				}

				if (manifolds.size() > 0) {

					std::vector<bool> merged(manifolds.size(), false);
					for (int i = 0; i < manifolds.size(); i++) {
						if (merged[i]) {
							continue;
						}

						for (int j = i + 1; j < manifolds.size(); j++) {
							if (!merged[j]) {

								double v = manifolds[i].normal.dot(manifolds[j].normal);
								if (1 - v < COS_TOL) {
									manifolds[i] = merge_manifold(manifolds[i], manifolds[j]);
									merged[j] = true;
								}

							}
						}
						int cull_target_point_count = 4;
						Manifold man = cull_manifold(manifolds[i], cull_target_point_count);

						//need a determenistic order for locking the constraint graph mutexes, otherwise deadlock can occur.
						//This is already ensured by constructor of Pair type
						ConstraintGraphNode* lock_first, *lock_second;
						lock_first = constraint_graph_nodes[b1->getID()];
						lock_second = constraint_graph_nodes[b2->getID()];

						lock_first->mutex.lock();
						lock_second->mutex.lock();
						for (int i = 0; i < man.points.size(); i++) {
							const ContactP& p = man.points[i];
							addContact(lock_first, lock_second, p.pos, man.normal, p.magicID, p.restitution, p.static_friction_coeff, p.kinetic_friction_coeff, man.points.size(), p.pen_depth, posCorrectCoeff(contact_pos_correct_hardness, step_time));
						}
						lock_first->mutex.unlock();
						lock_second->mutex.unlock();

						std::vector<ColAction> thispair_actions;
						if (get_action_map.find(b1->getID()) != get_action_map.end() && get_action_map[b1->getID()].find(b2->getID()) != get_action_map[b1->getID()].end()) {
							for (ColActionID c : get_action_map[b1->getID()][b2->getID()]) thispair_actions.push_back(col_actions[c]);
						}
						if (get_action_map.find(b1->getID()) != get_action_map.end() && get_action_map[b1->getID()].find(all()) != get_action_map[b1->getID()].end()) {
							for (ColActionID c : get_action_map[b1->getID()][all()]) thispair_actions.push_back(col_actions[c]);
						}
						if (get_action_map.find(all()) != get_action_map.end() && get_action_map[all()].find(b2->getID()) != get_action_map[all()].end()) {
							for (ColActionID c : get_action_map[all()][b2->getID()]) thispair_actions.push_back(col_actions[c]);
						}
						if (get_action_map.find(all()) != get_action_map.end() && get_action_map[all()].find(all()) != get_action_map[all()].end()) {
							for (ColActionID c : get_action_map[all()][all()]) thispair_actions.push_back(col_actions[c]);
						}

						if (thispair_actions.size() > 0) {
							
							TriggeredActionPair pair_action_info = { b1, b2, std::vector<Manifold>(), thispair_actions };
							for (int i = 0; i < manifolds.size(); i++) {
								if (!merged[i]) {
									pair_action_info.manifolds.push_back(manifolds[i]);
								}
							}
							action_mutex.lock();
							triggered_actions.push_back(pair_action_info);
							action_mutex.unlock();
						}
					}
				}
			}
		};

		if (use_multithread) {
			thread_manager.do_all<Pair<RigidBody*>>(n_threads, possible_intersections, collision_detect);
		}
		else {
			for (const Pair<RigidBody*>& p : possible_intersections) {
				collision_detect(p);
			}
		}

		auto t4 = std::chrono::system_clock::now();

		//order of elements in triggered_actions depends on order of thread execution, sorting to restore determenism
		std::sort(triggered_actions.begin(), triggered_actions.end(), 
			[](const TriggeredActionPair& p1, const TriggeredActionPair& p2) {
				if (p1.b1->getID() < p2.b1->getID()) {
					return true;
				}
				else if (p1.b1->getID() > p2.b1->getID()) {
					return false;
				}
				else {
					return p1.b2->getID() < p2.b2->getID();
				}
			}
		);

		for (const TriggeredActionPair& pair : triggered_actions) {
			for (const ColAction& c : pair.triggered_actions) {
				c(pair.b1, pair.b2, pair.manifolds);
			}
		}

		ActiveConstraintData active_data = sleepOrSolveIslands();

		auto t5 = std::chrono::system_clock::now();

		if (use_multithread) {
			thread_manager.do_all<std::vector<Constraint*>>(n_threads, active_data.island_systems,
				[&](const std::vector<Constraint*>& island_system) {
					PGS_solve(this, island_system, pgsVelIterations, pgsPosIterations);
				}
			);
		}
		else {
			for (const std::vector<Constraint*>& island_system : active_data.island_systems) {
				PGS_solve(this, island_system, pgsVelIterations, pgsPosIterations);
			}
		}

		auto t6 = std::chrono::system_clock::now();

		auto update_positions = [&](RigidBody* b) {
			if ((b->getMovementType() != RigidBody::FIXED) && !b->getAsleep()) {
				b->prev_com = b->getCOM();
				b->prev_orientation = b->getOrientation();

				b->com += (b->vel + b->psuedo_vel) * step_time;

				if (b->ang_vel.magSqrd() != 0) {
					bool no_gyro = is_internal_gyro_forces_disabled || b->getMovementType() == RigidBody::KINEMATIC;

					b->rotateWhileApplyingGyroAccel(step_time, angle_velocity_update_tick_count, no_gyro);
					mthz::Vec3 psuedo_rot = b->psuedo_ang_vel;
					b->orientation = mthz::Quaternion(step_time * psuedo_rot.mag(), psuedo_rot) * b->orientation;
				}

				b->psuedo_vel = mthz::Vec3(0, 0, 0);
				b->psuedo_ang_vel = mthz::Vec3(0, 0, 0);
			}
		};

		if (use_multithread) {
			thread_manager.do_all<RigidBody*>(n_threads, bodies, update_positions);
		}
		else {
			for (RigidBody* b : bodies) {
				update_positions(b);
			}
		}

		applyMasterSlavePosCorrect(active_data);

		auto update_geometry = [&](RigidBody* b) {
			bool moveable = (b->getMovementType() != RigidBody::FIXED) && !b->getAsleep();
			if (moveable && (b->getCOM() != b->prev_com || b->getOrientation() != b->prev_orientation)) {
				b->updateGeometry();
			}
		};

		if (use_multithread) {
			thread_manager.do_all<RigidBody*>(n_threads, bodies, update_geometry);
		}
		else {
			for (RigidBody* b : bodies) {
				update_geometry(b);
			}
		}

		auto t7 = std::chrono::system_clock::now();

		if (print_performance_data) {

			maintain_time += std::chrono::duration<float>(t2 - t1).count();
			broadphase_time += std::chrono::duration<float>(t3 - t2).count();
			sat_time += std::chrono::duration<float>(t4 - t3).count();
			dfs_time += std::chrono::duration<float>(t5 - t4).count();
			pgs_time += std::chrono::duration<float>(t6 - t5).count();
			update_time += std::chrono::duration<float>(t7 - t6).count();

			if (i++ % 1000 == 0) {
				float total = maintain_time + broadphase_time + sat_time + dfs_time + pgs_time + update_time;
				printf("total time: %f, maintain: %f, broadphase: %f, sat: %f, dfs: %f, pgs: %f, update: %f\n", total, maintain_time / total, broadphase_time / total, sat_time / total, dfs_time / total, pgs_time / total, update_time / total);
				maintain_time = 0;
				broadphase_time = 0;
				sat_time = 0;
				dfs_time = 0;
				pgs_time = 0;
				update_time = 0;

				if (broadphase == TEST_COMPARE) {
					printf("AABB_Tree: %f, Octree %f, Bruteforce: %f\n", aabb_time, octree_time, none_time);
				}
			}

		}
	}

	void PhysicsEngine::applyMasterSlavePosCorrect(const ActiveConstraintData& a) {
		for (BallSocket* b : a.mast_slav_bss) {
			RigidBody* slave;
			mthz::Vec3 master_attachp, slave_attachp;
			if (b->b1_master) {
				slave = b->b2;
				master_attachp = b->b1->getTrackedP(b->b1_point_key);
				slave_attachp = b->b2->getTrackedP(b->b2_point_key);
			}
			else {
				slave = b->b1;
				master_attachp = b->b2->getTrackedP(b->b2_point_key);
				slave_attachp = b->b1->getTrackedP(b->b1_point_key);
			}

			slave->translateNoGeomUpdate(master_attachp - slave_attachp);
		}
		for (Hinge* h : a.mast_slav_hs) {
			RigidBody* slave;
			mthz::Vec3 master_attachp, slave_attachp;
			mthz::Vec3 master_rotaxis, slave_rotaxis;
			if (h->b1_master) {
				slave = h->b2;
				master_attachp = h->b1->getTrackedP(h->b1_point_key);
				slave_attachp = h->b2->getTrackedP(h->b2_point_key);
				master_rotaxis = h->b1->getOrientation().applyRotation(h->b1_rot_axis_body_space);
				slave_rotaxis = h->b2->getOrientation().applyRotation(h->b2_rot_axis_body_space);
			}
			else {
				slave = h->b1;
				master_attachp = h->b2->getTrackedP(h->b2_point_key);
				slave_attachp = h->b1->getTrackedP(h->b1_point_key);
				master_rotaxis = h->b2->getOrientation().applyRotation(h->b2_rot_axis_body_space);
				slave_rotaxis = h->b1->getOrientation().applyRotation(h->b1_rot_axis_body_space);
			}

			mthz::Vec3 crs = slave_rotaxis.cross(master_rotaxis);
			if (crs.magSqrd() > 0.000000001) {
				mthz::Vec3 rot_axis = crs.normalize();
				double ang = asin(crs.mag());
				slave->rotate(mthz::Quaternion(ang, rot_axis));
			}
			slave->translateNoGeomUpdate(master_attachp - slave_attachp);
		}
		for (Slider* s : a.mast_slav_ss) {
			RigidBody* master, *slave;
			mthz::Vec3 master_attachp, slave_attachp;
			mthz::Vec3 master_slide_axis;
			if (s->b1_master) {
				master = s->b1; slave = s->b2;
				master_attachp = s->b1->getTrackedP(s->b1_point_key);
				slave_attachp = s->b2->getTrackedP(s->b2_point_key);
				master_slide_axis = s->b1->getOrientation().applyRotation(s->b1_slide_axis_body_space);
			}
			else {
				master = s->b1; slave = s->b1;
				master_attachp = s->b2->getTrackedP(s->b2_point_key);
				slave_attachp = s->b1->getTrackedP(s->b1_point_key);
				master_slide_axis = s->b2->getOrientation().applyRotation(s->b2_slide_axis_body_space);
			}

			slave->rotateNoGeomUpdate(master->getOrientation() * slave->getOrientation().conjugate());
			mthz::Vec3 attachp_diff = master_attachp - slave_attachp;
			slave->translateNoGeomUpdate(attachp_diff - master_slide_axis * master_slide_axis.dot(attachp_diff));
		}
		for (SlidingHinge* s : a.mast_slav_shs) {
			RigidBody* master, * slave;
			mthz::Vec3 master_attachp, slave_attachp;
			mthz::Vec3 master_slide_axis, slave_slide_axis;
			if (s->b1_master) {
				master = s->b1; slave = s->b2;
				master_attachp = s->b1->getTrackedP(s->b1_point_key);
				slave_attachp = s->b2->getTrackedP(s->b2_point_key);
				master_slide_axis = s->b1->getOrientation().applyRotation(s->b1_slide_axis_body_space);
			}
			else {
				master = s->b1; slave = s->b1;
				master_attachp = s->b2->getTrackedP(s->b2_point_key);
				slave_attachp = s->b1->getTrackedP(s->b1_point_key);
				master_slide_axis = s->b2->getOrientation().applyRotation(s->b2_slide_axis_body_space);
			}

			mthz::Vec3 crs = slave_slide_axis.cross(master_slide_axis);
			if (crs.magSqrd() > 0.000000001) {
				mthz::Vec3 rot_axis = crs.normalize();
				double ang = asin(crs.mag());
				slave->rotateNoGeomUpdate(mthz::Quaternion(ang, rot_axis));
			}
			mthz::Vec3 attachp_diff = master_attachp - slave_attachp;
			slave->translateNoGeomUpdate(attachp_diff - master_slide_axis * master_slide_axis.dot(attachp_diff));
		}
		for (Weld* w : a.mast_slav_ws) {
			RigidBody* master, * slave;
			mthz::Vec3 master_attachp, slave_attachp;
			if (w->b1_master) {
				master = w->b1; slave = w->b2;
				master_attachp = w->b1->getTrackedP(w->b1_point_key);
				slave_attachp = w->b2->getTrackedP(w->b2_point_key);
			}
			else {
				master = w->b1; slave = w->b1;
				master_attachp = w->b2->getTrackedP(w->b2_point_key);
				slave_attachp = w->b1->getTrackedP(w->b1_point_key);
			}
			slave->rotateNoGeomUpdate(master->getOrientation()* slave->getOrientation().conjugate());
			slave->translateNoGeomUpdate(master_attachp - slave_attachp);
		}
	}

	RigidBody* PhysicsEngine::createRigidBody(const ConvexUnionGeometry& geometry, RigidBody::MovementType movement_type, mthz::Vec3 position, mthz::Quaternion orientation, bool override_center_of_mass, mthz::Vec3 center_of_mass_override) {
		RigidBody* r = new RigidBody(geometry, position, orientation, next_id++, override_center_of_mass, center_of_mass_override);
		r->setMovementType(movement_type);
		bodies.push_back(r);
		constraint_graph_nodes[r->getID()] = new ConstraintGraphNode(r);

		if (broadphase == AABB_TREE || broadphase == TEST_COMPARE) {
			aabb_tree.add(r, r->getMovementType() == RigidBody::DYNAMIC, r->getID(), r->aabb);
		}
		return r;
	}

	RigidBody* PhysicsEngine::createRigidBody(const StaticMeshGeometry& geometry, bool fixed) {
		RigidBody* r = new RigidBody(geometry, next_id++);
		if (fixed) {
			r->setMovementType(RigidBody::FIXED);
		}
		else {
			r->setMovementType(RigidBody::KINEMATIC);
		}
		bodies.push_back(r);
		constraint_graph_nodes[r->getID()] = new ConstraintGraphNode(r);

		if (broadphase == AABB_TREE || broadphase == TEST_COMPARE) {
			aabb_tree.add(r, r->getMovementType() == RigidBody::DYNAMIC, r->getID(), r->aabb);
		}
		return r;
	}

	void PhysicsEngine::removeRigidBody(RigidBody* r) {
		assert(std::find(bodies.begin(), bodies.end(), r) != bodies.end());
		bodies_to_delete.push_back(r);
	}

	void PhysicsEngine::deleteRigidBody(RigidBody* r) {
		if (broadphase == AABB_TREE || broadphase == TEST_COMPARE) {
			aabb_tree.remove(r->getID());
		}

		assert(std::find(bodies.begin(), bodies.end(), r) != bodies.end());
		bodies.erase(std::remove(bodies.begin(), bodies.end(), r));
		ConstraintGraphNode* n = constraint_graph_nodes[r->getID()];
		constraint_graph_nodes.erase(r->getID());
		delete n;
		delete r;
	}

	void PhysicsEngine::disallowCollision(RigidBody* b1, RigidBody* b2) {
		b1->no_collision_set.insert(b2);
		b2->no_collision_set.insert(b1);
	}

	void PhysicsEngine::disallowCollisionSet(const std::initializer_list<RigidBody*>& bodies) {
		for (auto b1 = bodies.begin(); b1 != bodies.end(); b1++) {
			for (auto b2 = b1 + 1; b2 != bodies.end(); b2++) {
				disallowCollision(*b1, *b2);
			}
		}
	}

	void PhysicsEngine::reallowCollisionSet(const std::initializer_list<RigidBody*>& bodies) {
		for (auto b1 = bodies.begin(); b1 != bodies.end(); b1++) {
			for (auto b2 = b1 + 1; b2 != bodies.end(); b2++) {
				reallowCollision(*b1, *b2);
			}
		}
	}

	bool PhysicsEngine::collisionAllowed(RigidBody* b1, RigidBody* b2) {
		return !b1->no_collision && !b2->no_collision && b1->no_collision_set.find(b2) == b1->no_collision_set.end();
	}

	void PhysicsEngine::reallowCollision(RigidBody* b1, RigidBody* b2) {
		auto i = b1->no_collision_set.find(b2);
		if (i != b1->no_collision_set.end()) {
			b1->no_collision_set.erase(i);
			b2->no_collision_set.erase(b2->no_collision_set.find(b1));
		}
	}

	RayHitInfo PhysicsEngine::raycastFirstIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir, std::vector<RigidBody*> ignore_list) {
		std::vector<RigidBody*> candidates;
		if (broadphase == TEST_COMPARE || broadphase == AABB_TREE) {
			candidates = aabb_tree.raycastHitCandidates(ray_origin, ray_dir);
		}
		else {
			for (RigidBody* b : bodies) {
				if (AABB::rayIntersectsAABB(b->aabb, ray_origin, ray_dir)) candidates.push_back(b);
			}
		}


		RayHitInfo closest_hit_info = { false };
		RigidBody* closest_hit_body = nullptr;
		for (RigidBody* b : candidates) {

			if (std::find(ignore_list.begin(), ignore_list.end(), b) != ignore_list.end()) continue;

			RayHitInfo hit_info = b->checkRayIntersection(ray_origin, ray_dir);
			if (hit_info.did_hit && (!closest_hit_info.did_hit || hit_info.hit_distance < closest_hit_info.hit_distance)) {
				closest_hit_info = hit_info;
				closest_hit_body = b;
			}

		}

		return closest_hit_info;
	}

	std::vector<RigidBody*> PhysicsEngine::getBodies() {
		std::vector<RigidBody*> out;
		for (RigidBody* b : bodies) {
			if (std::find(bodies_to_delete.begin(), bodies_to_delete.end(), b) == bodies_to_delete.end()) out.push_back(b);
		}
		return out;
	}

	unsigned int PhysicsEngine::getNextBodyID() {
		return next_id;
	}

	ConstraintID PhysicsEngine::addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, double pos_correct_strength) {
		return addBallSocketConstraint(b1, b2, attach_pos_local, attach_pos_local, pos_correct_strength);
	}

	ConstraintID PhysicsEngine::addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double pos_correct_strength) {
		int uniqueID = nextConstraintID++;
		disallowCollision(b1, b2);
		BallSocket* bs = new BallSocket{
			b1, b2,
			BallSocketConstraint(),
			b1->trackPoint(b1_attach_pos_local),
			b2->trackPoint(b2_attach_pos_local),
			pos_correct_strength,
			PSUEDO_VELOCITY,
			true,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1->getID()];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2->getID()];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->ballSocketConstraints.push_back(bs);

		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::BALL, uniqueID };
	}

	ConstraintID PhysicsEngine::addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, mthz::Vec3 rot_axis_local, double min_angle, double max_angle, double pos_correct_strength, double rot_correct_strength) {
		return addHingeConstraint(b1, b2, attach_pos_local, attach_pos_local, rot_axis_local, rot_axis_local, min_angle, max_angle, pos_correct_strength, rot_correct_strength);
	}

	ConstraintID PhysicsEngine::addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double min_angle, double max_angle, double pos_correct_strength, double rot_correct_strength) {
		disallowCollision(b1, b2);
		int uniqueID = nextConstraintID++;


		Hinge* h = new Hinge{
			b1, b2,
			HingeConstraint(),
			Motor(b1, b2, b1_rot_axis_local, b2_rot_axis_local, min_angle, max_angle),
			b1->trackPoint(b1_attach_pos_local),
			b2->trackPoint(b2_attach_pos_local),
			b1_rot_axis_local.normalize(),
			b2_rot_axis_local.normalize(),
			pos_correct_strength,
			rot_correct_strength,
			PSUEDO_VELOCITY,
			true,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1->getID()];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2->getID()];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->hingeConstraints.push_back(h);

		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::HINGE, uniqueID };
	}

	ConstraintID PhysicsEngine::addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double negative_slide_limit, double positive_slide_limit, double pos_correct_strength, double rot_correct_strength) {
		return addSliderConstraint(b1, b2, slider_pos_local, slider_pos_local, slider_axis_local, slider_axis_local, negative_slide_limit, positive_slide_limit, pos_correct_strength, rot_correct_strength);
	}

	ConstraintID PhysicsEngine::addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local,
		double negative_slide_limit, double positive_slide_limit, double pos_correct_strength, double rot_correct_strength) {
		disallowCollision(b1, b2);
		int uniqueID = nextConstraintID++;

		Slider* s = new Slider{
			b1, b2,
			SliderConstraint(),
			SlideLimitConstraint(),
			PistonConstraint(),
			b1->trackPoint(b1_slider_pos_local),
			b2->trackPoint(b2_slider_pos_local),
			b1_slider_axis_local.normalize(),
			b2_slider_axis_local.normalize(),
			0, 0,
			pos_correct_strength,
			rot_correct_strength,
			positive_slide_limit,
			negative_slide_limit,
			false,
			PSUEDO_VELOCITY,
			true,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1->getID()];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2->getID()];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->sliderConstraints.push_back(s);
		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::SLIDER, uniqueID };
	}

	ConstraintID PhysicsEngine::addSlidingHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double negative_slide_limit, double positive_slide_limit, double min_angle, double max_angle, double pos_correct_strength, double rot_correct_strength) {
		return addSlidingHingeConstraint(b1, b2, slider_pos_local, slider_pos_local, slider_axis_local, slider_axis_local, negative_slide_limit, positive_slide_limit, min_angle, max_angle, pos_correct_strength, rot_correct_strength);
	}

	ConstraintID PhysicsEngine::addSlidingHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local,
		double negative_slide_limit, double positive_slide_limit, double min_angle, double max_angle, double pos_correct_strength, double rot_correct_strength) {
		disallowCollision(b1, b2);
		int uniqueID = nextConstraintID++;

		SlidingHinge* s = new SlidingHinge{
			b1, b2,
			SlidingHingeConstraint(),
			SlideLimitConstraint(),
			PistonConstraint(),
			Motor(b1, b2, b1_slider_axis_local, b2_slider_axis_local, min_angle, max_angle),
			b1->trackPoint(b1_slider_pos_local),
			b2->trackPoint(b2_slider_pos_local),
			b1_slider_axis_local.normalize(),
			b2_slider_axis_local.normalize(),
			0, 0,
			pos_correct_strength,
			rot_correct_strength,
			positive_slide_limit,
			negative_slide_limit,
			false,
			PSUEDO_VELOCITY,
			true,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1->getID()];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2->getID()];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->slidingHingeConstraints.push_back(s);
		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::SLIDING_HINGE, uniqueID };
	}

	ConstraintID PhysicsEngine::addWeldConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_point_local, double pos_correct_strength, double rot_correct_strength) {
		return addWeldConstraint(b1, b2, attach_point_local, attach_point_local, pos_correct_strength, rot_correct_strength);
	}

	ConstraintID PhysicsEngine::addWeldConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_point_local, mthz::Vec3 b2_attach_point_local, double pos_correct_strength, double rot_correct_strength) {
		disallowCollision(b1, b2);
		int uniqueID = nextConstraintID++;

		Weld* w = new Weld{
			b1, b2,
			WeldConstraint(),
			b1->trackPoint(b1_attach_point_local),
			b2->trackPoint(b2_attach_point_local),
			pos_correct_strength,
			rot_correct_strength,
			PSUEDO_VELOCITY,
			true,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1->getID()];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2->getID()];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->weldConstraints.push_back(w);
		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::WELD, uniqueID };
	}

	ConstraintID PhysicsEngine::addSpring(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double damping, double stiffness, double resting_length) {
		int uniqueID = nextConstraintID++;
		RigidBody::PKey b1_key = b1->trackPoint(b1_attach_pos_local);
		RigidBody::PKey b2_key = b2->trackPoint(b2_attach_pos_local);
		if (resting_length < 0) resting_length = (b1->getTrackedP(b1_key) - b2->getTrackedP(b2_key)).mag(); //default

		Spring* s = new Spring{
			b1, b2,
			b1_key,
			b2_key,
			stiffness,
			damping,
			resting_length,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1->getID()];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2->getID()];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->springs.push_back(s);
		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::SPRING, uniqueID };

	}

	void PhysicsEngine::setConstraintPosCorrectMethod(ConstraintID id, PosErrorResolutionMode error_correct_method, bool b1_master) {
		assert(constraint_map.find(id.uniqueID) != constraint_map.end());

		SharedConstraintsEdge* e = constraint_map[id.uniqueID];
		switch (id.type) {
		case ConstraintID::BALL:
			for (int i = 0; i < e->ballSocketConstraints.size(); i++) {
				BallSocket* b = e->ballSocketConstraints[i];
				if (b->uniqueID == id.uniqueID) {
					b->pos_error_mode = error_correct_method;
					b->b1_master = b1_master;
				}
			}
			break;
		case ConstraintID::HINGE:
			for (int i = 0; i < e->hingeConstraints.size(); i++) {
				Hinge* h = e->hingeConstraints[i];
				if (h->uniqueID == id.uniqueID) {
					h->pos_error_mode = error_correct_method;
					h->b1_master = b1_master;
				}
			}
			break;
		case ConstraintID::SLIDER:
			for (int i = 0; i < e->sliderConstraints.size(); i++) {
				Slider* s = e->sliderConstraints[i];
				if (s->uniqueID == id.uniqueID) {
					s->pos_error_mode = error_correct_method;
					s->b1_master = b1_master;
				}
			}
			break;
		case ConstraintID::SLIDING_HINGE:
			for (int i = 0; i < e->slidingHingeConstraints.size(); i++) {
				SlidingHinge* s = e->slidingHingeConstraints[i];
				if (s->uniqueID == id.uniqueID) {
					s->pos_error_mode = error_correct_method;
					s->b1_master = b1_master;

				}

			}
			break;
		case ConstraintID::WELD:
			for (int i = 0; i < e->weldConstraints.size(); i++) {
				Weld* w = e->weldConstraints[i];
				if (w->uniqueID == id.uniqueID) {
					w->pos_error_mode = error_correct_method;
					w->b1_master = b1_master;

				}

			}
			break;
		}
	}

	void PhysicsEngine::removeConstraint(ConstraintID id, bool reenable_collision) {
		assert(constraint_map.find(id.uniqueID) != constraint_map.end());

		SharedConstraintsEdge* e = constraint_map[id.uniqueID];
		e->n1->b->alertWakingAction();
		e->n2->b->alertWakingAction();
		if (reenable_collision) {
			reallowCollision(e->n1->b, e->n2->b);
		}

		switch (id.type) {
		case ConstraintID::BALL:
			for (int i = 0; i < e->ballSocketConstraints.size(); i++) {
				BallSocket* b = e->ballSocketConstraints[i];
				if (b->uniqueID == id.uniqueID) {
					delete b;
					e->ballSocketConstraints.erase(e->ballSocketConstraints.begin() + i);
				}
			}
			break;
		case ConstraintID::HINGE:
			for (int i = 0; i < e->hingeConstraints.size(); i++) {
				Hinge* h = e->hingeConstraints[i];
				if (h->uniqueID == id.uniqueID) {
					delete h;
					e->hingeConstraints.erase(e->hingeConstraints.begin() + i);
				}
			}
			break;
		case ConstraintID::SLIDER:
			for (int i = 0; i < e->sliderConstraints.size(); i++) {
				Slider* s = e->sliderConstraints[i];
				if (s->uniqueID == id.uniqueID) {
					delete s;
					e->sliderConstraints.erase(e->sliderConstraints.begin() + i);
				}
			}
			break;
		case ConstraintID::SLIDING_HINGE:
			for (int i = 0; i < e->slidingHingeConstraints.size(); i++) {
				SlidingHinge* s = e->slidingHingeConstraints[i];
				if (s->uniqueID == id.uniqueID) {
					delete s;
					e->slidingHingeConstraints.erase(e->slidingHingeConstraints.begin() + i);

				}

			}
			break;
		case ConstraintID::WELD:
			for (int i = 0; i < e->weldConstraints.size(); i++) {
				Weld* w = e->weldConstraints[i];
				if (w->uniqueID == id.uniqueID) {
					delete w;
					e->weldConstraints.erase(e->weldConstraints.begin() + i);

				}

			}
			break;
		case ConstraintID::SPRING:
			for (int i = 0; i < e->springs.size(); i++) {
				Spring* s = e->springs[i];
				if (s->uniqueID == id.uniqueID) {
					delete s;
					e->springs.erase(e->springs.begin() + i);

				}

			}
			break;
		}

		constraint_map.erase(id.uniqueID);

	}

	void PhysicsEngine::addContact(ConstraintGraphNode* n1, ConstraintGraphNode* n2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness) {
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);
		RigidBody* b1 = n1->b;
		RigidBody* b2 = n2->b;

		for (Contact* c : e->contactConstraints) {
			if (c->magic == magic) {
				double friction = (c->friction.getStaticReady()) ? static_friction : kinetic_friction;
				NVec<1> contact_impulse = warm_start_disabled ? NVec<1>{ 0.0 } : c->contact.impulse;
				NVec<2> friction_impulse = warm_start_disabled ? NVec<2> { 0.0 } : c->friction.impulse;
				//In niche circumstances two pairs of friction and contact constraints can oppose each other, and if friction is greater than one this can cause a loop of increasing impulses.
				//Limiting the friction the the normal impulse from the last update helps mitigate this. Still not great if friction is high though.
				double normal_impulse_limit = (warm_start_disabled || !friction_impulse_limit_enabled) ? std::numeric_limits<double>::infinity() : c->contact.impulse.v[0];

				c->contact = ContactConstraint(b1, b2, norm, p, bounce, pen_depth, hardness, contact_impulse, cutoff_vel);
				c->friction = FrictionConstraint(b1, b2, norm, p, friction, &c->contact, friction_impulse, c->friction.u, c->friction.w, normal_impulse_limit);
				c->memory_life = contact_life;
				c->is_live_contact = true;
				return;
			}
		}

		//if no warm start existed
		Contact* c = new Contact();
		c->b1 = b1;
		c->b2 = b2;
		c->contact = ContactConstraint(b1, b2, norm, p, bounce, pen_depth, hardness, NVec<1>{0.0}, cutoff_vel);
		c->friction = FrictionConstraint(b1, b2, norm, p, kinetic_friction, &c->contact);
		c->magic = magic;
		c->memory_life = contact_life;
		c->is_live_contact = true;

		e->contactConstraints.push_back(c);
	}

	void PhysicsEngine::applyVelocityChange(RigidBody* b, const mthz::Vec3& delta_vel, const mthz::Vec3& delta_ang_vel, const mthz::Vec3& delta_psuedo_vel, const mthz::Vec3& delta_psuedo_ang_vel) {
		if (b->getMovementType() == RigidBody::FIXED || b->getMovementType() == RigidBody::KINEMATIC) return;

		b->vel += delta_vel;
		b->ang_vel += delta_ang_vel;
		b->psuedo_vel += delta_psuedo_vel;
		b->psuedo_ang_vel += delta_psuedo_ang_vel;

		assert(!(isnan(b->vel.mag()) || isnan(b->ang_vel.mag()) || isnan(b->psuedo_vel.mag()) || isnan(b->psuedo_ang_vel.mag())));
		assert(!std::isinf(b->ang_vel.mag()) && !std::isinf(b->vel.mag()) && !std::isinf(b->psuedo_ang_vel.mag()) && !std::isinf(b->psuedo_vel.mag()));

		double wake_vel = 0.25 * vel_sleep_coeff * gravity.mag(); //a little extra sensitive
		if (b->getAsleep() && (b->vel.mag() > wake_vel || b->ang_vel.mag() > wake_vel)) {
			constraint_graph_lock.lock();
			if (b->getAsleep()) { //in case of race condition, shouldn't really matter too much though
				wakeupIsland(constraint_graph_nodes[b->getID()]);
			}
			 constraint_graph_lock.unlock();
		}
		else if (b->getAsleep()) {
			b->vel = mthz::Vec3(0, 0, 0);
			b->ang_vel = mthz::Vec3(0, 0, 0);
		}
	}

	void PhysicsEngine::setSleepingEnabled(bool sleeping) {
		sleeping_enabled = sleeping;
	}

	void PhysicsEngine::setGravity(const mthz::Vec3& v) {
		gravity = v;
		cutoff_vel = getCutoffVel(step_time, gravity);
	}

	void PhysicsEngine::setBroadphase(BroadPhaseStructure b) {
		if ((broadphase != AABB_TREE && broadphase != TEST_COMPARE) && (b == AABB_TREE || b == TEST_COMPARE)) {
			//resets aabb tree, ensures all rigid bodies will be inserted in it (doing this to catch any new rigid bodies removed/deleted while AABBtree wasn't set as broadphase type)
			setAABBTreeMarginSize(aabbtree_margin_size);
		}

		broadphase = b;
	}

	void PhysicsEngine::setAABBTreeMarginSize(double d) {
		assert(d >= 0);
		aabbtree_margin_size = d;
		aabb_tree = AABBTree<RigidBody*>(aabbtree_margin_size); //resets tree

		//reinsert all elements
		for (RigidBody* r : bodies) {
			aabb_tree.add(r, r->getMovementType() == RigidBody::DYNAMIC, r->getID(), r->aabb);
		}
	}

	void PhysicsEngine::setOctreeParams(double size, double minsize, mthz::Vec3 center) {
		octree_center = center;
		octree_size = size;
		octree_minsize = minsize;
	}

	void PhysicsEngine::setAngleVelUpdateTickCount(int n) {
		assert(n >= 0);
		angle_velocity_update_tick_count = n;
	}

	void PhysicsEngine::setInternalGyroscopicForcesDisabled(bool b) {
		is_internal_gyro_forces_disabled = b;
	}

	void PhysicsEngine::setWarmStartDisabled(bool b) {
		warm_start_disabled = b;
	}

	void PhysicsEngine::setSleepParameters(double vel_sensitivity, double ang_vel_sensitivity, double aceleration_sensitivity, double sleep_assesment_time, int non_sleepy_tick_threshold) {
		vel_sleep_coeff = vel_sensitivity;
		ang_vel_eps = ang_vel_sensitivity;
		accel_sleep_coeff = aceleration_sensitivity;
		this->non_sleepy_tick_threshold = non_sleepy_tick_threshold;
	}

	void PhysicsEngine::deleteWarmstartData(RigidBody* r) {
		ConstraintGraphNode* n = constraint_graph_nodes[r->getID()];

		std::vector<SharedConstraintsEdge*> edges = n->constraints;
		for (SharedConstraintsEdge* e : n->constraints) {
			for (Contact* c : e->contactConstraints) {
				c->contact.impulse = NVec<1>{ 0.0 };
				c->friction.impulse = NVec<2>{ 0.0 };
			}
			for (BallSocket* bs : e->ballSocketConstraints) {
				bs->constraint.impulse = NVec<3>{ 0.0 };
			}
			for (Hinge* h : e->hingeConstraints) {
				h->constraint.impulse = NVec<5>{ 0.0 };
				h->motor.motor_constraint.impulse = NVec<1>{ 0.0 };
			}
			for (Slider* s : e->sliderConstraints) {
				s->constraint.impulse = NVec<5>{ 0.0 };
				s->piston_force.impulse = NVec<1>{ 0.0 };
			}
			for (SlidingHinge* s : e->slidingHingeConstraints) {
				s->constraint.impulse = NVec<4>{ 0.0 };
				s->piston_force.impulse = NVec<1>{ 0.0 };
				s->motor.motor_constraint.impulse = NVec<1>{ 0.0 };
			}
			for (Weld* w : e->weldConstraints) {
				w->constraint.impulse = NVec<6>{ 0.0 };
			}
		}
	}

	void PhysicsEngine::forceAABBTreeUpdate() {
		if (broadphase == AABB_TREE || broadphase == TEST_COMPARE) {
			for (RigidBody* b : bodies) {
				aabb_tree.update(b, b->getMovementType() == RigidBody::DYNAMIC, b->getID(), b->aabb);
			}
		}
	}

	void PhysicsEngine::setStep_time(double s) {
		step_time = s;
		cutoff_vel = getCutoffVel(step_time, gravity);
	}

	//TODO make ang vel more sensitive
	//average can be saved an adjusted per frame if necessary for performance
	bool PhysicsEngine::bodySleepy(RigidBody* r) {
		//not ideal as it prevents all other bodies in the same island from sleeping as well, but to make work otherwise would require a lot of reworking, and its a pretty niche use case
		if (r->sleep_disabled) return false;	

		const std::vector<RigidBody::MovementState>& body_history = r->history;
		int n = body_history.size();
		if (n == 0 || 2 * n < (sleep_delay / step_time)) {
			return false;
		}

		mthz::Vec3 average_vel;
		mthz::Vec3 average_ang_vel;
		for (const RigidBody::MovementState& s : body_history) {
			average_vel += s.vel;
			average_ang_vel += s.ang_vel;
		}

		average_vel /= n;
		average_ang_vel /= n;
		mthz::Vec3 average_accel = (body_history.back().vel - body_history.front().vel) / (step_time * (n - 1));
		mthz::Vec3 average_ang_accel = (body_history.back().ang_vel - body_history.front().ang_vel) / (step_time * (n - 1));

		double vel_eps = vel_sleep_coeff * gravity.mag();
		double accel_eps = accel_sleep_coeff * gravity.mag();
		double ang_accel_eps = accel_sleep_coeff * 2.5;


		bool a = average_vel.mag() <= vel_eps;
		bool b = average_ang_vel.mag() <= ang_vel_eps;
		bool c = average_accel.mag() <= accel_eps;
		bool d = average_ang_accel.mag() <= ang_accel_eps;

		return average_vel.mag() <= vel_eps && average_ang_vel.mag() <= ang_vel_eps && average_accel.mag() <= accel_eps && average_ang_accel.mag() <= ang_accel_eps;
	}

	bool PhysicsEngine::readyToSleep(RigidBody* b) {
		return b->getMovementType() != RigidBody::KINEMATIC && (b->sleep_ready_counter >= sleep_delay || b->getMovementType() == RigidBody::FIXED);
	}

	//Fixed bodies need to be treated a little weird. They should not bridge islands together (two seperate piles on the same floor should be different islands).
	//Thus fixed bodies exist only as leaves, and can't be 'curr' in initial call to dfs
	//Exception being where the very first node is fixed to propogate changes, such as translating a fixed object requires waking bodies around it
	void PhysicsEngine::bfsVisitAll(ConstraintGraphNode* curr, std::set<ConstraintGraphNode*>* visited, void* in, std::function<void(ConstraintGraphNode* curr, void* in)> action) {
		std::vector<ConstraintGraphNode*> to_visit = { curr };

		while (!to_visit.empty()) {
			ConstraintGraphNode* visiting_node = to_visit.back();
			to_visit.pop_back();

			bool already_visited = !visited->insert(visiting_node).second;
			if (already_visited) continue;

			if (visiting_node->b->getMovementType() == RigidBody::DYNAMIC) {
				action(visiting_node, in);
			}
			for (SharedConstraintsEdge* e : visiting_node->constraints) {
				ConstraintGraphNode* n = e->other(visiting_node);
				if (n->b->getMovementType() == RigidBody::DYNAMIC && visited->find(n) == visited->end()) {
					to_visit.push_back(n);
				}
			}
		}
	}

	void PhysicsEngine::wakeupIsland(ConstraintGraphNode* foothold) {
		std::set<ConstraintGraphNode*> visited;
		bfsVisitAll(foothold, &visited, nullptr, [this](ConstraintGraphNode* curr, void* in) {
			curr->b->wake();
		});
	}

	PhysicsEngine::Motor::Motor(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double min_angle, double max_angle)
		: b1(b1), b2(b2), target_velocity(0), max_torque(0), target_position(0), mode(OFF), min_motor_position(min_angle), max_motor_position(max_angle)
	{
		mthz::Vec3 u1, w1;
		b1_rot_axis_local.getPerpendicularBasis(&u1, &w1);
		mthz::Vec3 u2, tmp;
		b2_rot_axis_local.getPerpendicularBasis(&u2, &tmp);

		b1_u_axis_reference = u1;
		b1_w_axis_reference = w1;
		b2_rot_comparison_axis = u2;
		mthz::Vec3 rot_ref_axis_u = b1->orientation.applyRotation(b1_u_axis_reference);
		mthz::Vec3 rot_ref_axis_w = b1->orientation.applyRotation(b1_w_axis_reference);
		mthz::Vec3 rot_compare_axis = b2->orientation.applyRotation(b2_rot_comparison_axis);
		mthz::Vec3 b1_hinge_axis = b1->orientation.applyRotation(b1_rot_axis_local);

		motor_angular_position = 0; //normalizing for the calculatePosition calculation;
		motor_angular_position = calculatePosition(b1_hinge_axis, mthz::Vec3(), mthz::Vec3(), 0);
	}

	double PhysicsEngine::Motor::calculatePosition(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double step_time) {
		mthz::Vec3 ref_axis_u = b1->orientation.applyRotation(b1_u_axis_reference);
		mthz::Vec3 ref_axis_w = b1->orientation.applyRotation(b1_w_axis_reference);
		mthz::Vec3 compare_axis = b2->orientation.applyRotation(b2_rot_comparison_axis);

		//estimate based on velocity
		double relative_angular_velocity = rot_axis.dot(b1_ang_vel) - rot_axis.dot(b2_ang_vel);
		double predicted_new_angle = motor_angular_position + relative_angular_velocity * step_time;

		//use concrete orientations to eliminate drifting error
		if (abs(compare_axis.dot(rot_axis)) > 0.999) {
			//doesnt work if compairson is perp to rotation plane. just give up.
			return predicted_new_angle;
		}
		else {
			//normalize compare axis to rotation plane.
			mthz::Vec3 compare_norm = (compare_axis - rot_axis * rot_axis.dot(compare_axis)).normalize();

			//numerical accuracy issue breaks acos
			double dot_v = compare_norm.dot(ref_axis_u);
			if (dot_v > 1.0) dot_v = 1.0;
			if (dot_v < -1.0) dot_v = -1.0;

			//relative angle is flipped such that angles are clockwise. this is because i did things weird and this easiest way to fix it
			double relative_angle = 2 * PI - acos(dot_v);
			if (compare_norm.dot(ref_axis_w) < 0) relative_angle = 2 * PI - relative_angle;

			//pred_angle = n * 2PI + r
			double r = fmod(predicted_new_angle, 2 * PI);
			double n2PI = predicted_new_angle - r;

			//this is to handle edge cases where we are crossing the threshold between n2PI and (n+1)2PI, or n2PI to (n-1)2PI
			double corrected_angle_candidates[] = { n2PI + relative_angle - 2 * PI, n2PI + relative_angle, n2PI + relative_angle + 2 * PI };
			double closest = std::numeric_limits<double>::infinity();
			for (double c : corrected_angle_candidates) {
				if (abs(predicted_new_angle - c) < abs(predicted_new_angle - closest))
					closest = c;
			}

			return closest;
		}
	}

	void PhysicsEngine::Motor::writePrevVel(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel) {
		prev_velocity = rot_axis.dot(b1_ang_vel) - rot_axis.dot(b2_ang_vel);
	}

	double PhysicsEngine::Motor::getConstraintTargetVelocityValue(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double step_time) {
		switch (mode) {
		case OFF:
			return 0;
		case CONST_TORQUE:
			return (max_torque > 0) ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
		case TARGET_VELOCITY:
			return target_velocity;
		case TARGET_POSITION:
			double ang_diff = motor_angular_position - target_position;
			double target_dir = (ang_diff > 0) ? -1 : 1;
			//account for fixed/unfixed potentially changing after constraint created
			double vel_diff_respect_time = abs((rot_axis.dot(b1_ang_vel) - rot_axis.dot(b2_ang_vel)) - prev_velocity);

			const double EPS = 0.0000001;

			double inertia;
			if (vel_diff_respect_time < EPS || abs(motor_constraint.impulse.v[0] < EPS)) {
				//avoid dividing by 0, just try a different way (not the most accurate which is why is not used for general method)
				inertia = 1.0 / (rot_axis.dot(b1->getInvTensor() * rot_axis) + rot_axis.dot(b2->getInvTensor() * rot_axis));
			}
			else {
				inertia = abs(motor_constraint.impulse.v[0]) / vel_diff_respect_time;
			}

			double returnval = target_dir * sqrt(2 * abs(ang_diff) * max_torque / inertia);
			assert(!std::isinf(returnval));
			return returnval;
		}
	}

	bool PhysicsEngine::Motor::constraintIsActive() {
		return (mode != OFF && max_torque != 0) || motor_angular_position < min_motor_position || motor_angular_position > max_motor_position;
	}

	void PhysicsEngine::maintainConstraintGraphApplyPoweredConstraints() {
		static int current_visit_tag_value = 0;
		current_visit_tag_value++; //used to avoid visiting same constraint twice

		//Spring forces are not included in the PGS solver, so in order for other constraints to be aware of their influence they must be applied before the target velocities of constraints are calculated
		for (const auto& kv_pair : constraint_graph_nodes) {
			ConstraintGraphNode* n = kv_pair.second;
			for (auto i = 0; i < n->constraints.size(); i++) {
				SharedConstraintsEdge* e = n->constraints[i];

				if (e->visited_tag == current_visit_tag_value)
					continue; //skip
				e->visited_tag = current_visit_tag_value;

				for (Spring* s : e->springs) {
					mthz::Vec3 b1_pos = s->b1->getTrackedP(s->b1_point_key);
					mthz::Vec3 b2_pos = s->b2->getTrackedP(s->b2_point_key);
					mthz::Vec3 diff = b2_pos - b1_pos;
					double distance = diff.mag();
					mthz::Vec3 dir = (distance != 0) ? (b2_pos - b1_pos).normalize() : mthz::Vec3(0, -1, 0);
					double expand_force = s->stiffness * (distance - s->resting_length);

					s->b1->applyImpulse(dir * expand_force * step_time, b1_pos);
					s->b2->applyImpulse(-dir * expand_force * step_time, b2_pos);

					mthz::Vec3 rA = b1_pos - s->b1->getCOM();
					mthz::Vec3 rB = b2_pos - s->b2->getCOM();
					mthz::Vec3 rotDirA = s->b1->getInvTensor() * dir.cross(rA);
					mthz::Vec3 rotDirB = s->b2->getInvTensor() * dir.cross(rB);
					//how easily force translates into velocity change
					double velocity_sensitivity = 1.0 / (s->b1->getInvMass() + s->b2->getInvMass() + rA.cross(rotDirA).dot(dir) + rB.cross(rotDirB).dot(dir));

					double velocity = s->b2->getVelOfPoint(b1_pos).dot(dir) - s->b1->getVelOfPoint(b2_pos).dot(dir);
					double dampen = s->damping * velocity;
					double max_dampen = velocity * velocity_sensitivity / step_time; //max brings to standstill, more would overshoot and push in the opposite direction
					if (velocity > 0 && dampen > max_dampen) {
						dampen = max_dampen;
					}
					else if (velocity < 0 && dampen < max_dampen) {
						dampen = max_dampen;
					}

					s->b1->applyImpulse(dir * dampen * step_time, b1_pos);
					s->b2->applyImpulse(-dir * dampen * step_time, b2_pos);
				}
			}
		}


		current_visit_tag_value++; //need to update this value again
		for (const auto& kv_pair : constraint_graph_nodes) {
			ConstraintGraphNode* n = kv_pair.second;
			for (auto i = 0; i < n->constraints.size(); i++) {
				SharedConstraintsEdge* e = n->constraints[i];

				if (e->visited_tag == current_visit_tag_value) 
					continue; //skip
				e->visited_tag = current_visit_tag_value;

				for (auto j = e->contactConstraints.begin(); j != e->contactConstraints.end();) {
					Contact* contact = *j;
					contact->is_live_contact = false;
					if (!contact->b1->getAsleep() && !contact->b2->getAsleep() && contact->memory_life-- < 0) {
						delete contact;
						j = e->contactConstraints.erase(j);
					}
					else {
						j++;
					}
				}
				for (BallSocket* bs: e->ballSocketConstraints) {
					//update constraint for new positions
					mthz::Vec3 b1_pos = bs->b1->getTrackedP(bs->b1_point_key);
					mthz::Vec3 b2_pos = bs->b2->getTrackedP(bs->b2_point_key);
					NVec<3> starting_impulse = warm_start_disabled ? NVec<3>{ 0.0} : bs->constraint.impulse;
					double pos_correct_coeff = bs->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(bs->pos_correct_hardness, step_time) : 0;
					bs->constraint = BallSocketConstraint(bs->b1, bs->b2, b1_pos, b2_pos, pos_correct_coeff, starting_impulse);
				}
				for (Hinge* h : e->hingeConstraints) {
					mthz::Vec3 b1_pos = h->b1->getTrackedP(h->b1_point_key);
					mthz::Vec3 b2_pos = h->b2->getTrackedP(h->b2_point_key);
					mthz::Vec3 b1_hinge_axis = h->b1->orientation.applyRotation(h->b1_rot_axis_body_space);
					mthz::Vec3 b2_hinge_axis = h->b2->orientation.applyRotation(h->b2_rot_axis_body_space);

					h->motor.motor_angular_position = h->motor.calculatePosition(b1_hinge_axis, h->b1->getAngVel(), h->b2->getAngVel(), step_time);
					NVec<1> motor_starting_impulse = warm_start_disabled ? NVec<1>{ 0.0} : h->motor.motor_constraint.impulse;
					if (h->motor.constraintIsActive()) {
						h->motor.motor_constraint = MotorConstraint(h->b1, h->b2, b1_hinge_axis, h->motor.getConstraintTargetVelocityValue(b1_hinge_axis, h->b1->getAngVel(), h->b2->getAngVel(),
							step_time), abs(h->motor.max_torque * step_time), h->motor.motor_angular_position, h->motor.min_motor_position, h->motor.max_motor_position, posCorrectCoeff(h->rot_correct_hardness, step_time), motor_starting_impulse);
					}
					h->motor.writePrevVel(b1_hinge_axis, h->b1->getAngVel(), h->b2->getAngVel());

					double pos_correct_coeff = h->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(h->pos_correct_hardness, step_time) : 0;
					double rot_correct_coeff = h->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(h->rot_correct_hardness, step_time) : 0;

					NVec<5> starting_impulse = warm_start_disabled ? NVec<5>{ 0.0} : h->constraint.impulse;
					h->constraint = HingeConstraint(h->b1, h->b2, b1_pos, b2_pos, b1_hinge_axis, b2_hinge_axis, pos_correct_coeff, rot_correct_coeff, starting_impulse, h->constraint.u, h->constraint.w);
				}
				for (Slider* s : e->sliderConstraints) {
					mthz::Vec3 b1_pos = s->b1->getTrackedP(s->b1_point_key);
					mthz::Vec3 b2_pos = s->b2->getTrackedP(s->b2_point_key);
					mthz::Vec3 b1_slide_axis = s->b1->orientation.applyRotation(s->b1_slide_axis_body_space);
					mthz::Vec3 b2_slide_axis = s->b2->orientation.applyRotation(s->b2_slide_axis_body_space);

					double slider_value = b2_pos.dot(b1_slide_axis) - b1_pos.dot(b1_slide_axis);
					s->slide_limit_exceeded = slider_value < s->negative_slide_limit || slider_value > s->positive_slide_limit;

					if (s->slide_limit_exceeded) {
						NVec<1> limit_starting_impulse = warm_start_disabled ? NVec<1>{ 0.0} : s->slide_limit.impulse;
						s->slide_limit = SlideLimitConstraint(s->b1, s->b2, b1_slide_axis, slider_value, s->positive_slide_limit, s->negative_slide_limit, posCorrectCoeff(s->pos_correct_hardness, step_time), limit_starting_impulse);
					}

					if (s->max_piston_force != 0) {
						NVec<1> piston_starting_impulse = warm_start_disabled ? NVec<1>{ 0.0} : s->piston_force.impulse;
						s->piston_force = PistonConstraint(s->b1, s->b2, b1_slide_axis, s->target_velocity, s->max_piston_force * step_time, piston_starting_impulse);
					}

					double pos_correct_coeff = s->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(s->pos_correct_hardness, step_time) : 0;
					double rot_correct_coeff = s->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(s->rot_correct_hardness, step_time) : 0;

					NVec<5> starting_impulse = warm_start_disabled ? NVec<5>{ 0.0} : s->constraint.impulse;
					s->constraint = SliderConstraint(s->b1, s->b2, b1_pos, b2_pos, b1_slide_axis, pos_correct_coeff, pos_correct_coeff, starting_impulse, s->constraint.u, s->constraint.w);
				}
				for (SlidingHinge* s : e->slidingHingeConstraints) {
					mthz::Vec3 b1_pos = s->b1->getTrackedP(s->b1_point_key);
					mthz::Vec3 b2_pos = s->b2->getTrackedP(s->b2_point_key);
					mthz::Vec3 b1_slide_axis = s->b1->orientation.applyRotation(s->b1_slide_axis_body_space);
					mthz::Vec3 b2_slide_axis = s->b2->orientation.applyRotation(s->b2_slide_axis_body_space);

					double slider_value = b2_pos.dot(b1_slide_axis) - b1_pos.dot(b1_slide_axis);
					s->slide_limit_exceeded = slider_value < s->negative_slide_limit || slider_value > s->positive_slide_limit;

					if (s->slide_limit_exceeded) {
						NVec<1> limit_starting_impulse = warm_start_disabled ? NVec<1>{ 0.0} : s->slide_limit.impulse;
						s->slide_limit = SlideLimitConstraint(s->b1, s->b2, b1_slide_axis, slider_value, s->positive_slide_limit, s->negative_slide_limit, posCorrectCoeff(s->pos_correct_hardness, step_time), limit_starting_impulse);
					}

					if (s->max_piston_force != 0) {
						NVec<1> piston_starting_impulse = warm_start_disabled ? NVec<1>{ 0.0} : s->piston_force.impulse;
						s->piston_force = PistonConstraint(s->b1, s->b2, b1_slide_axis, s->target_velocity, s->max_piston_force * step_time, piston_starting_impulse);
					}

					s->motor.motor_angular_position = s->motor.calculatePosition(b1_slide_axis, s->b1->getAngVel(), s->b2->getAngVel(), step_time);
					if (s->motor.constraintIsActive()) {
						NVec<1> motor_starting_impulse = warm_start_disabled ? NVec<1>{ 0.0} : s->motor.motor_constraint.impulse;
						s->motor.motor_constraint = MotorConstraint(s->b1, s->b2, b1_slide_axis, s->motor.getConstraintTargetVelocityValue(b1_slide_axis, s->b1->getAngVel(), s->b2->getAngVel(), step_time), abs(s->motor.max_torque * step_time), s->motor.motor_angular_position, s->motor.min_motor_position, s->motor.max_motor_position, posCorrectCoeff(s->rot_correct_hardness, step_time), motor_starting_impulse);
					}
					s->motor.writePrevVel(b1_slide_axis, s->b1->getAngVel(), s->b2->getAngVel());

					double pos_correct_coeff = s->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(s->pos_correct_hardness, step_time) : 0;
					double rot_correct_coeff = s->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(s->rot_correct_hardness, step_time) : 0;

					NVec<4> starting_impulse = warm_start_disabled ? NVec<4>{ 0.0} : s->constraint.impulse;
					s->constraint = SlidingHingeConstraint(s->b1, s->b2, b1_pos, b2_pos, b1_slide_axis, b2_slide_axis, pos_correct_coeff, rot_correct_coeff, starting_impulse, s->constraint.u, s->constraint.w);
				}
				for (Weld* w : e->weldConstraints) {
					mthz::Vec3 b1_pos = w->b1->getTrackedP(w->b1_point_key);
					mthz::Vec3 b2_pos = w->b2->getTrackedP(w->b2_point_key);

					double pos_correct_coeff = w->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(w->pos_correct_hardness, step_time) : 0;
					double rot_correct_coeff = w->pos_error_mode == PSUEDO_VELOCITY || true ? posCorrectCoeff(w->rot_correct_hardness, step_time) : 0;

					w->constraint = WeldConstraint(w->b1, w->b2, b1_pos, b2_pos, pos_correct_coeff, rot_correct_coeff, w->constraint.impulse);
				}

				if (e->noConstraintsLeft()) {
					delete e;
					i--;
				}
			}
		}
	}

	PhysicsEngine::ActiveConstraintData PhysicsEngine::sleepOrSolveIslands() {
		std::set<ConstraintGraphNode*> visited;
		ActiveConstraintData out;
		//simple dfs
		for (const auto& kv_pair : constraint_graph_nodes) {
			ConstraintGraphNode* n = kv_pair.second;

			//fixed bodies can only be leaves of graph
			if (n->b->getMovementType() != RigidBody::DYNAMIC || visited.find(n) != visited.end()) {
				continue;
			}

			bool all_ready_to_sleep = true; //assumed true until its false
			std::vector<Constraint*> island_constraints;
			std::vector<RigidBody*> island_bodies;

			struct InStruct {
				bool* all_ready_to_sleep;
				std::vector<Constraint*>* island_constraints;
				std::vector<RigidBody*>* island_bodies;
				std::vector<BallSocket*>* mast_slav_bss;
				std::vector<Hinge*>* mast_slav_hs;
				std::vector<Slider*>* mast_slav_ss;
				std::vector<SlidingHinge*>* mast_slav_shs;
				std::vector<Weld*>* mast_slav_ws;
			};
			
			InStruct in = { &all_ready_to_sleep, &island_constraints, &island_bodies, &out.mast_slav_bss, &out.mast_slav_hs, &out.mast_slav_ss, &out.mast_slav_shs, &out.mast_slav_ws };
			int new_contact_life = this->contact_life;
			bfsVisitAll(n, &visited, (void*)&in, [&visited, new_contact_life, this](ConstraintGraphNode* curr, void* in) {
				InStruct* output = (InStruct*)in;

				bool interacting_with_kinematic = false;
				
				output->island_bodies->push_back(curr->b);
				for (SharedConstraintsEdge* e : curr->constraints) {
					ConstraintGraphNode* n = e->other(curr);

					if (n->b->getMovementType() == RigidBody::KINEMATIC) interacting_with_kinematic = true;
					if (curr->b->getAsleep() && (n->b->getAsleep() || n->b->getMovementType() == RigidBody::FIXED) || visited.find(n) != visited.end()) continue;

					for (Contact* c: e->contactConstraints) {
						if (c->is_live_contact) {
							output->island_constraints->push_back(&c->contact);
							output->island_constraints->push_back(&c->friction);
						}
					}
					for (BallSocket* bs : e->ballSocketConstraints) {
						output->island_constraints->push_back(&bs->constraint);
						if (bs->pos_error_mode == MASTER_SLAVE) output->mast_slav_bss->push_back(bs);
					}
					for (Hinge* h : e->hingeConstraints) {
						output->island_constraints->push_back(&h->constraint);
						if (h->pos_error_mode == MASTER_SLAVE) output->mast_slav_hs->push_back(h);
						if (h->motor.constraintIsActive()) {
							output->island_constraints->push_back(&h->motor.motor_constraint);
						}
					}
					for (Slider* s : e->sliderConstraints) {
						output->island_constraints->push_back(&s->constraint);
						if (s->pos_error_mode == MASTER_SLAVE) output->mast_slav_ss->push_back(s);
						if (s->max_piston_force != 0) {
							output->island_constraints->push_back(&s->piston_force);
						}
						if (s->slide_limit_exceeded) {
							output->island_constraints->push_back(&s->slide_limit);
						}
					}
					for (SlidingHinge* s : e->slidingHingeConstraints) {
						output->island_constraints->push_back(&s->constraint);
						if (s->pos_error_mode == MASTER_SLAVE) output->mast_slav_shs->push_back(s);
						if (s->max_piston_force != 0) {
							output->island_constraints->push_back(&s->piston_force);
						}
						if (s->slide_limit_exceeded) {
							output->island_constraints->push_back(&s->slide_limit);
						}
						if (s->motor.constraintIsActive()) {
							output->island_constraints->push_back(&s->motor.motor_constraint);
						}
					}
					for (Weld* w : e->weldConstraints) {
						if (w->pos_error_mode == MASTER_SLAVE) output->mast_slav_ws->push_back(w);
						output->island_constraints->push_back(&w->constraint);
					}
					
				}

				if (interacting_with_kinematic || !readyToSleep(curr->b)) {
					*output->all_ready_to_sleep = false;
				}
			});
			if (sleeping_enabled && all_ready_to_sleep) {
				for (RigidBody* b : island_bodies) {
					b->sleep();
				}
			}
			else if (island_constraints.size() > 0) {
				out.island_systems.push_back(island_constraints);
			}
		}

		return out;
	}

	CollisionTarget CollisionTarget::with(RigidBody* r) {
		return CollisionTarget(false, r->getID());
	}

	ColActionID PhysicsEngine::registerCollisionAction(CollisionTarget b1, CollisionTarget b2, const ColAction& action) {
		unsigned int t1 = (b1.with_all) ? all() : b1.specific_target;
		unsigned int t2 = (b2.with_all) ? all() : b2.specific_target;

		get_action_map[t1][t2].push_back(nextActionID);
		//avoiding duplicates
		if (!b1.with_all && !b2.with_all) {
			get_action_map[t2][t1].push_back(nextActionID);
		}
		col_actions[nextActionID] = action;

		return nextActionID++;
	}

	void PhysicsEngine::removeCollisionAction(ColActionID action_key) {
		for ( auto& kv_pair1 : get_action_map) {
			for (auto& kv_pair2 : kv_pair1.second) {
				std::vector<ColActionID>& reg_acts = kv_pair2.second;
				reg_acts.erase(std::remove(reg_acts.begin(), reg_acts.end(), action_key));
			}
		}
		col_actions.erase(action_key);
	}

	//would be nice to do something like fetch motor eventually but the redundancy is acceptable for now
	void PhysicsEngine::setPiston(ConstraintID id, double max_force, double target_velocity) {
		assert(constraint_map.find(id.uniqueID) != constraint_map.end());
		if (id.type == ConstraintID::SLIDER) {
			Slider* slider = nullptr;
			for (Slider* s : constraint_map[id.uniqueID]->sliderConstraints) {
				if (s->uniqueID == id.uniqueID) {
					slider = s;
					break;
				}
			}
			assert(slider != nullptr);

			slider->target_velocity = target_velocity;
			slider->max_piston_force = max_force;

			if (slider->constraint.a != nullptr) {
				slider->constraint.a->alertWakingAction();
				slider->constraint.b->alertWakingAction();
			}
		}
		else if (id.type == ConstraintID::SLIDING_HINGE) {
			SlidingHinge* slider = nullptr;
			for (SlidingHinge* s : constraint_map[id.uniqueID]->slidingHingeConstraints) {
				if (s->uniqueID == id.uniqueID) {
					slider = s;
					break;
				}
			}
			assert(slider != nullptr);

			slider->target_velocity = target_velocity;
			slider->max_piston_force = max_force;

			if (slider->constraint.a != nullptr) {
				slider->constraint.a->alertWakingAction();
				slider->constraint.b->alertWakingAction();
			}
		}
	}

	void PhysicsEngine::setMotorOff(ConstraintID id) {
		Motor* m = fetchMotor(id);

		m->mode = OFF;

		m->b1->alertWakingAction();
		m->b2->alertWakingAction();
	}

	void PhysicsEngine::setMotorConstantTorque(ConstraintID id, double torque) {
		Motor* m = fetchMotor(id);

		m->mode = CONST_TORQUE;
		m->max_torque = torque;

		m->b1->alertWakingAction();
		m->b2->alertWakingAction();
	}

	void PhysicsEngine::setMotorTargetVelocity(ConstraintID id, double max_torque, double target_velocity) {
		Motor* m = fetchMotor(id);

		m->mode = TARGET_VELOCITY;
		m->max_torque = max_torque;
		m->target_velocity = target_velocity;

		m->b1->alertWakingAction();
		m->b2->alertWakingAction();
	}

	void PhysicsEngine::setMotorTargetPosition(ConstraintID id, double max_torque, double target_position) {
		Motor* m = fetchMotor(id);

		m->mode = TARGET_POSITION;
		m->max_torque = max_torque;
		m->target_position = target_position;

		m->b1->alertWakingAction();
		m->b2->alertWakingAction();
	}

	PhysicsEngine::Motor* PhysicsEngine::fetchMotor(ConstraintID id) {
		assert(constraint_map.find(id.uniqueID) != constraint_map.end());
		if (id.type == ConstraintID::HINGE) {

			Hinge* hinge = nullptr;
			for (Hinge* h : constraint_map[id.uniqueID]->hingeConstraints) {
				if (h->uniqueID == id.uniqueID) {
					hinge = h;
					break;
				}
			}
			assert(hinge != nullptr);

			return &hinge->motor;
		}
		else if (id.type == ConstraintID::SLIDING_HINGE) {

			SlidingHinge* hinge = nullptr;
			for (SlidingHinge* h : constraint_map[id.uniqueID]->slidingHingeConstraints) {
				if (h->uniqueID == id.uniqueID) {
					hinge = h;
					break;
				}
			}
			assert(hinge != nullptr);

			return &hinge->motor;
		}
		else {
			assert(false);
		}
	}

	double PhysicsEngine::getMotorAngularPosition(ConstraintID id) {
		assert(constraint_map.find(id.uniqueID) != constraint_map.end());
		if (id.type == ConstraintID::HINGE) {
			Hinge* hinge = nullptr;
			for (Hinge* h : constraint_map[id.uniqueID]->hingeConstraints) {
				if (h->uniqueID == id.uniqueID) {
					hinge = h;
					break;
				}
			}
			assert(hinge != nullptr);

			return hinge->motor.motor_angular_position;
		}
		else if (id.type == ConstraintID::SLIDING_HINGE) {
			SlidingHinge* hinge = nullptr;
			for (SlidingHinge* h : constraint_map[id.uniqueID]->slidingHingeConstraints) {
				if (h->uniqueID == id.uniqueID) {
					hinge = h;
					break;
				}
			}
			assert(hinge != nullptr);

			return hinge->motor.motor_angular_position;
		}

		
	}

	PhysicsEngine::SharedConstraintsEdge::~SharedConstraintsEdge() {
		n1->constraints.erase(std::remove(n1->constraints.begin(), n1->constraints.end(), this));
		n2->constraints.erase(std::remove(n2->constraints.begin(), n2->constraints.end(), this));
		for (Contact* c : contactConstraints) delete c;
		for (BallSocket* b : ballSocketConstraints) delete b;
		for (Hinge* h : hingeConstraints) delete h;
		for (Slider* s : sliderConstraints) delete s;
	}

	PhysicsEngine::ConstraintGraphNode::~ConstraintGraphNode() {
		while (!constraints.empty()) {
			delete constraints.back(); //destructor removes itself from the vector
		}
	}

	PhysicsEngine::SharedConstraintsEdge* PhysicsEngine::ConstraintGraphNode::getOrCreateEdgeTo(ConstraintGraphNode* n2) {

		for (SharedConstraintsEdge* c : constraints) {
			if (c->other(this) == n2) {
				return c;
			}
		}

		//doesnt exist, create new edge
		SharedConstraintsEdge* c = new SharedConstraintsEdge(this, n2);
		insertNewEdge(c);
		n2->insertNewEdge(c);
		return c;
	}
	

	void PhysicsEngine::ConstraintGraphNode::insertNewEdge(SharedConstraintsEdge* e) {
		unsigned int other_constraint_id = e->other(this)->b->getID();

		//edges can be created in multi-threaded context. for determenism, they need to maintain sorted order.
		for (int i = 0; i < constraints.size(); i++) {
			if (other_constraint_id < constraints[i]->other(this)->b->getID()) {
				constraints.insert(constraints.begin() + i, e);
				return;
			}
		}
		constraints.push_back(e);
	}
}

	