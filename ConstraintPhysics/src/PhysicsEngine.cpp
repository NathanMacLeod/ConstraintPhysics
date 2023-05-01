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
			delete constraint_graph_nodes[b];
			delete b;
		}
	}

	static float octree_maintain_time = 0;
	static float sat_time = 0;
	static float dfs_time = 0;
	static float pgs_time = 0;
	static int i = 0;
	void PhysicsEngine::timeStep() {

		//remove deleted bodies from relevant structures
		while(!bodies_to_delete.empty()) {
			RigidBody* to_delete = bodies_to_delete.back();
			assert(std::find(bodies.begin(), bodies.end(), to_delete) != bodies.end());
			bodies.erase(std::remove(bodies.begin(), bodies.end(), to_delete));
			bodies_to_delete.pop_back();
		}

		auto t1 = std::chrono::system_clock::now();
	
		Octree<RigidBody> octree(octree_center, octree_size, octree_minsize);
		//determine which bodies are active, apply gravity and pass them to broad-phase collision detection
		for (RigidBody* b : bodies) {
			if (b->recievedWakingAction) {
				wakeupIsland(constraint_graph_nodes[b]);
			}
			b->recievedWakingAction = false;

			if (!b->asleep && !b->fixed) {
				b->recordMovementState(sleep_delay / step_time);
				if (bodySleepy(b->history)) {
					b->sleep_ready_counter += step_time;
				}
				else {
					b->sleep_ready_counter = 0;
				}
				
				b->vel += gravity * step_time;
			}

			octree.insert(*b, b->aabb);
		}

		maintainConstraintGraphApplyPoweredConstraints();

		std::vector<Octree<RigidBody>::Pair> possible_intersections = octree.getAllIntersections();
		auto t2 = std::chrono::system_clock::now();

		std::mutex action_mutex;
		struct TriggeredActionPair {
			RigidBody* b1;
			RigidBody* b2;
			std::vector<Manifold> manifolds;
			std::vector<ColAction> triggered_actions;
		};
		std::vector<TriggeredActionPair> triggered_actions;
		auto collision_detect = [&](const Octree<RigidBody>::Pair& p) {
			RigidBody* b1 = p.t1;
			RigidBody* b2 = p.t2;

			if ((!b1->fixed || !b1->asleep || !b2->fixed || !b2->asleep) && collisionAllowed(b1, b2)) {

				std::vector<Manifold> manifolds;
				for (int i = 0; i < b1->geometry.size(); i++) {
					for (int j = 0; j < b2->geometry.size(); j++) {
						const ConvexPrimitive& c1 = b1->geometry[i];
						const ConvexPrimitive& c2 = b2->geometry[j];
						bool checking_AABB_redundant = b1->geometry.size() == 1 && b2->geometry.size() == 1;
						if (!checking_AABB_redundant && !AABB::intersects(b1->geometry_AABB[i], b2->geometry_AABB[j])) {
							continue;
						}

						Manifold man = detectCollision(c1, c2);

						if (man.max_pen_depth > 0) {
							manifolds.push_back(man);
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

						mthz::Vec3 u, w;
						man.normal.getPerpendicularBasis(&u, &w);

						constraint_graph_lock.lock();
						for (int i = 0; i < man.points.size(); i++) {
							const ContactP& p = man.points[i];
							addContact(b1, b2, p.pos, man.normal, p.magicID, p.restitution, p.static_friction_coeff, p.kinetic_friction_coeff, man.points.size(), p.pen_depth, posCorrectCoeff(350, step_time));
						}
						constraint_graph_lock.unlock();

						std::vector<ColAction> thispair_actions;
						action_mutex.lock();
						for (ColActionID c : get_action_map[b1][b2]) thispair_actions.push_back(col_actions[c]);
						for (ColActionID c : get_action_map[b1][b2]) thispair_actions.push_back(col_actions[c]);
						for (ColActionID c : get_action_map[all()][b2]) thispair_actions.push_back(col_actions[c]);
						for (ColActionID c : get_action_map[all()][all()]) thispair_actions.push_back(col_actions[c]);
						action_mutex.unlock();

						if (thispair_actions.size() > 0) {
							action_mutex.lock();
							TriggeredActionPair pair_action_info = { b1, b2, std::vector<Manifold>(), thispair_actions };
							for (int i = 0; i < manifolds.size(); i++) {
								if (!merged[i]) {
									pair_action_info.manifolds.push_back(manifolds[i]);
								}
							}
							triggered_actions.push_back(pair_action_info);
							action_mutex.unlock();
						}
					}
				}
			}
		};

		if (use_multithread) {
			thread_manager.do_all<Octree<RigidBody>::Pair>(n_threads, possible_intersections, collision_detect);
		}
		else {
			for (const Octree<RigidBody>::Pair& p : possible_intersections) {
				collision_detect(p);
			}
		}

		auto t3 = std::chrono::system_clock::now();

		for (const TriggeredActionPair& pair : triggered_actions) {
			for (const ColAction& c : pair.triggered_actions) {
				c(pair.b1, pair.b2, pair.manifolds);
			}
		}

		std::vector<std::vector<Constraint*>> island_systems = sleepOrSolveIslands();

		auto t4 = std::chrono::system_clock::now();

		if (use_multithread) {
			thread_manager.do_all<std::vector<Constraint*>>(n_threads, island_systems,
				[&](const std::vector<Constraint*>& island_system) {
					PGS_solve(this, island_system, pgsVelIterations, pgsPosIterations);
				}
			);
		}
		else {
			for (const std::vector<Constraint*>& island_system : island_systems) {
				PGS_solve(this, island_system, pgsVelIterations, pgsPosIterations);
			}
		}
		
		auto t5 = std::chrono::system_clock::now();

		for (RigidBody* b : bodies) {
			if (!b->fixed && !b->asleep) {
				b->com += (b->vel + b->psuedo_vel) * step_time;
				if (b->ang_vel.magSqrd() != 0) {
					b->rotateWhileApplyingGyroAccel(step_time, 4);
					mthz::Vec3 psuedo_rot = b->psuedo_ang_vel;
					b->orientation = mthz::Quaternion(step_time * psuedo_rot.mag(), psuedo_rot) * b->orientation;
				}

				b->updateGeometry();

				if (isnan(b->ang_vel.x)) {
					int a = 1 + 2;
				}

				b->psuedo_vel = mthz::Vec3(0, 0, 0);
				b->psuedo_ang_vel = mthz::Vec3(0, 0, 0);
			}
		}

		if (print_performance_data) {

			octree_maintain_time += std::chrono::duration<float>(t2 - t1).count();
			sat_time += std::chrono::duration<float>(t3 - t2).count();
			dfs_time += std::chrono::duration<float>(t4 - t3).count();
			pgs_time += std::chrono::duration<float>(t5 - t4).count();

			if (i++ % 1000 == 0) {
				float total = octree_maintain_time + sat_time + dfs_time + pgs_time;
				printf("total time: %f, octree/maintain: %f, sat: %f, dfs: %f, pgs: %f\n", total, octree_maintain_time / total, sat_time / total, dfs_time / total, pgs_time / total);
				octree_maintain_time = 0;
				sat_time = 0;
				dfs_time = 0;
				pgs_time = 0;
			}

		}
	}

	RigidBody* PhysicsEngine::createRigidBody(const Geometry& geometry, bool fixed, mthz::Vec3 position, mthz::Quaternion orientation) {
		RigidBody* r = new RigidBody(geometry, position, orientation);
		r->fixed = fixed;
		bodies.push_back(r);
		constraint_graph_nodes[r] = new ConstraintGraphNode(r);
		return r;
	}

	void PhysicsEngine::removeRigidBody(RigidBody* r) {
		assert(std::find(bodies.begin(), bodies.end(), r) != bodies.end());
		bodies_to_delete.push_back(r);
	}

	void PhysicsEngine::deleteRigidBody(RigidBody* r) {
		assert(std::find(bodies.begin(), bodies.end(), r) != bodies.end());
		bodies.erase(std::remove(bodies.begin(), bodies.end(), r));
		ConstraintGraphNode* n = constraint_graph_nodes[r];
		constraint_graph_nodes.erase(r);
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

	ConstraintID PhysicsEngine::addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, double pos_correct_strength) {
		return addBallSocketConstraint(b1, b2, attach_pos_local, attach_pos_local, pos_correct_strength);
	}

	ConstraintID PhysicsEngine::addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double pos_correct_strength) {
		int uniqueID = nextConstraintID++;
		disallowCollision(b1, b2);
		BallSocket* bs = new BallSocket{
			BallSocketConstraint(),
			b1->trackPoint(b1_attach_pos_local),
			b2->trackPoint(b2_attach_pos_local),
			pos_correct_strength,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->ballSocketConstraints.push_back(bs);
		
		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::BALL, uniqueID };
	}

	ConstraintID PhysicsEngine::addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, mthz::Vec3 rot_axis_local, double pos_correct_strength, double rot_correct_strength) {
		return addHingeConstraint(b1, b2, attach_pos_local, attach_pos_local, rot_axis_local, rot_axis_local, pos_correct_strength, rot_correct_strength);
	}

	ConstraintID PhysicsEngine::addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double pos_correct_strength, double rot_correct_strength) {
		disallowCollision(b1, b2);
		int uniqueID = nextConstraintID++;
		mthz::Vec3 u1, w1;
		b1_rot_axis_local.getPerpendicularBasis(&u1, &w1);
		mthz::Vec3 u2, tmp;
		b2_rot_axis_local.getPerpendicularBasis(&u2, &tmp);

		Hinge* h = new Hinge{
			HingeConstraint(),
			MotorConstraint(),
			b1->trackPoint(b1_attach_pos_local),
			b2->trackPoint(b2_attach_pos_local),
			b1_rot_axis_local.normalize(),
			b2_rot_axis_local.normalize(),
			pos_correct_strength,
			rot_correct_strength,
			0, 0,
			uniqueID
		};

		h->b1_u_axis_reference = u1;
		h->b1_w_axis_reference = w1;
		h->b2_rot_comparison_axis = u2;
		mthz::Vec3 rot_ref_axis_u = b1->orientation.applyRotation(h->b1_u_axis_reference);
		mthz::Vec3 rot_ref_axis_w = b1->orientation.applyRotation(h->b1_w_axis_reference);
		mthz::Vec3 rot_compare_axis = b2->orientation.applyRotation(h->b2_rot_comparison_axis);
		mthz::Vec3 b1_hinge_axis = b1->orientation.applyRotation(h->b1_rot_axis_body_space);
		h->motor_angular_position = calculateMotorPosition(0, b1_hinge_axis, rot_ref_axis_u, rot_ref_axis_w, rot_compare_axis, mthz::Vec3(), mthz::Vec3(), 0);

		h->min_motor_position = -std::numeric_limits<double>::infinity();
		h->max_motor_position = std::numeric_limits<double>::infinity();

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->hingeConstraints.push_back(h);
		
		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::HINGE, uniqueID };
	}

	ConstraintID PhysicsEngine::addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double pos_correct_strength, double rot_correct_strength, double positive_slide_limit, double negative_slide_limit) {
		return addSliderConstraint(b1, b2, slider_pos_local, slider_pos_local, slider_axis_local, slider_axis_local, pos_correct_strength, rot_correct_strength, positive_slide_limit, negative_slide_limit);
	}

	ConstraintID PhysicsEngine::addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local, 
		double pos_correct_strength, double rot_correct_strength, double positive_slide_limit, double negative_slide_limit) {
		disallowCollision(b1, b2);
		int uniqueID = nextConstraintID++;

		Slider* s = new Slider{
			SliderConstraint(),
			ContactConstraint(),
			b1->trackPoint(b1_slider_pos_local),
			b2->trackPoint(b2_slider_pos_local),
			b1_slider_axis_local.normalize(),
			b2_slider_axis_local.normalize(),
			pos_correct_strength,
			rot_correct_strength,
			positive_slide_limit,
			negative_slide_limit,
			false,
			uniqueID
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->sliderConstraints.push_back(s);
		constraint_map[uniqueID] = e;
		return ConstraintID{ ConstraintID::SLIDER, uniqueID };
	}

	void PhysicsEngine::removeConstraint(ConstraintID id, bool reenable_collision) {
		assert(constraint_map.find(id.uniqueID) != constraint_map.end());

		SharedConstraintsEdge* e = constraint_map[id.uniqueID];
		e->n1->b->alertWakingAction();
		e->n2->b->alertWakingAction();
		reallowCollision(e->n1->b, e->n2->b);
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
		}

		constraint_map.erase(id.uniqueID);
		
	}

	void PhysicsEngine::addContact(RigidBody* b1, RigidBody* b2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness) {
		ConstraintGraphNode* n1 = constraint_graph_nodes[b1]; ConstraintGraphNode* n2 = constraint_graph_nodes[b2];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		for (Contact* c : e->contactConstraints) {
			if (c->magic == magic) {
				double friction = (c->friction.getStaticReady()) ? static_friction : kinetic_friction;

				c->contact = ContactConstraint(b1, b2, norm, p, bounce, pen_depth, hardness, c->contact.impulse, cutoff_vel);
				c->friction = FrictionConstraint(b1, b2, norm, p, friction, n_points, &c->contact, c->friction.impulse, c->friction.u, c->friction.w);
				c->memory_life = contact_life;
				c->is_live_contact = true;
				return;
			}
		}

		//if no warm start existed
		Contact* c = new Contact();
		c->contact = ContactConstraint(b1, b2, norm, p, bounce, pen_depth, hardness, NVec<1>{0.0}, cutoff_vel);
		c->friction = FrictionConstraint(b1, b2, norm, p, kinetic_friction, n_points, &c->contact);
		c->magic = magic;
		c->memory_life = contact_life;
		c->is_live_contact = true;

		e->contactConstraints.push_back(c);
	}

	void PhysicsEngine::applyVelocityChange(RigidBody* b, const mthz::Vec3& delta_vel, const mthz::Vec3& delta_ang_vel, const mthz::Vec3& delta_psuedo_vel, const mthz::Vec3& delta_psuedo_ang_vel) {
		b->vel += delta_vel;
		b->ang_vel += delta_ang_vel;
		b->psuedo_vel += delta_psuedo_vel;
		b->psuedo_ang_vel += delta_psuedo_ang_vel;
		double wake_vel = 0.25 * vel_sleep_coeff * gravity.mag(); //a little extra sensitive
		if (b->asleep && (b->vel.mag() > wake_vel || b->ang_vel.mag() > wake_vel)) {
			constraint_graph_lock.lock();
			if (b->asleep) { //in case of race condition, shouldn't really matter too much though
				wakeupIsland(constraint_graph_nodes[b]);
			}
			constraint_graph_lock.unlock();
		}
		else if (b->asleep) {
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

	void PhysicsEngine::setOctreeParams(double size, double minsize, mthz::Vec3 center) {
		octree_center = center;
		octree_size = size;
		octree_minsize = minsize;
	}

	void PhysicsEngine::setStep_time(double s) {
		step_time = s;
		cutoff_vel = getCutoffVel(step_time, gravity);
	}

	//average can be saved an adjusted per frame if necessary for performance
	bool PhysicsEngine::bodySleepy(const std::vector<RigidBody::MovementState>& body_history) {
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
		return average_vel.mag() <= vel_eps && average_ang_vel.mag() <= vel_eps && average_accel.mag() <= 2*accel_eps && average_ang_accel.mag() <= accel_eps;
	}

	bool PhysicsEngine::readyToSleep(RigidBody* b) {
		return b->sleep_ready_counter >= sleep_delay || b->fixed;
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

			if (!visiting_node->b->fixed) {
				action(visiting_node, in);
			}
			for (SharedConstraintsEdge* e : visiting_node->constraints) {
				ConstraintGraphNode* n = e->other(visiting_node);
				if (!n->b->fixed && visited->find(n) == visited->end()) {
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

	double PhysicsEngine::calculateMotorPosition(double current_position, mthz::Vec3 rot_axis, mthz::Vec3 ref_axis_u, mthz::Vec3 ref_axis_w, mthz::Vec3 compare_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double timestep) {
		//estimate based on velocity
		double relative_angular_velocity = rot_axis.dot(b1_ang_vel) - rot_axis.dot(b2_ang_vel);
		double predicted_new_angle = current_position + relative_angular_velocity * step_time;

		//use concrete orientations to eliminate drifting error
		if (abs(compare_axis.dot(rot_axis)) > 0.999) {
			//doesnt work if compairson is perp to rotation plane. just give up.
			return predicted_new_angle;
		}
		else {
			

			//normalize compare axis to rotation plane.
			mthz::Vec3 compare_norm = (compare_axis - rot_axis * rot_axis.dot(compare_axis)).normalize();
			//relative angle is flipped such that angles are clockwise. this is because i did things weird and this easiest way to fix it
			double relative_angle = 2 * PI - acos(compare_norm.dot(ref_axis_u));
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

	void PhysicsEngine::maintainConstraintGraphApplyPoweredConstraints() {
		for (const auto& kv_pair : constraint_graph_nodes) {
			ConstraintGraphNode* n = kv_pair.second;
			for (auto i = 0; i < n->constraints.size(); i++) {
				SharedConstraintsEdge* e = n->constraints[i];
				RigidBody* b1 = e->n1->b;
				RigidBody* b2 = e->n2->b;
				for (auto j = e->contactConstraints.begin(); j != e->contactConstraints.end();) {
					Contact* contact = *j;
					contact->is_live_contact = false;
					if (!b1->asleep && !b2->asleep && contact->memory_life-- < 0) {
						delete contact;
						j = e->contactConstraints.erase(j);
					}
					else {
						j++;
					}
				}
				for (BallSocket* bs: e->ballSocketConstraints) {
					//update constraint for new positions
					mthz::Vec3 b1_pos = b1->getTrackedP(bs->b1_point_key);
					mthz::Vec3 b2_pos = b2->getTrackedP(bs->b2_point_key);
					bs->constraint = BallSocketConstraint(b1, b2, b1_pos, b2_pos, posCorrectCoeff(bs->pos_correct_hardness, step_time), bs->constraint.impulse);
				}
				for (Hinge* h : e->hingeConstraints) {
					mthz::Vec3 b1_pos = b1->getTrackedP(h->b1_point_key);
					mthz::Vec3 b2_pos = b2->getTrackedP(h->b2_point_key);
					mthz::Vec3 b1_hinge_axis = b1->orientation.applyRotation(h->b1_rot_axis_body_space);
					mthz::Vec3 b2_hinge_axis = b2->orientation.applyRotation(h->b2_rot_axis_body_space);

					mthz::Vec3 rot_ref_axis_u = b1->orientation.applyRotation(h->b1_u_axis_reference);
					mthz::Vec3 rot_ref_axis_w = b1->orientation.applyRotation(h->b1_w_axis_reference);
					mthz::Vec3 rot_compare_axis = b2->orientation.applyRotation(h->b2_rot_comparison_axis);
					h->motor_angular_position = calculateMotorPosition(h->motor_angular_position, b1_hinge_axis, rot_ref_axis_u, rot_ref_axis_w, rot_compare_axis, b1->getAngVel(), b2->getAngVel(), step_time);
					

					if (h->max_torque > 0 || h->min_motor_position != -std::numeric_limits<double>::infinity() || h->max_motor_position != std::numeric_limits<double>::infinity()) {
						h->motor_constraint = MotorConstraint(b1, b2, b1_hinge_axis, h->target_velocity, h->max_torque, h->motor_angular_position, h->min_motor_position, h->max_motor_position, h->rot_correct_hardness, h->motor_constraint.impulse);
					}

					h->constraint = HingeConstraint(b1, b2, b1_pos, b2_pos, b1_hinge_axis, b2_hinge_axis, posCorrectCoeff(h->pos_correct_hardness, step_time), 
						posCorrectCoeff(h->rot_correct_hardness, step_time), h->constraint.impulse, h->constraint.u, h->constraint.w);
				}
				for (Slider* s : e->sliderConstraints) {
					mthz::Vec3 b1_pos = b1->getTrackedP(s->b1_point_key);
					mthz::Vec3 b2_pos = b2->getTrackedP(s->b2_point_key);
					mthz::Vec3 b1_slide_axis = b1->orientation.applyRotation(s->b1_slide_axis_body_space);
					mthz::Vec3 b2_slide_axis = b2->orientation.applyRotation(s->b2_slide_axis_body_space);

					double slider_value = b2_pos.dot(b1_slide_axis) - b1_pos.dot(b1_slide_axis);
					if (slider_value > s->positive_slide_limit) {
						s->slide_limit = ContactConstraint(b1, b2, -b1_slide_axis, b1_pos + s->positive_slide_limit * b1_slide_axis, 0, slider_value - s->positive_slide_limit, s->pos_correct_hardness, s->slide_limit.impulse);
						s->slide_limit_exceeded = true;
					}
					else if (slider_value < -s->negative_slide_limit) {
						s->slide_limit = ContactConstraint(b1, b2, b1_slide_axis, b1_pos - s->negative_slide_limit * b1_slide_axis, 0, -s->negative_slide_limit - slider_value, s->pos_correct_hardness, s->slide_limit.impulse);
						s->slide_limit_exceeded = true;
					}
					else {
						s->slide_limit.impulse = NVec<1>{ 0 };
						s->slide_limit_exceeded = false;
					}

					s->constraint = SliderConstraint(b1, b2, b1_pos, b2_pos, b1_slide_axis, b2_slide_axis, posCorrectCoeff(s->pos_correct_hardness, step_time),
						posCorrectCoeff(s->rot_correct_hardness, step_time), s->constraint.impulse, s->constraint.u, s->constraint.w);
				}
				if (e->noConstraintsLeft()) {
					delete e;
					i--;
				}
			}
		}
	}

	std::vector<std::vector<Constraint*>> PhysicsEngine::sleepOrSolveIslands() {
		std::set<ConstraintGraphNode*> visited;
		std::vector<std::vector<Constraint*>> island_systems;
		//simple dfs
		for (const auto& kv_pair : constraint_graph_nodes) {
			ConstraintGraphNode* n = kv_pair.second;

			//fixed bodies can only be leaves of graph
			if (n->b->fixed || n->b->asleep || visited.find(n) != visited.end()) {
				continue;
			}

			bool all_ready_to_sleep = true; //assumed true until its false
			std::vector<Constraint*> island_constraints;
			std::vector<RigidBody*> island_bodies;

			struct InStruct {
				bool* all_ready_to_sleep;
				std::vector<Constraint*>* island_constraints;
				std::vector<RigidBody*>* island_bodies;
			};
			
			InStruct in = { &all_ready_to_sleep, &island_constraints, &island_bodies };
			int new_contact_life = this->contact_life;
			bfsVisitAll(n, &visited, (void*)&in, [&visited, new_contact_life, this](ConstraintGraphNode* curr, void* in) {
				InStruct* output = (InStruct*)in;
				if (*output->all_ready_to_sleep && !readyToSleep(curr->b)) {
					*output->all_ready_to_sleep = false;
				}
				output->island_bodies->push_back(curr->b);
				for (SharedConstraintsEdge* e : curr->constraints) {
					ConstraintGraphNode* n = e->other(curr);
					if (visited.find(n) == visited.end()) {		
						for (Contact* c: e->contactConstraints) {
							if (c->is_live_contact) {
								output->island_constraints->push_back(&c->contact);
								output->island_constraints->push_back(&c->friction);
							}
						}
						for (BallSocket* bs : e->ballSocketConstraints) {
							output->island_constraints->push_back(&bs->constraint);
						}
						for (Hinge* h : e->hingeConstraints) {
							output->island_constraints->push_back(&h->constraint);
							if (h->max_torque > 0 || h->min_motor_position != -std::numeric_limits<double>::infinity() || h->max_motor_position != std::numeric_limits<double>::infinity()) {
								output->island_constraints->push_back(&h->motor_constraint);
							}
						}
						for (Slider* s : e->sliderConstraints) {
							output->island_constraints->push_back(&s->constraint);
							if (s->slide_limit_exceeded) {
								output->island_constraints->push_back(&s->slide_limit);
							}
						}
					}
				}
			});
			if (sleeping_enabled && all_ready_to_sleep) {
				for (RigidBody* b : island_bodies) {
					b->sleep();
				}
			}
			else if (island_constraints.size() > 0) {
				island_systems.push_back(island_constraints);
			}
		}

		return island_systems;
	}

	CollisionTarget CollisionTarget::all() {
		return CollisionTarget(true, nullptr);
	}

	CollisionTarget CollisionTarget::with(RigidBody* r) {
		return CollisionTarget(false, r);
	}

	ColActionID PhysicsEngine::registerCollisionAction(CollisionTarget b1, CollisionTarget b2, const ColAction& action) {
		RigidBody* t1 = (b1.with_all) ? all() : b1.specific_target;
		RigidBody* t2 = (b2.with_all) ? all() : b2.specific_target;
		get_action_map[t1][t2].push_back(nextActionID);
		get_action_map[t2][t1].push_back(nextActionID);
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

	void PhysicsEngine::setMotorProperties(ConstraintID id, double max_torque, double min_angle, double max_angle) {
		assert(id.type == ConstraintID::HINGE && constraint_map.find(id.uniqueID) != constraint_map.end());
		Hinge* hinge = nullptr;
		for (Hinge* h : constraint_map[id.uniqueID]->hingeConstraints) {
			if (h->uniqueID == id.uniqueID) {
				hinge = h;
				break;
			}
		}
		assert(hinge != nullptr);

		hinge->max_torque = max_torque;
		hinge->min_motor_position = min_angle;
		hinge->max_motor_position = max_angle;

		if (hinge->constraint.a != nullptr) {
			hinge->constraint.a->alertWakingAction();
			hinge->constraint.b->alertWakingAction();
		}
	}

	void PhysicsEngine::setMotorTargetVelocity(ConstraintID id, double target_velocity) {
		assert(id.type == ConstraintID::HINGE && constraint_map.find(id.uniqueID) != constraint_map.end());
		Hinge* hinge = nullptr;
		for (Hinge* h : constraint_map[id.uniqueID]->hingeConstraints) {
			if (h->uniqueID == id.uniqueID) {
				hinge = h;
				break;
			}
		}
		assert(hinge != nullptr);

		hinge->target_velocity = target_velocity;

		if (hinge->constraint.a != nullptr) {
			hinge->constraint.a->alertWakingAction();
			hinge->constraint.b->alertWakingAction();
		}
	}

	double PhysicsEngine::getMotorAngularPosition(ConstraintID id) {
		assert(id.type == ConstraintID::HINGE && constraint_map.find(id.uniqueID) != constraint_map.end());
		Hinge* hinge = nullptr;
		for (Hinge* h : constraint_map[id.uniqueID]->hingeConstraints) {
			if (h->uniqueID == id.uniqueID) {
				hinge = h;
				break;
			}
		}
		assert(hinge != nullptr);

		return hinge->motor_angular_position;
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
		SharedConstraintsEdge* c = new SharedConstraintsEdge(this, n2);
		constraints.push_back(c);
		n2->constraints.push_back(c);
		return c;
	}
	
}

	