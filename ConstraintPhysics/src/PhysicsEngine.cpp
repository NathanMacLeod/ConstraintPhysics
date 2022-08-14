#include "PhysicsEngine.h"
#include "RigidBody.h"
#include "Octree.h"
#include "ThreadManager.h"
#include <vector>
#include <algorithm>

//debug
#include <chrono>

static const double FLATTENING_BIAS_MAG = 0;
static const double FLAT_ENOUGH = cos(3.1415926535 * 0.1 / 180.0);

namespace phyz {

	double parallel_t = 0;
	double single_t = 0;

	void PhysicsEngine::timeStep() {

		Octree<RigidBody> octree(mthz::Vec3(0, 0, 0), 1000, 1);

		int n_asleep = 0;
		int n_not_asleep = 0;
		int n_fixed = 0;
		for (RigidBody* b : bodies) {
			octree.insert(*b, b->aabb);
			if (!b->asleep && !b->fixed) {
				b->recordMovementState(sleep_delay / step_time);
				if (bodySleepy(b->history)) {
					b->sleep_ready_counter += step_time;
				}
				else {
					b->sleep_ready_counter = 0;
				}
				
				if (!b->fixed) {
					b->updateGeometry();
					b->vel += gravity * step_time;
					b->applyGyroAccel(step_time);
				}
			}
		}

		maintainConstraintGraph();

		std::vector<Octree<RigidBody>::Pair> possible_intersections = octree.getAllIntersections();

		//auto t1 = std::chrono::system_clock::now();
		thread_manager.do_all<Octree<RigidBody>::Pair>(8, possible_intersections,
			[&](const Octree<RigidBody>::Pair& p) {
				RigidBody* b1 = p.t1;
				RigidBody* b2 = p.t2;

				if ((b1->fixed || b1->asleep) && (b2->fixed || b2->asleep)) {
					return;
				}
				else if (!collisionAllowed(b1, b2)) {
					return;
				}

				std::vector<Manifold> manifolds;
				for (int i = 0; i < b1->geometry.size(); i++) {
					for (int j = 0; j < b2->geometry.size(); j++) {
						const ConvexPoly& c1 = b1->geometry[i];
						const ConvexPoly& c2 = b2->geometry[j];
						if ((b1->geometry.size() > 1 || b2->geometry.size() > 1) && !AABB::intersects(b1->geometry_AABB[i], b2->geometry_AABB[j])) {
							continue;
						}

						Manifold man = SAT(c1, b1->gauss_maps[i], c2, b2->gauss_maps[j]);

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
						for (int j = i+1; j < manifolds.size(); j++) {
							if (!merged[j]) {
								double v = manifolds[i].normal.dot(manifolds[j].normal);
								if (1 - v < COS_TOL) {
									manifolds[i] = merge_manifold(manifolds[i], manifolds[j]);
								}
								merged[j] = true;
							}
						}
						Manifold man = cull_manifold(manifolds[i], 4);

						mthz::Vec3 u, w;
						man.normal.getPerpendicularBasis(&u, &w);

						constraint_graph_lock.lock();
						for (int i = 0; i < man.points.size(); i++) {
							const ContactP& p = man.points[i];
							addContact(b1, b2, p.pos, man.normal, p.magicID, 0.3, 1.1, 0.5, man.points.size(), p.pen_depth, posCorrectCoeff(270, step_time));
						}
						constraint_graph_lock.unlock();
					}
				}
			}
		);
		
		std::vector<std::vector<Constraint*>> island_systems = sleepOrSolveIslands();

		thread_manager.do_all<std::vector<Constraint*>>(8, island_systems,
			[&](const std::vector<Constraint*>& island_system) {
				PGS_solve(this, island_system);
			}
		);

		for (RigidBody* b : bodies) {
			if (!b->fixed && !b->asleep) {
				b->com += (b->vel + b->psuedo_vel) * step_time;
				if (b->ang_vel.magSqrd() != 0) {
					mthz::Vec3 rot = b->ang_vel + b->psuedo_ang_vel;
					b->orientation = mthz::Quaternion(step_time * rot.mag(), rot) * b->orientation;
				}

				b->psuedo_vel = mthz::Vec3(0, 0, 0);
				b->psuedo_ang_vel = mthz::Vec3(0, 0, 0);
			}
		}
	}

	RigidBody* PhysicsEngine::createRigidBody(const std::vector<ConvexPoly>& geometry, bool fixed, double density) {
		RigidBody* r = new RigidBody(geometry, density);
		r->fixed = fixed;
		bodies.push_back(r);
		constraint_graph_nodes[r] = new ConstraintGraphNode(r);
		return r;
	}

	void PhysicsEngine::disallowCollision(RigidBody* b1, RigidBody* b2) {
		b1->no_collision_set.insert(b2);
		b2->no_collision_set.insert(b1);
	}

	bool PhysicsEngine::collisionAllowed(RigidBody* b1, RigidBody* b2) {
		return b1->no_collision_set.find(b2) == b1->no_collision_set.end();
	}

	void PhysicsEngine::reallowCollision(RigidBody* b1, RigidBody* b2) {
		auto i = b1->no_collision_set.find(b2);
		if (i != b1->no_collision_set.end()) {
			b1->no_collision_set.erase(i);
			b2->no_collision_set.erase(b2->no_collision_set.find(b1));
		}
	}

	void PhysicsEngine::addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 ball_socket_position, double pos_correct_strength) {
		disallowCollision(b1, b2);
		BallSocket* bs = new BallSocket{
			BallSocketConstraint(b1, b2, ball_socket_position, mthz::Vec3(), pos_correct_strength * step_time),
			b1->track_point(ball_socket_position),
			b2->track_point(ball_socket_position),
			pos_correct_strength
		};

		ConstraintGraphNode* n1 = constraint_graph_nodes[b1];
		ConstraintGraphNode* n2 = constraint_graph_nodes[b2];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		e->ballSocketConstraints.push_back(bs);
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

	mthz::Vec3 PhysicsEngine::getGravity() {
		return gravity;
	}

	void PhysicsEngine::setGravity(const mthz::Vec3& v) {
		gravity = v;
		cutoff_vel = getCutoffVel(step_time, gravity);
	}

	double PhysicsEngine::getStep_time() {
		return step_time;
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

		mthz::Vec3 average_pos;
		mthz::Vec3 average_ang_vel;

		for (const RigidBody::MovementState& s : body_history) {
			average_pos += s.position;
			average_ang_vel += s.ang_vel;
		}
		
		
		average_pos /= n;
		average_ang_vel /= n;
		mthz::Vec3 average_accel = (body_history.back().vel - body_history.front().vel) / (step_time * (n - 1));
		mthz::Vec3 average_ang_accel = (body_history.back().ang_vel - body_history.front().ang_vel) / (step_time * (n - 1));

		double vel_eps = vel_sleep_coeff * gravity.mag();
		double accel_eps = accel_sleep_coeff * gravity.mag();
		return average_pos.mag() <= vel_eps && average_ang_vel.mag() <= vel_eps && average_accel.mag() <= accel_eps && average_ang_accel.mag() <= accel_eps;
	}

	bool PhysicsEngine::readyToSleep(RigidBody* b) {
		return b->sleep_ready_counter >= sleep_delay || b->fixed;
	}

	//Fixed bodies need to be treated a little weird. They should not bridge islands together (two seperate piles on the same floor should be different islands).
	//Thus fixed bodies exist only as leaves, and can't be 'curr' in initial call to dfs
	void PhysicsEngine::dfsVisitAll(ConstraintGraphNode* curr, std::set<ConstraintGraphNode*>* visited, void* in, std::function<void(ConstraintGraphNode* curr, void* in)> action) {
		if (curr->b->fixed) {
			printf("ERROR: dfs should not visit fixed nodes\n");
		}
		visited->insert(curr);
		action(curr, in);
		for (SharedConstraintsEdge* e : curr->constraints) {
			ConstraintGraphNode* n = e->other(curr);
			if (!n->b->fixed && visited->find(n) == visited->end()) {
				dfsVisitAll(n, visited, in, action);
			}
		}
	}

	void PhysicsEngine::wakeupIsland(ConstraintGraphNode* foothold) {
		std::set<ConstraintGraphNode*> visited;
		dfsVisitAll(foothold, &visited, nullptr, [this](ConstraintGraphNode* curr, void* in) {
			curr->b->wake();
		});
	}
	void PhysicsEngine::addContact(RigidBody* b1, RigidBody* b2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness) {
		ConstraintGraphNode* n1 = constraint_graph_nodes[b1]; ConstraintGraphNode* n2 = constraint_graph_nodes[b2];
		SharedConstraintsEdge* e = n1->getOrCreateEdgeTo(n2);

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);

		for (Contact* c : e->contactConstraints) {
			if (c->magic == magic) {
				double friction = (c->friction1.getStaticReady() && c->friction2.getStaticReady()) ? static_friction : kinetic_friction;

				c->contact = ContactConstraint(b1, b2, norm, p, bounce, pen_depth, hardness, c->contact.impulse, cutoff_vel);
				c->friction1 = FrictionConstraint(b1, b2, u, p, friction, n_points, &c->contact, c->friction1.impulse);
				c->friction2 = FrictionConstraint(b1, b2, w, p, friction, n_points, &c->contact, c->friction2.impulse);
				c->memory_life = contact_life;
				c->is_live_contact = true;
				return;
			}
		}

		//if no warm start existed
		Contact* c = new Contact();
		c->contact = ContactConstraint(b1, b2, norm, p, bounce, pen_depth, hardness, NVec<1>{0.0}, cutoff_vel);
		c->friction1 = FrictionConstraint(b1, b2, u, p, kinetic_friction, n_points, &c->contact, NVec<1>{0.0});
		c->friction2 = FrictionConstraint(b1, b2, w, p, kinetic_friction, n_points, &c->contact, NVec<1>{0.0});
		c->magic = magic;
		c->memory_life = contact_life;
		c->is_live_contact = true;
		
		e->contactConstraints.push_back(c);
	}

	void PhysicsEngine::maintainConstraintGraph() {
		for (const auto& kv_pair : constraint_graph_nodes) {
			ConstraintGraphNode* n = kv_pair.second;
			for (auto i = n->constraints.begin(); i != n->constraints.end();) {
				SharedConstraintsEdge* e = *i;
				for (auto j = e->contactConstraints.begin(); j != e->contactConstraints.end();) {
					Contact* contact = *j;
					contact->is_live_contact = false;
					if (!e->n1->b->asleep && !e->n2->b->asleep && contact->memory_life-- < 0) {
						delete contact;
						j = e->contactConstraints.erase(j);
					}
					else {
						j++;
					}
				}
				for (BallSocket* bs: e->ballSocketConstraints) {
					//update constraint for new positions
					RigidBody* b1 = e->n1->b;
					RigidBody* b2 = e->n2->b;
					mthz::Vec3 b1_pos = b1->getTrackedP(bs->b1_point);
					mthz::Vec3 b2_pos = b2->getTrackedP(bs->b2_point);
					bs->constraint = BallSocketConstraint(b1, b2, b1_pos, b2_pos - b1_pos, bs->pos_correct_hardness*step_time, bs->constraint.impulse);
				}
				if (e->noConstraintsLeft()) {
					ConstraintGraphNode* reciprocal = e->other(n);
					reciprocal->constraints.erase(std::remove(reciprocal->constraints.begin(), reciprocal->constraints.end(), e));
					delete e;
					i = n->constraints.erase(i);
				}
				else {
					i++;
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
			dfsVisitAll(n, &visited, (void*)&in, [&visited, new_contact_life, this](ConstraintGraphNode* curr, void* in) {
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
								output->island_constraints->push_back(&c->friction1);
								output->island_constraints->push_back(&c->friction2);
							}
						}
						for (BallSocket* bs : e->ballSocketConstraints) {
							output->island_constraints->push_back(&bs->constraint);
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

	