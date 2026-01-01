#pragma once
#include "RigidBody.h"
#include "CollisionDetect.h"
#include "ConstraintSolver.h"
#include "HolonomicBlockSolver.h"
#include "ThreadManager.h"
#include "Octree.h"
#include "AABB_Tree.h"
#include <set>
#include <functional>
#include <unordered_map>
#include <mutex>
#include "PersistentConstraint.h"

class DebugDemo;

namespace phyz {

	class PhysicsEngine;

	struct CollisionTarget {
	public:
		static CollisionTarget all();
		static CollisionTarget with(RigidBody* r);

		friend class PhysicsEngine;
	private:
		CollisionTarget(bool with_all, unsigned int specific_target) : with_all(with_all), specific_target(specific_target) {}

		bool with_all;
		unsigned int specific_target;
	};

	typedef int ColActionID;
	typedef std::function<void(RigidBody* b1, RigidBody* b2, const std::vector<Manifold>& manifold)> ColAction;

	struct RayHitInfo {
		bool did_hit;
		RigidBody* hit_object;
		mthz::Vec3 hit_position;
		mthz::Vec3 surface_normal;
		double hit_distance;
	};

	enum BroadPhaseStructure { NONE, OCTREE, AABB_TREE, TEST_COMPARE };

	class PhysicsEngine {
	public:
		~PhysicsEngine();

		static void enableMultithreading(int n_threads);
		static void disableMultithreading();
		static void setPrintPerformanceData(bool print_data);

		void timeStep();
		void extrapolateObjectPositions(double time_elapsed);
		RigidBody* createRigidBody(const ConvexUnionGeometry& geometry, RigidBody::MovementType movement_type=RigidBody::DYNAMIC, mthz::Vec3 position=mthz::Vec3(), mthz::Quaternion orientation=mthz::Quaternion(), bool override_center_of_mass=false, mthz::Vec3 center_of_mass_override=mthz::Vec3());
		RigidBody* createRigidBody(const StaticMeshGeometry& geometry, bool fixed=true);
		void removeRigidBody(RigidBody* r);
		void applyVelocityChange(RigidBody* b, const mthz::Vec3& delta_vel, const mthz::Vec3& delta_ang_vel, const mthz::Vec3& delta_psuedo_vel=mthz::Vec3(), const mthz::Vec3&delta_psuedo_ang_vel=mthz::Vec3());
		void disallowCollisionSet(const std::initializer_list<RigidBody*>& bodies);
		void disallowCollisionSet(const std::vector<RigidBody*>& bodies);
		void reallowCollisionSet(const std::initializer_list<RigidBody*>& bodies);
		void reallowCollisionSet(const std::vector<RigidBody*>& bodies);
		void disallowCollision(RigidBody* b1, RigidBody* b2);
		bool collisionAllowed(RigidBody* b1, RigidBody* b2);
		void reallowCollision(RigidBody* b1, RigidBody* b2);
		
		RayHitInfo raycastFirstIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir, std::vector<RigidBody*> ignore_list = std::vector<RigidBody*>()) const;

		inline uint32_t getNumBodies() { return static_cast<uint32_t>(bodies.size()); }
		inline mthz::Vec3 getGravity() { return gravity; }
		inline double getStep_time() { return step_time; }
		std::vector<RigidBody*> getBodies();
		unsigned int getNextBodyID();

		void setPGSIterations(int n_vel, int n_pos, int n_holonomic = 3) { pgsVelIterations = n_vel; pgsPosIterations = n_pos; pgsHolonomicIterations = 0;/* n_holonomic;*/ }
		void setSleepingEnabled(bool sleeping);
		void setStep_time(double d);
		void setGravity(const mthz::Vec3& v);
		void setBroadphase(BroadPhaseStructure b);
		void setAABBTreeMarginSize(double d);
		void setOctreeParams(double size, double minsize, mthz::Vec3 center = mthz::Vec3(0, 0, 0));
		void setAngleVelUpdateTickCount(int n);
		void setInternalGyroscopicForcesDisabled(bool b);
		void setWarmStartDisabled(bool b);
		void setSleepParameters(double vel_sensitivity, double ang_vel_sensitivity, double aceleration_sensitivity, double sleep_assesment_time, int non_sleepy_tick_threshold);
		void setGlobalConstraintForceMixing(double cfm);
		void setHolonomicSolverCFM(double cfm);

		//really only exists for debugging
		void deleteWarmstartData(RigidBody* r);

		//should probably be placed with a different scheme eventually. This is for when position/orientation is changed for a rigid body, but AABB needs to be updated before next physics tick for use by querying raycasts.
		void forceAABBTreeUpdate();
		
		ColActionID registerCollisionAction(CollisionTarget b1, CollisionTarget b2, const ColAction& action);
		void removeCollisionAction(ColActionID action_key);

		ConstraintID addDistanceConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double target_distance=-1, double pos_correct_strength=1000);
		ConstraintID addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double pos_correct_strength = 1000);
		ConstraintID addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);
		ConstraintID addMotorConstraint(ConstraintID base_constraint, double min_angle = -std::numeric_limits<double>::infinity(), double max_angle = std::numeric_limits<double>::infinity());
		ConstraintID addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);
		ConstraintID addPistonConstraint(ConstraintID base_constraint, double negative_slide_limit = -std::numeric_limits<double>::infinity(), double positive_slide_limit = std::numeric_limits<double>::infinity());
		ConstraintID addSlidingHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);

		ConstraintID addWeldConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_point_local, mthz::Vec3 b2_attach_point_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);


		ConstraintID addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, double pos_correct_strength = 1000);
		ConstraintID addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, mthz::Vec3 rot_axis_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);

		ConstraintID addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);

		ConstraintID addSlidingHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);

		ConstraintID addSpring(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double damping, double stiffness, double resting_length = -1);
		ConstraintID addWeldConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_point_local, double pos_correct_strength = 1000, double rot_correct_strength = 1000);

		void setConstraintUseCustomCFM(ConstraintID id, double custom_cfm);
		void setConstraintUseGlobalCFM(ConstraintID id);
		void removeConstraint(ConstraintID id, bool reenable_collision=true);

		void setMotorOff(ConstraintID id);
		void setMotorConstantTorque(ConstraintID id, double torque);
		void setMotorTargetVelocity(ConstraintID id, double max_torque, double target_velocity);
		void setMotorTargetPosition(ConstraintID id, double max_torque, double target_position);

		void setPistonOff(ConstraintID id);
		void setPistonConstantForce(ConstraintID id, double force);
		void setPistonTargetVelocity(ConstraintID id, double max_force, double target_velocity);
		void setPistonTargetPosition(ConstraintID id, double max_force, double target_position);

		void setPiston(ConstraintID id, double max_force, double target_velocity);
		double getMotorAngularPosition(ConstraintID id);
		double getPistonPosition(ConstraintID id);

		void setDistanceConstraintTargetDistance(ConstraintID id, double target_distance);
		double getDistanceConstraintTargetDistance(ConstraintID id);

		//just makes debugging easier
		friend class DebugDemo;
	private:

		static int n_threads;
		static ThreadManager thread_manager;
		static bool use_multithread;
		static bool print_performance_data;

		unsigned int next_id = 1;

		int pgsVelIterations = 1;
		int pgsPosIterations = 1;
		int sub_itr_count = 8;
		int pgsHolonomicIterations = 0;// 1;
		bool using_holonomic_system_solver() { return pgsHolonomicIterations > 0; }

		BroadPhaseStructure broadphase = AABB_TREE;
		double aabbtree_margin_size = 0.1;
		AABBTree<RigidBody*> aabb_tree = AABBTree<RigidBody*>(aabbtree_margin_size);
		mthz::Vec3 octree_center = mthz::Vec3(0, 0, 0);
		double octree_size = 2000;
		double octree_minsize = 1;
	
		double holonomic_block_solver_CFM = 0.00001;
		bool compute_holonomic_inverse_in_parallel = true;

		int angle_velocity_update_tick_count = 4;
		bool is_internal_gyro_forces_disabled = false;
		bool friction_impulse_limit_enabled = false;

		struct ConstraintGraphNode; 
		void addContact(ConstraintGraphNode* n1, ConstraintGraphNode* n2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness);
		void maintainConstraintGraphApplyPoweredConstraints(bool is_first_sub_itr, bool is_last_sub_itr, double delta_time);
		//void updateConstraints(std::vector<Constraint*> constraints);
		void bfsVisitAll(ConstraintGraphNode* curr, std::set<ConstraintGraphNode*>* visited, void* in, std::function<void(ConstraintGraphNode* curr, void* in)> action);
		inline double getCutoffVel(double step_time, const mthz::Vec3& gravity) { return 2 * gravity.mag() * step_time; }
		bool bodySleepy(RigidBody* r);
		void deleteRigidBody(RigidBody* r);
		bool readyToSleep(RigidBody* b);
		void wakeupIsland(ConstraintGraphNode* foothold);

		std::vector<RigidBody*> bodies;
		std::vector<RigidBody*> bodies_to_delete;
		std::vector<HolonomicSystem> holonomic_systems;
		mthz::Vec3 gravity = mthz::Vec3(0, -6, 0);
		double step_time = 1.0 / 90;
		double cutoff_vel = 0;
		int contact_life = 6;
		double contact_pos_correct_hardness = 1000;
		bool sleeping_enabled = true;
		double sleep_delay = 0.5;
		int non_sleepy_tick_threshold = 3;
		
		double vel_sleep_coeff = 0.1;
		double ang_vel_eps = 0.1;
		double accel_sleep_coeff = 0.044;

		bool warm_start_disabled = false;
		double warm_start_coefficient = 0.975;

		//Constraint Graph
		std::unordered_map<unsigned int, ConstraintGraphNode*> constraint_graph_nodes;
		std::mutex constraint_graph_lock;

		//constraint force mixing - softens constraints and makes more stable
		double global_cfm = 0;// 0.025;

		struct IslandConstraints {
			std::vector<Constraint*> constraints;
			std::vector<HolonomicSystem*> systems;
		};

		struct ActiveConstraintData {
			std::vector<IslandConstraints> island_systems;
		};

		ActiveConstraintData sleepOrSolveIslands();

		struct HolonomicSystemNodes;

		struct SharedConstraintsEdge {
			SharedConstraintsEdge(ConstraintGraphNode* n1, ConstraintGraphNode* n2) : n1(n1), n2(n2) {}
			~SharedConstraintsEdge();

			ConstraintGraphNode* n1;
			ConstraintGraphNode* n2;
			HolonomicSystemNodes* h = nullptr;
			bool holonomic_system_scan_needed = false;
			std::vector<PersistentConstraint*> constraints;

			int visited_tag = 0;

			inline ConstraintGraphNode* other(ConstraintGraphNode* c) { return (c->b == n1->b ? n2 : n1); }
			bool noConstraintsLeft() { 
				return constraints.empty();
			}

			bool hasHolonomicConstraint() {
				for (PersistentConstraint* c : constraints) {
					if (c->isHolonomicConstraint()) return true;
				}
				return false;
			}
		};

		void removeExpiredContactConstraints(SharedConstraintsEdge* e);

		struct HolonomicSystemNodes {
			HolonomicSystemNodes(std::vector<SharedConstraintsEdge*> member_edges);
			void recalculateSystem();

			bool constraints_changed_flag;
			bool edge_removed_flag;
			std::vector<SharedConstraintsEdge*> member_edges;
			HolonomicSystem system;
		};

		std::vector<SharedConstraintsEdge*> getAllEdgesConnectedHolonomically(SharedConstraintsEdge* e);
		void shatterFracturedHolonomicSystems();
		void maintainAllHolonomicSystemStuffRelatedToThisEdge(SharedConstraintsEdge* e);

		struct ConstraintGraphNode {
			ConstraintGraphNode(RigidBody* b) : b(b), constraints(std::vector<SharedConstraintsEdge*>()) {}
			~ConstraintGraphNode();

			RigidBody* b;
			std::vector<SharedConstraintsEdge*> constraints;
			std::mutex mutex;

			SharedConstraintsEdge* getOrCreateEdgeTo(ConstraintGraphNode* n2);
		private:
			void insertNewEdge(SharedConstraintsEdge* e);
		};

		PersistentConstraint* getConstraint(ConstraintID id);

		int nextConstraintID = 0;
		std::unordered_map<int, SharedConstraintsEdge*> constraint_map;

		ColActionID nextActionID = 0;
		inline static unsigned int all() { return 0; }
		std::unordered_map<unsigned int, std::unordered_map<unsigned int, std::vector<ColActionID>>> get_action_map;
		std::unordered_map<ColActionID, ColAction> col_actions;
	};

}