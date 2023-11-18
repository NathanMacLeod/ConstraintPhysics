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

	//PSUEDO_VELOCITY: position correction is done by calcualting a bonus velocity to push constrained objects back to valid positions
	//MASTER_SLAVE: The master body (designed by master_b1 bool) dictates the correct position, and the slave object is translated to it. 
	//Psuedo velocities are still calculated and applied for the sake of other constraints that interact with the master/slave pair.
	enum PosErrorResolutionMode { PSUEDO_VELOCITY, MASTER_SLAVE };

	struct ConstraintID {
		ConstraintID() : uniqueID(-1) {}
		enum Type { BALL, HINGE, SLIDER, SPRING, SLIDING_HINGE, WELD };
		inline Type getType() { return type; }

		friend class PhysicsEngine;
	private:
		ConstraintID(Type type, int id) : type(type), uniqueID(id) {}

		Type type;
		int uniqueID;
	};

	struct RayHitInfo {
		bool did_hit;
		RigidBody* hit_object;
		mthz::Vec3 hit_position;
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
		RigidBody* createRigidBody(const ConvexUnionGeometry& geometry, RigidBody::MovementType movement_type=RigidBody::DYNAMIC, mthz::Vec3 position=mthz::Vec3(), mthz::Quaternion orientation=mthz::Quaternion(), bool override_center_of_mass=false, mthz::Vec3 center_of_mass_override=mthz::Vec3());
		RigidBody* createRigidBody(const StaticMeshGeometry& geometry, bool fixed=true);
		void removeRigidBody(RigidBody* r);
		void applyVelocityChange(RigidBody* b, const mthz::Vec3& delta_vel, const mthz::Vec3& delta_ang_vel, const mthz::Vec3& delta_psuedo_vel=mthz::Vec3(), const mthz::Vec3&delta_psuedo_ang_vel=mthz::Vec3());
		void disallowCollisionSet(const std::initializer_list<RigidBody*>& bodies);
		void reallowCollisionSet(const std::initializer_list<RigidBody*>& bodies);
		void disallowCollision(RigidBody* b1, RigidBody* b2);
		bool collisionAllowed(RigidBody* b1, RigidBody* b2);
		void reallowCollision(RigidBody* b1, RigidBody* b2);
		
		RayHitInfo raycastFirstIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir, std::vector<RigidBody*> ignore_list = std::vector<RigidBody*>());

		inline int getNumBodies() { return bodies.size(); }
		inline mthz::Vec3 getGravity() { return gravity; }
		inline double getStep_time() { return step_time; }
		std::vector<RigidBody*> getBodies();
		unsigned int getNextBodyID();

		void setPGSIterations(int n_vel, int n_pos) { pgsVelIterations = n_vel; pgsPosIterations = n_pos; }
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

		//really only exists for debugging
		void deleteWarmstartData(RigidBody* r);
		void createDebugHolonomicSystem(mthz::Vec3 pos);

		//should probably be placed with a different scheme eventually. This is for when position/orientation is changed for a rigid body, but AABB needs to be updated before next physics tick for use by querying raycasts.
		void forceAABBTreeUpdate();
		
		ColActionID registerCollisionAction(CollisionTarget b1, CollisionTarget b2, const ColAction& action);
		void removeCollisionAction(ColActionID action_key);

		ConstraintID addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double pos_correct_strength=350);
		ConstraintID addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local,
			double min_angle = -std::numeric_limits<double>::infinity(), double max_angle = std::numeric_limits<double>::infinity(), double pos_correct_strength = 350, double rot_correct_strength = 350);

		ConstraintID addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local, 
			double negative_slide_limit = -std::numeric_limits<double>::infinity(), double positive_slide_limit = std::numeric_limits<double>::infinity(), double pos_correct_strength = 350, double rot_correct_strength=350);

		ConstraintID addSlidingHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local,
			double negative_slide_limit = -std::numeric_limits<double>::infinity(), double positive_slide_limit = std::numeric_limits<double>::infinity(), double min_angle = -std::numeric_limits<double>::infinity(), 
			double max_angle = std::numeric_limits<double>::infinity(), double pos_correct_strength = 350, double rot_correct_strength = 350);

		ConstraintID addWeldConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_point_local, mthz::Vec3 b2_attach_point_local, double pos_correct_strength = 350, double rot_correct_strength = 350);


		ConstraintID addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, double pos_correct_strength = 350);
		ConstraintID addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, mthz::Vec3 rot_axis_local, double min_angle = -std::numeric_limits<double>::infinity(), 
			double max_angle = std::numeric_limits<double>::infinity(), double pos_correct_strength = 350, double rot_correct_strength = 350);

		ConstraintID addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double negative_slide_limit = -std::numeric_limits<double>::infinity(), 
			double positive_slide_limit = std::numeric_limits<double>::infinity(), double pos_correct_strength = 350, double rot_correct_strength = 350);

		ConstraintID addSlidingHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double negative_slide_limit = -std::numeric_limits<double>::infinity(),
			double positive_slide_limit = std::numeric_limits<double>::infinity(), double min_angle = -std::numeric_limits<double>::infinity(), double max_angle = std::numeric_limits<double>::infinity(), 
			double pos_correct_strength = 350, double rot_correct_strength = 350);

		ConstraintID addSpring(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double damping, double stiffness, double resting_length = -1);
		ConstraintID addWeldConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_point_local, double pos_correct_strength = 350, double rot_correct_strength = 350);

		void setConstraintPosCorrectMethod(ConstraintID id, PosErrorResolutionMode error_correct_method, bool b1_master = true);
		void setConstraintUseCustomCFM(ConstraintID id, double custom_cfm);
		void setConstraintUseGlobalCFM(ConstraintID id);
		void removeConstraint(ConstraintID id, bool reenable_collision=true);

		void setMotorOff(ConstraintID id);
		void setMotorConstantTorque(ConstraintID id, double torque);
		void setMotorTargetVelocity(ConstraintID id, double max_torque, double target_velocity);
		void setMotorTargetPosition(ConstraintID id, double max_torque, double target_position);

		void setPiston(ConstraintID id, double max_force, double target_velocity);
		double getMotorAngularPosition(ConstraintID id);

		//just makes debugging easier
		friend class DebugDemo;
	private:

		static int n_threads;
		static ThreadManager thread_manager;
		static bool use_multithread;
		static bool print_performance_data;

		unsigned int next_id = 1;

		int pgsVelIterations = 20;
		int pgsPosIterations = 15;


		BroadPhaseStructure broadphase = AABB_TREE;
		double aabbtree_margin_size = 0.1;
		AABBTree<RigidBody*> aabb_tree = AABBTree<RigidBody*>(aabbtree_margin_size);
		mthz::Vec3 octree_center = mthz::Vec3(0, 0, 0);
		double octree_size = 2000;
		double octree_minsize = 1;
	

		int angle_velocity_update_tick_count = 4;
		bool is_internal_gyro_forces_disabled = false;
		bool friction_impulse_limit_enabled = false;

		struct ConstraintGraphNode; 
		void addContact(ConstraintGraphNode* n1, ConstraintGraphNode* n2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness, CFM cfm);
		void maintainConstraintGraphApplyPoweredConstraints();
		void bfsVisitAll(ConstraintGraphNode* curr, std::set<ConstraintGraphNode*>* visited, void* in, std::function<void(ConstraintGraphNode* curr, void* in)> action);
		inline double getCutoffVel(double step_time, const mthz::Vec3& gravity) { return 2 * gravity.mag() * step_time; }
		bool bodySleepy(RigidBody* r);
		void deleteRigidBody(RigidBody* r);
		bool readyToSleep(RigidBody* b);
		void wakeupIsland(ConstraintGraphNode* foothold);

		inline double posCorrectCoeff(double pos_correct_strength, double step_time) { return std::min<double>(pos_correct_strength * step_time, 1.0 / step_time); }

		std::vector<RigidBody*> bodies;
		std::vector<RigidBody*> bodies_to_delete;
		std::vector<HolonomicSystem> holonomic_systems;
		mthz::Vec3 gravity;
		double step_time = 1.0 / 90;
		double cutoff_vel = 0;
		int contact_life = 6;
		double contact_pos_correct_hardness = 350;
		bool sleeping_enabled = true;
		double sleep_delay = 0.5;
		int non_sleepy_tick_threshold = 3;
		
		double vel_sleep_coeff = 0.1;
		double ang_vel_eps = 0.1;
		double accel_sleep_coeff = 0.044;

		bool warm_start_disabled = false;

		//Constraint Graph
		std::unordered_map<unsigned int, ConstraintGraphNode*> constraint_graph_nodes;
		std::mutex constraint_graph_lock;

		//constraint force mixing - softens constraints and makes more stable
		double global_cfm = 0.025;
		
		struct Contact {
			RigidBody* b1;
			RigidBody* b2;
			ContactConstraint contact;
			FrictionConstraint friction;
			CFM cfm;
			MagicID magic;
			int memory_life; //how many frames can be referenced for warm starting before is deleted
			bool is_live_contact;
		};

		struct BallSocket {
			RigidBody* b1;
			RigidBody* b2;
			BallSocketConstraint constraint;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			CFM cfm;
			double pos_correct_hardness;
			PosErrorResolutionMode pos_error_mode;
			bool b1_master;
			int uniqueID;
		};

		enum MotorMode { OFF, CONST_TORQUE, TARGET_VELOCITY, TARGET_POSITION };

		struct Motor {
			Motor(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double min_angle, double max_angle);
			double calculatePosition(mthz::Vec3 rot_axis, mthz::Vec3 ang_vel_b1, mthz::Vec3 ang_vel_b2, double timestep);
			double getConstraintTargetVelocityValue(mthz::Vec3 rot_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double step_time);
			void writePrevVel(mthz::Vec3 rot_axis, mthz::Vec3 ang_vel_b1, mthz::Vec3 ang_vel_b2);
			bool constraintIsActive();

			RigidBody* b1;
			RigidBody* b2;
			MotorConstraint motor_constraint;
			mthz::Vec3 b1_u_axis_reference;
			mthz::Vec3 b1_w_axis_reference;
			mthz::Vec3 b2_rot_comparison_axis;
			MotorMode mode;
			double motor_angular_position;
			double min_motor_position;
			double max_motor_position;
			double max_torque;
			double target_velocity;
			double target_position;
			double prev_velocity;
		};

		struct Hinge {
			RigidBody* b1;
			RigidBody* b2;
			HingeConstraint constraint;
			Motor motor;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			mthz::Vec3 b1_rot_axis_body_space;
			mthz::Vec3 b2_rot_axis_body_space;
			CFM cfm;
			double pos_correct_hardness;
			double rot_correct_hardness;
			PosErrorResolutionMode pos_error_mode;
			bool b1_master;
			int uniqueID;
		};

		struct Slider {
			RigidBody* b1;
			RigidBody* b2;
			SliderConstraint constraint;
			SlideLimitConstraint slide_limit;
			PistonConstraint piston_force;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			mthz::Vec3 b1_slide_axis_body_space;
			mthz::Vec3 b2_slide_axis_body_space;
			double max_piston_force;
			double target_velocity;
			CFM cfm;
			double pos_correct_hardness;
			double rot_correct_hardness;
			double positive_slide_limit;
			double negative_slide_limit;
			bool slide_limit_exceeded;
			PosErrorResolutionMode pos_error_mode;
			bool b1_master;
			int uniqueID;
		};

		struct SlidingHinge {
			RigidBody* b1;
			RigidBody* b2;
			SlidingHingeConstraint constraint;
			SlideLimitConstraint slide_limit;
			PistonConstraint piston_force;
			Motor motor;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			mthz::Vec3 b1_slide_axis_body_space;
			mthz::Vec3 b2_slide_axis_body_space;
			double max_piston_force;
			double target_velocity;
			CFM cfm;
			double pos_correct_hardness;
			double rot_correct_hardness;
			double positive_slide_limit;
			double negative_slide_limit;
			bool slide_limit_exceeded;
			PosErrorResolutionMode pos_error_mode;
			bool b1_master;
			int uniqueID;
		};

		struct Weld {
			RigidBody* b1;
			RigidBody* b2;
			WeldConstraint constraint;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			CFM cfm;
			double pos_correct_hardness;
			double rot_correct_hardness;
			PosErrorResolutionMode pos_error_mode;
			bool b1_master;
			int uniqueID;
		};

		struct Spring {
			RigidBody* b1;
			RigidBody* b2;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			double stiffness;
			double damping;
			double resting_length;
			int uniqueID;
		};

		struct ActiveConstraintData {
			std::vector<std::vector<Constraint*>> island_systems;
			std::vector<BallSocket*> mast_slav_bss;
			std::vector<Hinge*> mast_slav_hs;
			std::vector<Slider*> mast_slav_ss;
			std::vector<SlidingHinge*> mast_slav_shs;
			std::vector<Weld*> mast_slav_ws;
		};

		ActiveConstraintData sleepOrSolveIslands();
		void applyMasterSlavePosCorrect(const ActiveConstraintData& a);

		struct HolonomicSystemNodes;

		struct SharedConstraintsEdge {
			SharedConstraintsEdge(ConstraintGraphNode* n1, ConstraintGraphNode* n2) : n1(n1), n2(n2), contactConstraints(std::vector<Contact*>()) {}
			~SharedConstraintsEdge();

			ConstraintGraphNode* n1;
			ConstraintGraphNode* n2;
			HolonomicSystemNodes* h = nullptr;
			bool holonomic_system_scan_needed = false;
			std::vector<Contact*> contactConstraints;
			std::vector<BallSocket*> ballSocketConstraints;
			std::vector<Hinge*> hingeConstraints;
			std::vector<Slider*> sliderConstraints;
			std::vector<SlidingHinge*> slidingHingeConstraints;
			std::vector<Weld*> weldConstraints;
			std::vector<Spring*> springs;

			int visited_tag = 0;

			inline ConstraintGraphNode* other(ConstraintGraphNode* c) { return (c->b == n1->b ? n2 : n1); }
			bool noConstraintsLeft() { 
				return contactConstraints.empty()
					&& ballSocketConstraints.empty()
					&& hingeConstraints.empty()
					&& sliderConstraints.empty()
					&& slidingHingeConstraints.empty()
					&& weldConstraints.empty()
					&& springs.empty();
			}

			bool hasHolonomicConstraint() {
				return !(ballSocketConstraints.empty()
					  && hingeConstraints.empty()
					  && sliderConstraints.empty()
					  && slidingHingeConstraints.empty()
					  && weldConstraints.empty());
			}
		};

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

		Motor* fetchMotor(ConstraintID id);

		int nextConstraintID = 0;
		std::unordered_map<int, SharedConstraintsEdge*> constraint_map;

		ColActionID nextActionID = 0;
		inline static unsigned int all() { return 0; }
		std::unordered_map<unsigned int, std::unordered_map<unsigned int, std::vector<ColActionID>>> get_action_map;
		std::unordered_map<ColActionID, ColAction> col_actions;
	};

}