#pragma once
#include "RigidBody.h"
#include "CollisionDetect.h"
#include "ConstraintSolver.h"
#include "ThreadManager.h"
#include "Octree.h"
#include <set>
#include <functional>
#include <unordered_map>
#include <mutex>

namespace phyz {

	class PhysicsEngine;

	struct CollisionTarget {
	public:
		static CollisionTarget all();
		static CollisionTarget with(RigidBody* r);

		friend class PhysicsEngine;
	private:
		CollisionTarget(bool with_all, RigidBody* specific_target) : with_all(with_all), specific_target(specific_target) {}

		bool with_all;
		RigidBody* specific_target;
	};

	typedef int ColActionID;
	typedef std::function<void(RigidBody* b1, RigidBody* b2, const std::vector<Manifold>& manifold)> ColAction;

	struct ConstraintID {
		enum Type { BALL, HINGE, SLIDER, SPRING };
		inline Type getType() { return type; }

		friend class PhysicsEngine;
	private:
		ConstraintID(Type type, int id) : type(type), uniqueID(id) {}

		Type type;
		int uniqueID;
	};

	class PhysicsEngine {
	public:
		~PhysicsEngine();

		static void enableMultithreading(int n_threads);
		static void disableMultithreading();
		static void setPrintPerformanceData(bool print_data);

		void timeStep();
		RigidBody* createRigidBody(const Geometry& geometry, bool fixed=false, mthz::Vec3 position=mthz::Vec3(), mthz::Quaternion orientation=mthz::Quaternion());
		void removeRigidBody(RigidBody* r);
		void applyVelocityChange(RigidBody* b, const mthz::Vec3& delta_vel, const mthz::Vec3& delta_ang_vel, const mthz::Vec3& delta_psuedo_vel=mthz::Vec3(), const mthz::Vec3&delta_psuedo_ang_vel=mthz::Vec3());
		void disallowCollisionSet(const std::initializer_list<RigidBody*>& bodies);
		void disallowCollision(RigidBody* b1, RigidBody* b2);
		bool collisionAllowed(RigidBody* b1, RigidBody* b2);
		void reallowCollision(RigidBody* b1, RigidBody* b2);
		
		inline int getNumBodies() { return bodies.size(); }
		inline mthz::Vec3 getGravity() { return gravity; }
		inline double getStep_time() { return step_time; }

		void setPGSIterations(int n_vel, int n_pos) { pgsVelIterations = n_vel; pgsPosIterations = n_pos; }
		void setSleepingEnabled(bool sleeping);
		void setStep_time(double d);
		void setGravity(const mthz::Vec3& v);
		void setOctreeParams(double size, double minsize, mthz::Vec3 center = mthz::Vec3(0, 0, 0));
		
		ColActionID registerCollisionAction(CollisionTarget b1, CollisionTarget b2, const ColAction& action);
		void removeCollisionAction(ColActionID action_key);

		ConstraintID addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double pos_correct_strength=350);
		ConstraintID addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double pos_correct_strength=350, double rot_correct_strength=350);
		ConstraintID addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local, double pos_correct_strength = 350,
			double rot_correct_strength=350, double positive_slide_limit=std::numeric_limits<double>::infinity(), double negative_slide_limit=std::numeric_limits<double>::infinity());


		ConstraintID addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, double pos_correct_strength = 350);
		ConstraintID addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 attach_pos_local, mthz::Vec3 rot_axis_local, double pos_correct_strength = 350, double rot_correct_strength = 350);
		ConstraintID addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 slider_pos_local, mthz::Vec3 slider_axis_local, double pos_correct_strength = 350,
			double rot_correct_strength = 350, double positive_slide_limit = std::numeric_limits<double>::infinity(), double negative_slide_limit = -std::numeric_limits<double>::infinity());
		ConstraintID addSpring(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double damping, double stiffness, double resting_length = -1);



		void removeConstraint(ConstraintID id, bool reenable_collision=true);

		void setMotorProperties(ConstraintID id, double max_torque, double min_angle = -std::numeric_limits<double>::infinity(), double max_angle = std::numeric_limits<double>::infinity());
		void setMotorTargetVelocity(ConstraintID id, double target_velocity);
		void setPistonForce(ConstraintID id, double max_force);
		void setPistonTargetVelocity(ConstraintID id, double target_velocity);
		double getMotorAngularPosition(ConstraintID id);

	private:

		static int n_threads;
		static ThreadManager thread_manager;
		static bool use_multithread;
		static bool print_performance_data;

		int pgsVelIterations = 20;
		int pgsPosIterations = 15;

		mthz::Vec3 octree_center = mthz::Vec3(0, 0, 0);
		double octree_size = 2000;
		double octree_minsize = 1;

		struct ConstraintGraphNode; 
		void addContact(RigidBody* b1, RigidBody* b2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness);
		void maintainConstraintGraphApplyPoweredConstraints();
		void bfsVisitAll(ConstraintGraphNode* curr, std::set<ConstraintGraphNode*>* visited, void* in, std::function<void(ConstraintGraphNode* curr, void* in)> action);
		std::vector<std::vector<Constraint*>> sleepOrSolveIslands();
		inline double getCutoffVel(double step_time, const mthz::Vec3& gravity) { return 2 * gravity.mag() * step_time; }
		bool bodySleepy(const std::vector<RigidBody::MovementState>& body_history);
		void deleteRigidBody(RigidBody* r);
		bool readyToSleep(RigidBody* b);
		void wakeupIsland(ConstraintGraphNode* foothold);

		inline double posCorrectCoeff(double pos_correct_strength, double step_time) { return std::min<double>(pos_correct_strength * step_time, 1.0 / step_time); }

		std::vector<RigidBody*> bodies;
		std::vector<RigidBody*> bodies_to_delete;
		mthz::Vec3 gravity;
		double step_time = 1.0 / 90;
		double cutoff_vel = 0;
		int contact_life = 6;
		bool sleeping_enabled = true;
		double sleep_delay = 0.5;

		double vel_sleep_coeff = 0.1;
		double accel_sleep_coeff = 0.022;

		//Constraint Graph
		std::unordered_map<RigidBody*, ConstraintGraphNode*> constraint_graph_nodes;
		std::mutex constraint_graph_lock;

		struct Contact {
			ContactConstraint contact;
			FrictionConstraint friction;
			MagicID magic;
			int memory_life; //how many frames can be referenced for warm starting before is deleted
			bool is_live_contact;
		};

		struct BallSocket {
			BallSocketConstraint constraint;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			double pos_correct_hardness;
			int uniqueID;
		};

		struct Hinge {
			HingeConstraint constraint;
			MotorConstraint motor_constraint;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			mthz::Vec3 b1_rot_axis_body_space;
			mthz::Vec3 b2_rot_axis_body_space;
			double pos_correct_hardness;
			double rot_correct_hardness;
			double max_torque;
			double target_velocity;
			int uniqueID;

			mthz::Vec3 b1_u_axis_reference;
			mthz::Vec3 b1_w_axis_reference;
			mthz::Vec3 b2_rot_comparison_axis;
			double motor_angular_position;
			double min_motor_position;
			double max_motor_position;
		};

		struct Slider {
			SliderConstraint constraint;
			PistonConstraint piston_force;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			mthz::Vec3 b1_slide_axis_body_space;
			mthz::Vec3 b2_slide_axis_body_space;
			double max_piston_force;
			double target_velocity;
			double pos_correct_hardness;
			double rot_correct_hardness;
			double positive_slide_limit;
			double negative_slide_limit;
			bool slide_limit_exceeded;
			int uniqueID;
		};

		struct Spring {
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			double stiffness;
			double damping;
			double resting_length;
			int uniqueID;
		};

	
		double calculateMotorPosition(double current_position, mthz::Vec3 rot_axis, mthz::Vec3 u_ref_axis, mthz::Vec3 w_ref_axis, mthz::Vec3 compare_axis, mthz::Vec3 b1_ang_vel, mthz::Vec3 b2_ang_vel, double timestep);

		struct SharedConstraintsEdge {
			SharedConstraintsEdge(ConstraintGraphNode* n1, ConstraintGraphNode* n2) : n1(n1), n2(n2), contactConstraints(std::vector<Contact*>()) {}
			~SharedConstraintsEdge();

			ConstraintGraphNode* n1;
			ConstraintGraphNode* n2;
			std::vector<Contact*> contactConstraints;
			std::vector<BallSocket*> ballSocketConstraints;
			std::vector<Hinge*> hingeConstraints;
			std::vector<Slider*> sliderConstraints;
			std::vector<Spring*> springs;

			int visited_tag = 0;

			inline ConstraintGraphNode* other(ConstraintGraphNode* c) { return (c->b == n1->b ? n2 : n1); }
			bool noConstraintsLeft() { 
				return contactConstraints.empty() 
					&& ballSocketConstraints.empty()
					&& hingeConstraints.empty()
					&& sliderConstraints.empty()
					&& springs.empty();
			}
		};

		struct ConstraintGraphNode {
			ConstraintGraphNode(RigidBody* b) : b(b), constraints(std::vector<SharedConstraintsEdge*>()) {}
			~ConstraintGraphNode();

			RigidBody* b;
			std::vector<SharedConstraintsEdge*> constraints;

			SharedConstraintsEdge* getOrCreateEdgeTo(ConstraintGraphNode* n2);
		};

		int nextConstraintID = 0;
		std::unordered_map<int, SharedConstraintsEdge*> constraint_map;

		ColActionID nextActionID = 0;
		inline static RigidBody* all() { return nullptr; }
		std::unordered_map<RigidBody*, std::unordered_map<RigidBody*, std::vector<ColActionID>>> get_action_map;
		std::unordered_map<ColActionID, ColAction> col_actions;
	};

}