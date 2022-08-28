#pragma once
#include "RigidBody.h"
#include "CollisionDetect.h"
#include "ConstraintSolver.h"
#include "ThreadManager.h"
#include <set>
#include <functional>
#include <unordered_map>
#include <mutex>

namespace phyz {

	class PhysicsEngine {
	public:
		PhysicsEngine() {
#ifdef USE_MULTITHREAD
			thread_manager.init(N_THREADS);
#endif
		}

		void timeStep();
		RigidBody* createRigidBody(const Geometry& geometry, bool fixed=false, mthz::Vec3 position=mthz::Vec3(), mthz::Quaternion orientation=mthz::Quaternion());
		void applyVelocityChange(RigidBody* b, const mthz::Vec3& delta_vel, const mthz::Vec3& delta_ang_vel, const mthz::Vec3& delta_psuedo_vel=mthz::Vec3(), const mthz::Vec3&delta_psuedo_ang_vel=mthz::Vec3());
		void disallowCollision(RigidBody* b1, RigidBody* b2);
		bool collisionAllowed(RigidBody* b1, RigidBody* b2);
		void reallowCollision(RigidBody* b1, RigidBody* b2);
		
		inline int getNumBodies() { return bodies.size(); }
		inline mthz::Vec3 getGravity() { return gravity; }
		inline double getStep_time() { return step_time; }

		void setStep_time(double d);
		void setGravity(const mthz::Vec3& v);

		typedef int MotorID;

		void addBallSocketConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, double pos_correct_strength=250);
		MotorID addHingeConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_attach_pos_local, mthz::Vec3 b2_attach_pos_local, mthz::Vec3 b1_rot_axis_local, mthz::Vec3 b2_rot_axis_local, double pos_correct_strength=250, double rot_correct_strength=250);
		void addSliderConstraint(RigidBody* b1, RigidBody* b2, mthz::Vec3 b1_slider_pos_local, mthz::Vec3 b2_slider_pos_local, mthz::Vec3 b1_slider_axis_local, mthz::Vec3 b2_slider_axis_local, double pos_correct_strength=250, double rot_correct_strength=250);

		void setMotorPower(MotorID id, double power);

	private:

		static const int N_THREADS = 4;
		struct ConstraintGraphNode; 
		
		void addContact(RigidBody* b1, RigidBody* b2, mthz::Vec3 p, mthz::Vec3 norm, const MagicID& magic, double bounce, double static_friction, double kinetic_friction, int n_points, double pen_depth, double hardness);
		void maintainConstraintGraphApplyPoweredConstraints();
		void dfsVisitAll(ConstraintGraphNode* curr, std::set<ConstraintGraphNode*>* visited, void* in, std::function<void(ConstraintGraphNode* curr, void* in)> action);
		std::vector<std::vector<Constraint*>> sleepOrSolveIslands();
		inline double getCutoffVel(double step_time, const mthz::Vec3& gravity) { return 2 * gravity.mag() * step_time; }
		bool bodySleepy(const std::vector<RigidBody::MovementState>& body_history);
		bool readyToSleep(RigidBody* b);
		void wakeupIsland(ConstraintGraphNode* foothold);

		inline double posCorrectCoeff(double pos_correct_strength, double step_time) { return std::min<double>(pos_correct_strength * step_time, 1.0 / step_time); }

		std::vector<RigidBody*> bodies;
		mthz::Vec3 gravity;
		double step_time = 1.0 / 90;
		double cutoff_vel = 0;
		int contact_life = 6;
		bool sleeping_enabled = false;
		double sleep_delay = 0.5;

		double vel_sleep_coeff = 0.1;
		double accel_sleep_coeff = 0.022;

		ThreadManager thread_manager;

		//Constraint Graph
		std::unordered_map<RigidBody*, ConstraintGraphNode*> constraint_graph_nodes;
		std::mutex constraint_graph_lock;

		struct Contact {
			ContactConstraint contact;
			FrictionConstraint friction1;
			FrictionConstraint friction2;
			MagicID magic;
			int memory_life; //how many frames can be referenced for warm starting before is deleted
			bool is_live_contact;
		};

		struct BallSocket {
			BallSocketConstraint constraint;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			double pos_correct_hardness;
		};

		struct Hinge {
			HingeConstraint constraint;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			mthz::Vec3 b1_rot_axis_body_space;
			mthz::Vec3 b2_rot_axis_body_space;
			double pos_correct_hardness;
			double rot_correct_hardness;
			double power_level;
		};

		struct Slider {
			SliderConstraint constraint;
			RigidBody::PKey b1_point_key;
			RigidBody::PKey b2_point_key;
			mthz::Vec3 b1_slide_axis_body_space;
			mthz::Vec3 b2_slide_axis_body_space;
			double pos_correct_hardness;
			double rot_correct_hardness;
		};

		struct SharedConstraintsEdge {
			SharedConstraintsEdge(ConstraintGraphNode* n1, ConstraintGraphNode* n2) : n1(n1), n2(n2), contactConstraints(std::vector<Contact*>()) {}

			ConstraintGraphNode* n1;
			ConstraintGraphNode* n2;
			std::vector<Contact*> contactConstraints;
			std::vector<BallSocket*> ballSocketConstraints;
			std::vector<Hinge*> hingeConstraints;
			std::vector<Slider*> sliderConstraints;

			inline ConstraintGraphNode* other(ConstraintGraphNode* c) { return (c->b == n1->b ? n2 : n1); }
			bool noConstraintsLeft() { 
				return contactConstraints.empty() 
					&& ballSocketConstraints.empty()
					&& hingeConstraints.empty()
					&& sliderConstraints.empty();
			}
		};

		struct ConstraintGraphNode {
			ConstraintGraphNode(RigidBody* b) : b(b), constraints(std::vector<SharedConstraintsEdge*>()) {}

			RigidBody* b;
			std::vector<SharedConstraintsEdge*> constraints;

			SharedConstraintsEdge* getOrCreateEdgeTo(ConstraintGraphNode* n2);
		};

		MotorID nextMotorID = 0;
		std::unordered_map<MotorID, Hinge*> motor_map;
	};

}