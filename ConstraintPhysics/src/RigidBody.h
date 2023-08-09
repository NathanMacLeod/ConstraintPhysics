#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "Geometry.h"
#include "ConvexPrimitive.h"
#include "AABB.h"
#include <Vector>
#include <set>

namespace phyz {

	class RigidBody {
	private:
		RigidBody(const ConvexUnionGeometry& source_geometry, const mthz::Vec3& pos, const mthz::Quaternion& orientation, unsigned int id);
		RigidBody(const StaticMeshGeometry& source_geometry, unsigned int id);
	public:
		typedef int PKey;
		enum GeometryType { CONVEX_UNION, STATIC_MESH };
		enum MovementType { DYNAMIC, FIXED, KINEMATIC };

		PKey trackPoint(mthz::Vec3 p); //track the movement of p which is on the body b. P given in local coordinates
		mthz::Vec3 getTrackedP(PKey pk);
		mthz::Vec3 getVelOfPoint(mthz::Vec3 p) const;
		inline unsigned int getID() const { return id; }

		double getMass();
		double getInvMass();
		mthz::Mat3 getInvTensor();
		bool getAsleep();
		inline MovementType getMovementType() { return movement_type; }
		inline bool getNoCollision() { return no_collision; }
		mthz::Vec3 getPos() { return getTrackedP(origin_pkey); }
		inline mthz::Vec3 getCOM() { return com; }
		inline mthz::Quaternion getOrientation() { return orientation; }
		mthz::Vec3 getVel();
		mthz::Vec3 getAngVel();
		inline GeometryType getGeometryType() { return geometry_type; }

		void applyForce(mthz::Vec3 force) { vel += force * getInvMass(); }
		void applyTorque(mthz::Vec3 torque) { ang_vel += getInvTensor() * torque; }
		void applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position);
		void setToPosition(const mthz::Vec3& pos);
		void setCOMtoPosition(const mthz::Vec3& pos);
		void translate(const mthz::Vec3& v);
		void setOrientation(const mthz::Quaternion orientation);
		void setVel(mthz::Vec3 vel);
		void setAngVel(mthz::Vec3 ang_vel);
		void setMovementType(MovementType type);
		void setNoCollision(bool no_collision);
		void setSleepDisabled(bool b) { sleep_disabled = b; }

		friend class PhysicsEngine;
	private:
		AABB aabb;
		MovementType movement_type;
		mthz::Vec3 local_coord_origin;
		PKey origin_pkey;
		unsigned int id;

		mthz::Quaternion orientation;
		mthz::Vec3 com;
		mthz::Vec3 vel;
		mthz::Vec3 ang_vel;
		mthz::Vec3 psuedo_vel;
		mthz::Vec3 psuedo_ang_vel;
		bool recievedWakingAction;
		bool sleep_disabled;

		void sleep();
		void wake();
		void alertWakingAction();

		void rotateWhileApplyingGyroAccel(float fElapsedTime, int n_itr = 1, bool gyro_accel_disabled=false);
		void updateGeometry();

		struct MovementState {
			mthz::Vec3 vel;
			mthz::Vec3 ang_vel;
		};

		std::vector<MovementState> history;
		void recordMovementState(int history_length);

		std::set<RigidBody*> no_collision_set;

		mthz::Mat3 invTensor;
		mthz::Mat3 tensor;
		mthz::Mat3 reference_invTensor;
		mthz::Mat3 reference_tensor;
		double mass;
		bool asleep;
		bool no_collision;
		double sleep_ready_counter;
		int non_sleepy_tick_count;
		
		GeometryType geometry_type;

		//used for convex union
		std::vector<AABB> geometry_AABB;
		std::vector<ConvexPrimitive> geometry;
		std::vector<ConvexPrimitive> reference_geometry;

		//for static mesh
		StaticMeshGeometry reference_mesh;
		StaticMeshGeometry mesh;
		
		std::vector<mthz::Vec3> track_p;
	};

	class OrderedBodyPair {
	public:
		OrderedBodyPair(RigidBody* t1, RigidBody* t2) {
			//keep ordering consistent to avoid {x, y}; {y, x} duplicates
			if (t1->getID() < t2->getID()) {
				this->t1 = t1;
				this->t2 = t2;
			}
			else {
				this->t1 = t2;
				this->t2 = t1;
			}
		}

		RigidBody* t1 = nullptr;
		RigidBody* t2 = nullptr;

		//for use in std::set
		bool operator<(const OrderedBodyPair& p) const {
			if (t1->getID() < p.t1->getID()) {
				return true;
			}
			else if (t1->getID() > p.t1->getID()) {
				return false;
			}
			else {
				return t2->getID() < p.t2->getID();
			}
		}

		bool operator==(const OrderedBodyPair& p) const {
			return t1 == p.t1 && t2 == p.t2;
		}
	};
}