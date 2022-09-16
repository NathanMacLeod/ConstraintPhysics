#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "Geometry.h"
#include "ConvexPoly.h"
#include "AABB.h"
#include <Vector>
#include <set>

namespace phyz {

	class RigidBody {
	private:
		RigidBody(const Geometry& source_geometry, const mthz::Vec3& pos, const mthz::Quaternion& orientation);
	public:
		typedef int PKey;

		PKey trackPoint(mthz::Vec3 p); //track the movement of p which is on the body b. P given in local coordinates
		mthz::Vec3 getTrackedP(PKey pk);
		mthz::Vec3 getVelOfPoint(mthz::Vec3 p) const;

		double getMass();
		double getInvMass();
		mthz::Mat3 getInvTensor();
		bool getAsleep() { return asleep;  }
		bool getIsFixed() { return fixed; }
		bool getNoCollision() { return no_collision; }
		mthz::Vec3 getPos() { return getTrackedP(origin_pkey); }
		mthz::Vec3 getCOM() { return com; }
		mthz::Quaternion getOrientation() { return orientation; }
		mthz::Vec3 getVel();
		mthz::Vec3 getAngVel();

		void applyForce(mthz::Vec3 force) { vel += force * getInvMass(); }
		void applyTorque(mthz::Vec3 torque) { ang_vel += getInvTensor() * torque; }
		void applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position);
		void setToPosition(const mthz::Vec3& pos);
		void setCOMtoPosition(const mthz::Vec3& pos);
		void translate(const mthz::Vec3& v);
		void setOrientation(const mthz::Quaternion orientation);
		void setVel(mthz::Vec3 vel);
		void setAngVel(mthz::Vec3 ang_vel);
		void setFixed(bool fixed);
		void setNoCollision(bool no_collision);

		friend class PhysicsEngine;
	private:
		AABB aabb;
		double radius;
		bool fixed;
		mthz::Vec3 local_coord_origin;
		PKey origin_pkey;

		mthz::Quaternion orientation;
		mthz::Vec3 com;
		mthz::Vec3 vel;
		mthz::Vec3 ang_vel;
		mthz::Vec3 psuedo_vel;
		mthz::Vec3 psuedo_ang_vel;
		bool recievedWakingAction;

		void sleep();
		void wake();
		void alertWakingAction();

		void applyGyroAccel(float fElapsedTime);
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
		
		std::vector<ConvexPoly> geometry;
		std::vector<AABB> geometry_AABB;
		std::vector<ConvexPoly> reference_geometry;
		std::vector<GaussMap> gauss_maps;
		std::vector<GaussMap> reference_gauss_maps;
		
		std::vector<mthz::Vec3> track_p;
	};

}