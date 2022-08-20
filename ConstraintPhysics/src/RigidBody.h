#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "ConvexPoly.h"
#include "AABB.h"
#include <Vector>
#include <set>

namespace phyz {

	class RigidBody;
	class Geometry {
	public:
		void pushConvexPoly(const ConvexPoly& p) { polyhedra.push_back(p); };
		void setOrigin(const mthz::Vec3 v) { origin_position = v; }
		void setOrientation(const mthz::Quaternion q) { orientation = q; }
		inline const std::vector<ConvexPoly>& getPolyhedra() const { return polyhedra; }

		static Geometry merge(const Geometry& g1, const Geometry& g2);

		friend class RigidBody;
	private:
		std::vector<ConvexPoly> polyhedra;
		mthz::Vec3 origin_position;
		mthz::Quaternion orientation;
	};

	class RigidBody {
	private:
		RigidBody(const Geometry& source_geometry, const mthz::Vec3& pos, const mthz::Quaternion& orientation);
	public:
		typedef int PKey;

		PKey trackPoint(mthz::Vec3 p); //track the movement of p which is on the body b. P given in world coordinates
		mthz::Vec3 getTrackedP(PKey pk);
		mthz::Vec3 getVelOfPoint(mthz::Vec3 p) const;

		double getMass();
		double getInvMass();
		mthz::Mat3 getInvTensor();
		bool getAsleep() { return asleep;  }
		mthz::Vec3 getPos() { return getTrackedP(pos_pkey); }
		mthz::Vec3 getCOM() { return com; }
		mthz::Quaternion getOrientation() { return orientation; }
		mthz::Vec3 getVel() { return vel; }
		mthz::Vec3 getAngVel() { return ang_vel; }

		void setToPosition(const mthz::Vec3 pos);
		void setCOMtoPosition(const mthz::Vec3 pos);
		void setOrientation(const mthz::Quaternion orientation);
		void setVel(mthz::Vec3 vel) { this->vel = vel; }
		void setAngVel(mthz::Vec3 ang_vel) { this->ang_vel = ang_vel; }

		void sleep();
		void wake();

		friend class PhysicsEngine;
	private:
		AABB aabb;
		double radius;
		bool fixed;
		PKey pos_pkey;

		mthz::Quaternion orientation;
		mthz::Vec3 com;
		mthz::Vec3 vel;
		mthz::Vec3 ang_vel;
		mthz::Vec3 psuedo_vel;
		mthz::Vec3 psuedo_ang_vel;

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
		double mass;
		bool asleep;
		double sleep_ready_counter;
		
		std::vector<ConvexPoly> geometry;
		std::vector<AABB> geometry_AABB;
		std::vector<ConvexPoly> reference_geometry;
		std::vector<GaussMap> gauss_maps;
		std::vector<GaussMap> reference_gauss_maps;
		
		std::vector<mthz::Vec3> track_p;
	};

}