#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "ConvexPoly.h"
#include <Vector>

namespace phyz {

	

	class RigidBody {
	public:
		typedef int PKey;

		RigidBody(const std::vector<ConvexPoly>& geometry, double density, int id);
		PKey track_point(mthz::Vec3 p); //track the movement of p which is on the body b. P given in world coordinates
		mthz::Vec3 getTrackedP(PKey pk);
		mthz::Vec3 getVelOfPoint(mthz::Vec3 p) const;
		void applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position);
		void applyGyroAccel(float fElapsedTime, int n_itr = 1);
		void updateGeometry();

		int id;
		mthz::Quaternion orientation;
		mthz::Vec3 com;
		mthz::Vec3 vel;
		mthz::Vec3 ang_vel;

		mthz::Mat3 invTensor;
		mthz::Mat3 tensor;
		double mass;

		double radius;
		bool fixed;

		friend class PhysicsEngine;
	private:
		std::vector<ConvexPoly> geometry;
		std::vector<ConvexPoly> reference_geometry;
		
		std::vector<mthz::Vec3> track_p;
	};

}