#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "ConvexPoly.h"
#include <Vector>

namespace phyz {

	class RigidBody {
	public:
		RigidBody(const std::vector<ConvexPoly>& geometry, double density, int id);
		void applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position, mthz::Mat3* invTensor = nullptr);
		void updateGeometry();

		int id;
		mthz::Quaternion orientation;
		mthz::Vec3 com;
		mthz::Vec3 vel;
		mthz::Vec3 ang_vel;

		mthz::Mat3 tensor;
		double mass;
		double radius;
		bool fixed;

		friend class PhysicsEngine;
	private:
		std::vector<ConvexPoly> geometry;
		std::vector<ConvexPoly> reference_geometry;
	};

}