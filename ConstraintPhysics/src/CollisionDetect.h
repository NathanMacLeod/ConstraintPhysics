#pragma once
#include "RigidBody.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <cinttypes>

namespace phyz {
	static const double M_PI = 3.14159265358979323846;
	static const double TOL_ANG = 2 * M_PI / 180;
	static const double COS_TOL = 1 - cos(TOL_ANG);
	static const double SIN_TOL = sin(TOL_ANG);
	const double CUTOFF_MAG = 0.00000001;

	struct Manifold {
		std::vector<mthz::Vec3> points;
		mthz::Vec3 normal;
		double pen_depth;
		double flatness; //cos angle between two surfaces
		uint64_t magicID;

		void generateMagicID(const RigidBody& a, const RigidBody& b) {
			int largerID = std::max<int>(a.id, b.id);
			int smallerID = std::min<int>(a.id, b.id);

			magicID = 0;
			magicID |= smallerID;
			magicID |= uint64_t(largerID) << 32;
		}
	};
	Manifold merge_manifold(const Manifold& m1, const Manifold& m2);
	Manifold cull_manifold(const Manifold& m, int new_size);

	//geom_indx only needed for generating magicIDs, can be ignored otherwise
	Manifold SAT(const ConvexPoly& a, const GaussMap& ag, const ConvexPoly& b, const GaussMap& bg);

}