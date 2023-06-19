#pragma once
#include "../../Math/src/Vec3.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <cinttypes>
#include <vector>

namespace phyz {
	class ConvexPrimitive;
	class Polyhedron;
	class Sphere;
	struct Material;
	struct GaussMap;

	static const double M_PI = 3.14159265358979323846;
	static const double TOL_ANG = 1 * M_PI / 180;
	static const double COS_TOL = 1 - cos(TOL_ANG);
	static const double SIN_TOL = sin(TOL_ANG);
	const double CUTOFF_MAG = 0.00000001;

	//magicID uniquely identifies a contact point as the combination of the two features that form it, e.g. a point and a plane, or two edges
	struct MagicID {
		uint64_t cID; // 0 - 31 c1 id, 32 - 63 c2 id
		uint64_t bID; // 0 - 31 c1 featureID, 32 - 63 c2 featureID
	};

	bool operator==(const MagicID& m1, const MagicID& m2);

	struct ContactP {
		mthz::Vec3 pos;
		double pen_depth;
		double restitution;
		double kinetic_friction_coeff;
		double static_friction_coeff;
		MagicID magicID;
	};

	struct Manifold {
		std::vector<ContactP> points;
		mthz::Vec3 normal;
		double max_pen_depth;
	};
	Manifold merge_manifold(const Manifold& m1, const Manifold& m2);
	Manifold cull_manifold(const Manifold& m, int new_size);

	
	struct ExtremaInfo {
		ExtremaInfo(int min_pID, int max_pID, double min_val, double max_val)
			: min_pID(min_pID), max_pID(max_pID), min_val(min_val), max_val(max_val)
		{}
		ExtremaInfo() 
			:min_val(std::numeric_limits<double>::infinity()), max_val(-std::numeric_limits<double>::infinity()), min_pID(-1), max_pID(-1)
		{}

		int min_pID;
		int max_pID;
		double min_val;
		double max_val;
	};

	inline ExtremaInfo recenter(const ExtremaInfo& info, double old_ref_value, double new_ref_value);
	inline ExtremaInfo findExtrema(const Polyhedron& c, mthz::Vec3 axis);
	Manifold detectCollision(const ConvexPrimitive& a, const ConvexPrimitive& b);

}

//providing hash function for MagicID
namespace std {
  template <> struct hash<phyz::MagicID> {
		inline size_t operator()(const phyz::MagicID& key) const {
			return key.cID * key.bID;
		}
	};
}