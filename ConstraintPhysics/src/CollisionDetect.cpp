#include "CollisionDetect.h"
#include "ConvexPrimitive.h"
#include "Geometry.h"

#include <cassert>

// TODO: there is a lot of code in this file that could be written much better, both for maintainability and performance. going to hold off on it for now though as I plan on doing SIMD at some point, which will require a lot of rewriting anyway

static uint32_t getEdgeID(uint16_t p1_id, uint16_t p2_id) {
	int min, max;
	if (p1_id < p2_id) {
		min = p1_id;
		max = p2_id;
	}
	else {
		min = p2_id;
		max = p1_id;
	}

	return (0x0000FFFF & min) + (0xFFFF0000 & ((uint32_t)max << 16));
}

namespace phyz {

	bool operator==(const MagicID& m1, const MagicID& m2) {
		return m1.bID == m2.bID && m1.cID == m2.cID;
	}

	MagicID swapOrder(const MagicID m) {
		uint64_t c1_cID = (0x00000000FFFFFFFF & m.cID);
		uint64_t c2_cID = (0xFFFFFFFF00000000 & m.cID) >> 32;
		uint64_t c1_bID = (0x00000000FFFFFFFF & m.bID);
		uint64_t c2_bID = (0xFFFFFFFF00000000 & m.bID) >> 32;

		MagicID out;
		out.cID = (0x00000000FFFFFFFF & c2_cID) + (0xFFFFFFFF00000000 & (c1_cID << 32));
		out.bID = (0x00000000FFFFFFFF & c2_bID) + (0xFFFFFFFF00000000 & (c1_bID << 32));

		return out;
	}

	static Manifold SAT_PolyPoly(const Polyhedron& a, int a_id, const Material& a_mat, const Polyhedron& b, int b_id, const Material& b_mat, int surfaceid1, int surfaceid2);
	static Manifold detectSphereSphere(const Sphere& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat);
	static Manifold detectCylinderCylinder(const Cylinder& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat);
	static Manifold detectCapsuleCapsule(const Capsule& a, int a_id, const Material& a_mat, const Capsule& b, int b_id, const Material& b_mat);
	static Manifold SAT_PolySphere(const Polyhedron& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat);
	static Manifold SAT_PolyCapsule(const Polyhedron& a, int a_id, const Material& a_mat, const Capsule& b, int b_id, const Material& b_mat);
	static Manifold SAT_PolyCylinder(const Polyhedron& a, int a_id, const Material& a_mat, const Cylinder&b, int b_id, const Material& b_mat);
	static Manifold detectSphereCapsule(const Sphere& a, int a_id, const Material& a_mat, const Capsule& b, int b_id, const Material& b_mat);
	static Manifold detectSphereCylinder(const Sphere& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat);
	static Manifold detectCapsuleCylinder(const Capsule &a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat);
	static std::vector<Manifold> SAT_PolyMesh(const Polyhedron& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);
	static std::vector<Manifold> SAT_SphereMesh(const Sphere& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);
	static std::vector<Manifold> SAT_CapsuleMesh(const Capsule& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);
	static std::vector<Manifold> SAT_CylinderMesh(const Cylinder& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);

	Manifold detectCollision(const ConvexPrimitive& a, const ConvexPrimitive& b, int surfaceid1, int surfaceid2) {
		switch (a.getType()) {
		case POLYHEDRON:
			switch (b.getType()) {
			case POLYHEDRON:
				return SAT_PolyPoly((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Polyhedron&)*b.getGeometry(), b.getID(), b.material, surfaceid1, surfaceid2);
			case SPHERE:
				return SAT_PolySphere((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Sphere&)*b.getGeometry(), b.getID(), b.material);
			case CAPSULE:
				return SAT_PolyCapsule((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Capsule&)*b.getGeometry(), b.getID(), b.material);
			case CYLINDER:
				return SAT_PolyCylinder((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Cylinder&)*b.getGeometry(), b.getID(), b.material);
			}
			break;
		case SPHERE:
			switch (b.getType()) {
			case POLYHEDRON:
			{
				Manifold out = SAT_PolySphere((const Polyhedron&)*b.getGeometry(), b.getID(), b.material, (const Sphere&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. SAT_PolySphere generates normal facing away from the polyhedron
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
			case SPHERE:
				return detectSphereSphere((const Sphere&)*a.getGeometry(), a.getID(), a.material, (const Sphere&)*b.getGeometry(), b.getID(), b.material);
			case CAPSULE:
				return detectSphereCapsule((const Sphere&)*a.getGeometry(), a.getID(), a.material, (const Capsule&)*b.getGeometry(), b.getID(), b.material);
			case CYLINDER:
				return detectSphereCylinder((const Sphere&)*a.getGeometry(), a.getID(), a.material, (const Cylinder&)*b.getGeometry(), b.getID(), b.material);
			}
			break;
		case CAPSULE:
			switch (b.getType()) {
			case POLYHEDRON:
			{
				Manifold out = SAT_PolyCapsule((const Polyhedron&)*b.getGeometry(), b.getID(), b.material, (const Capsule&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. SAT_PolySphere generates normal facing away from the polyhedron
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
			case SPHERE:
			{
				Manifold out = detectSphereCapsule((const Sphere&)*b.getGeometry(), b.getID(), b.material, (const Capsule&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. SAT_PolySphere generates normal facing away from the polyhedron
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
			case CAPSULE:
				return detectCapsuleCapsule((const Capsule&)*a.getGeometry(), a.getID(), a.material, (const Capsule&)*b.getGeometry(), b.getID(), b.material);
			case CYLINDER:
				return detectCapsuleCylinder((const Capsule&)*a.getGeometry(), a.getID(), a.material, (const Cylinder&)*b.getGeometry(), b.getID(), b.material);
			}
			break;
		case CYLINDER:
			switch (b.getType()) {
			case POLYHEDRON:
			{
				Manifold out = SAT_PolyCylinder((const Polyhedron&)*b.getGeometry(), b.getID(), b.material, (const Cylinder&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. SAT_PolySphere generates normal facing away from the polyhedron
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
			case SPHERE:
			{
				Manifold out = detectSphereCylinder((const Sphere&)*b.getGeometry(), b.getID(), b.material, (const Cylinder&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. detectSphereCylinder generates normal facing away from the sphere
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
			case CAPSULE:
			{
				Manifold out = detectCapsuleCylinder((const Capsule&)*b.getGeometry(), b.getID(), b.material, (const Cylinder&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. detectSphereCylinder generates normal facing away from the sphere
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
			case CYLINDER:
				return detectCylinderCylinder((const Cylinder&)*a.getGeometry(), a.getID(), a.material, (const Cylinder&)*b.getGeometry(), b.getID(), b.material);
			}
		}
		
		assert(false);
		return {};
	}

	std::vector<Manifold> detectCollision(const ConvexPrimitive& a, AABB a_aabb, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		switch (a.getType())
		{
		case POLYHEDRON:
			return SAT_PolyMesh((const Polyhedron&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		case SPHERE:
			return SAT_SphereMesh((const Sphere&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		case CAPSULE:
			return SAT_CapsuleMesh((const Capsule&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		case CYLINDER:
			return SAT_CylinderMesh((const Cylinder&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		}
		
		assert(false);
		return {};
	}

	std::vector<Manifold> detectCollision(const StaticMeshGeometry& a, mthz::Vec3 a_world_position, mthz::Quaternion a_world_orientation, const ConvexPrimitive& b, AABB b_aabb) {
		std::vector<Manifold> out;
		switch (b.getType()) {
		case POLYHEDRON:
			out = SAT_PolyMesh((const Polyhedron&)*b.getGeometry(), b_aabb, b.getID(), b.material, a, a_world_position, a_world_orientation);
			break;
		case SPHERE:
			out = SAT_SphereMesh((const Sphere&)*b.getGeometry(), b_aabb, b.getID(), b.material, a, a_world_position, a_world_orientation);
			break;
		case CAPSULE:
			out = SAT_CapsuleMesh((const Capsule&)*b.getGeometry(), b_aabb, b.getID(), b.material, a, a_world_position, a_world_orientation);
			break;
		case CYLINDER:
			out = SAT_CylinderMesh((const Cylinder&)*b.getGeometry(), b_aabb, b.getID(), b.material, a, a_world_position, a_world_orientation);
			break;
		}

		for (Manifold& m : out) {
			m.normal = -m.normal; //physics engine expects the manifold to be facing away from a. SAT_PolySphere generates normal facing away from the polyhedron
			for (ContactP& p : m.points) {
				p.magicID = swapOrder(p.magicID);
			}
		}
		return out;
	}

	struct TransformedTriangle {
		StaticMeshVertex vertices[3];
		StaticMeshHalfEdge edges[3];
		mthz::Vec3 normal;
		uint32_t original_triangle_id;
		Material material;
	};

	struct CheckNormResults {
		int a_maxPID;
		int b_maxPID;
		mthz::Vec3 norm;
		double pen_depth;

		inline bool seprAxisExists() { return pen_depth < 0; }
	};

	enum ContactAreaOrigin { FACE, EDGE, VERTEX, CYLINDER_BARREL };

	struct ContactArea {
		std::vector<mthz::NVec<2>> ps;
		std::vector<int> p_IDs;
		int surfaceID;
		ContactAreaOrigin origin;
	};

	static inline ContactArea projectFace(const Surface& s, mthz::Vec3 u, mthz::Vec3 w) {
		int n_points = s.n_points();
		ContactArea out = { std::vector<mthz::NVec<2>>(n_points), std::vector<int>(n_points), s.getSurfaceID(), FACE};
		
		for (int i = 0; i < n_points; i++) {
			mthz::Vec3 v = s.getPointI(i);
			out.ps[i] = mthz::NVec<2>{ v.dot(u), v.dot(w) };
			out.p_IDs[i] = s.point_indexes[i];
		}

		return out;
	}

	static inline ContactArea projectEdge(const Edge& e, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<mthz::NVec<2>>(), std::vector<int>(), -1, EDGE };

		out.ps = { mthz::NVec<2>{ e.p1().dot(u), e.p1().dot(w) }, mthz::NVec<2>{ e.p2().dot(u), e.p2().dot(w) } };
		out.p_IDs = { e.p1_indx, e.p2_indx };

		return out;
	}

	static ContactArea findContactArea(const Polyhedron& c, mthz::Vec3 n, mthz::Vec3 p, int p_ID, mthz::Vec3 u, mthz::Vec3 w, int surfaceid1=-1, int surfaceid2=-1) {
		// at this point SAT has given us the contact normal direction n, and a single point that is part of this objects collision area.
		// we want to figure out what other features are involved in this contact- either a whole face that includes p, an edge that includes p,
		// or maybe just p itself.

		int best_surface_index = -1;
		double best_surface_tolerance = 1.0;
		for (int surface_index : c.getFaceIndicesAdjacentToPointI(p_ID)) {
			const Surface& s = c.getSurfaces()[surface_index];
			double cos_ang = s.normal().dot(n);
			if (1 - cos_ang < best_surface_tolerance) {
				best_surface_tolerance = 1 - cos_ang;
				best_surface_index = surface_index;
			}
		}
		//double tol = (c.getSurfaces()[best_surface_index].getSurfaceID() == surfaceid1 || c.getSurfaces()[best_surface_index].getSurfaceID() == surfaceid2) ? PREVIOUS_FEATURE_BIASED_COS_TOL : COS_TOL;
		//if (tol == COS_TOL) { printf("using normal tol\n"); } else { printf("using cached tol\n"); }
		if (best_surface_tolerance <= COS_TOL) {
			return projectFace(c.getSurfaces()[best_surface_index], u, w);
		}

		int best_edge_index = -1;
		double best_edge_tolerance = 1.0;
		for (int edge_index : c.getEdgeIndicesAdjacentToPointI(p_ID)) {
			const Edge& e = c.getEdges()[edge_index];
			double sin_ang = abs((e.p2() - e.p1()).normalize().dot(n));
			if (sin_ang < best_edge_tolerance) {
				best_edge_tolerance = sin_ang;
				best_edge_index = edge_index;
			}
		}

		if (best_edge_tolerance <= SIN_TOL) {
			return projectEdge(c.getEdges()[best_edge_index], n, p, u, w);
		}

		return ContactArea{
			{ mthz::NVec<2>{ p.dot(u), p.dot(w) } },
			{ p_ID },
			-1,
			VERTEX
		};
		
	}

	static mthz::Vec3 acceptOrSnapNormalAgainstVertexGaussMap(const StaticMeshVertex& s, mthz::Vec3 normal) {
		assert(!s.valid_normal_gauss_map.empty()); // nothing to snap to. Rejected!

		mthz::Vec3 out_vector = normal;
		double closest_feature_dist = std::numeric_limits<double>::infinity(); // if normal is outside, the best feature we could snap to.

		//TODO: profile and determine which of these cross products should be cached, if any

		for (int i = 0; i < s.valid_normal_gauss_map.size(); i++) {
			mthz::Vec3 p1 = s.valid_normal_gauss_map[i], p2 = s.valid_normal_gauss_map[(i + 1) % s.valid_normal_gauss_map.size()];
			mthz::Vec3 inner_region_direction = p1.cross(p2);
			if (normal.dot(inner_region_direction) >= 0) { continue; } // norm appears inside according to this edge.

			// norm is outside. evaluate whether this edges features might be the best to snap to.
			mthz::Vec3 p1_to_p2_dir = inner_region_direction.cross(p1);
			if (p1_to_p2_dir.dot(normal) < 0) {
				// snap to p1.
				double snap_to_p1_dist = normal.cross(p1).mag();
				if (snap_to_p1_dist < closest_feature_dist) {
					closest_feature_dist = snap_to_p1_dist;
					out_vector = p1;
				}
			}
			else if (mthz::Vec3 p2_to_p1_dir = p2.cross(inner_region_direction); p2_to_p1_dir.dot(normal) >= 0) {
				// snap to edge
				// to snap to edge, delete component parallel to the edge in direction, then normalize. can only be done if normal is within the edges arc
				mthz::Vec3 inner_dir_normalized = inner_region_direction.normalize();
				mthz::Vec3 snapped = (normal - inner_dir_normalized * inner_dir_normalized.dot(normal)).normalize();
				double snap_dist = normal.cross(snapped).mag();
				if (snap_dist < closest_feature_dist) {
					closest_feature_dist = snap_dist;
					out_vector = snapped;
				}
			}
			else {
				//todo debug whether this is really needed
				//snap to p2
				double snap_to_p2_dist = normal.cross(p2).mag();
				if (snap_to_p2_dist < closest_feature_dist) {
					closest_feature_dist = snap_to_p2_dist;
					out_vector = p2;
				}
			}
			// else snap to p2. but we will let the next segment handle that in it's "snap to p1" case.
		}

		return out_vector;
	}

	static mthz::Vec3 acceptOrSnapNormalAgainstEdgeGaussArc(const StaticMeshHalfEdge& e, mthz::Vec3 normal) {
		assert(e.has_gauss_arc);

		//the normal should lie on the arc defined by the two points
		mthz::Vec3 arc_normal = e.gauss_arc_g1.cross(e.gauss_arc_g2);
		mthz::Vec3 p1_to_p2_dir = arc_normal.cross(e.gauss_arc_g1);

		if (p1_to_p2_dir.dot(normal) < 0) {
			//snap to p1
			return e.gauss_arc_g1;
		}
		else if (mthz::Vec3 p2_to_p1_dir = e.gauss_arc_g2.cross(arc_normal); p2_to_p1_dir.dot(normal) < 0) {
			//snap to p2
			return e.gauss_arc_g2;
		}
		else {
			//snap to arc edge
			arc_normal = arc_normal.normalize();
			return (normal - arc_normal * arc_normal.dot(normal)).normalize();
		}
	}

	static bool normSatisfiesVertexGaussMap(const StaticMeshVertex& s, mthz::Vec3 normal) {
		if (s.valid_normal_gauss_map.empty()) return false;

		for (int i = 0; i < s.valid_normal_gauss_map.size(); i++) {
			//TODO: just save these vectors rather than doing a cross product everytime
			mthz::Vec3 inner_region_direction = s.valid_normal_gauss_map[i].cross(s.valid_normal_gauss_map[(i + 1) % s.valid_normal_gauss_map.size()]);
			if (normal.dot(inner_region_direction) < 0) return false;
		}
		return true;
	}

	static bool normSatisfiesEdgeGaussArc(const StaticMeshHalfEdge& e, mthz::Vec3 normal) {
		if (!e.has_gauss_arc) return false;
		const double EPS = 0.0001;

		//the normal should lie on the arc defined by the two points
		mthz::Vec3 arc_normal = e.gauss_arc_g1.cross(e.gauss_arc_g2);
		//check vector lies close to the plane
		if (abs(normal.dot(arc_normal)) > EPS) return false;
		mthz::Vec3 v0_up = arc_normal.cross(e.gauss_arc_g1);

		//check vector doesnt lie outside the arc within the plane
		if (normal.dot(v0_up) < -EPS) return false;
		mthz::Vec3 v1_down = e.gauss_arc_g2.cross(arc_normal);
		if (normal.dot(v1_down) < -EPS) return false;

		return true;
	}

	static inline ContactArea projectTriangleFace(const TransformedTriangle& t, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<mthz::NVec<2>>(3), std::vector<int>(3), t.original_triangle_id, FACE };

		for (int i = 0; i < 3; i++) {
			mthz::Vec3 v = t.vertices[i].p;
			out.ps[i] = mthz::NVec<2>{ v.dot(u), v.dot(w) };
			out.p_IDs[i] = i;
		}

		return out;
	}

	static inline ContactArea projectTriangleEdge(mthz::Vec3 edge_p1, mthz::Vec3 edge_p2, uint32_t edge_id, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<mthz::NVec<2>>(), std::vector<int>(), -1, EDGE };

		out.ps = { mthz::NVec<2>{ edge_p1.dot(u), edge_p1.dot(w) }, mthz::NVec<2>{ edge_p2.dot(u), edge_p2.dot(w) } };
		out.p_IDs = { static_cast<int>(edge_id), static_cast<int>(edge_id) };

		return out;
	}

	static void findTriangleContactFeature(const TransformedTriangle& t, mthz::Vec3 n, int max_p_id, ContactAreaOrigin* feature_type_out, int* closest_feature_index_out) {
		double cos_ang = -t.normal.dot(n);

		// check if face
		if (1 - cos_ang <= COS_TOL) {
			*feature_type_out = FACE;
			return;
		}

		//check if edge
		for (int i = 0; i < 3; i++) {
			if (i != max_p_id && (i+1)%3 != max_p_id) {
				continue;
			}

			mthz::Vec3 p1 = t.vertices[i].p;
			mthz::Vec3 p2 = t.vertices[(i + 1) % 3].p;
			double sin_ang = abs((p2 - p1).normalize().dot(n));
			if (sin_ang <= SIN_TOL) {
				*feature_type_out = EDGE;
				*closest_feature_index_out = i;
				return;
			}
		}

		//is vertex
		*feature_type_out = VERTEX;
		*closest_feature_index_out = max_p_id;
	}

	static ContactArea findTriangleContactAreaAndCheckGaussMapSatisfied(const TransformedTriangle& t, mthz::Vec3 n, mthz::Vec3 p, int p_ID, mthz::Vec3 u, mthz::Vec3 w, bool* did_closest_feature_satisfy_gauss_map) {

		double cos_ang = t.normal.dot(n);
		if (1 - cos_ang <= COS_TOL) {
			*did_closest_feature_satisfy_gauss_map = true;
			return projectTriangleFace(t, u, w);
		}

		for (int i = 0; i < 3; i++) {
			// todo make not terrible
			if (i != p_ID && (i+1)%3 != p_ID) {
				continue;
			}

			mthz::Vec3 p1 = t.vertices[i].p;
			mthz::Vec3 p2 = t.vertices[(i+1)%3].p;
			double sin_ang = abs((p2 - p1).normalize().dot(n));
			if (sin_ang <= SIN_TOL) {
				*did_closest_feature_satisfy_gauss_map = normSatisfiesEdgeGaussArc(t.edges[i], n);
				return projectTriangleEdge(p1, p2, t.edges[i].id, n, p, u, w);
			}
		}

		for (const StaticMeshVertex& v : t.vertices) {
			if (v.self_index != static_cast<uint32_t>(p_ID)) continue;
			*did_closest_feature_satisfy_gauss_map = normSatisfiesVertexGaussMap(v, n);
		}

		return ContactArea{
			{ mthz::NVec<2>{ p.dot(u), p.dot(w) } },
			{ p_ID },
			-1,
			VERTEX
		};
	}

	static inline ContactArea projectCylinderFace(const std::vector<mthz::Vec3> face_verts, mthz::Vec3 u, mthz::Vec3 w, int face_id, int point_id_offset) {
		uint32_t n_points = static_cast<int>(face_verts.size());
		ContactArea out = { std::vector<mthz::NVec<2>>(n_points), std::vector<int>(n_points), face_id, FACE };

		for (uint32_t i = 0; i < n_points; i++) {
			out.ps[i] = { face_verts[i].dot(u), face_verts[i].dot(w) };
			out.p_IDs[i] = i + point_id_offset;
		}

		return out;
	}

	struct PPAir {
		mthz::Vec3 p1, p2;
	};

	PPAir cylinderLengthwiseLineInDirection(mthz::Vec3 center, mthz::Vec3 height_axis, double radius, double height, mthz::Vec3 n) {
		mthz::Vec3 barrel_axis = (n - height_axis * height_axis.dot(n)).normalize();
		mthz::Vec3 p1 = center + barrel_axis * radius + 0.5 * height * height_axis;
		mthz::Vec3 p2 = p1 - height * height_axis;
		return { p1, p2 };
	}

	static ContactArea findCylinderContactArea(const Cylinder& c, mthz::Vec3 n, mthz::Vec3 u, mthz::Vec3 w) {
		mthz::Vec3 height_axis = c.getHeightAxis();
		if (1 - abs(height_axis.dot(n)) <= COS_TOL && n.dot(height_axis) > 0) {
			return projectCylinderFace(c.getTopFaceApprox(), u, w, c.getTopSurfaceID(), c.getTopApproxPointIDOffset());
		}
		else if (1 - abs(height_axis.dot(n)) <= COS_TOL) {
			return projectCylinderFace(c.getBotFaceApprox(), u, w, c.getBotSurfaceID(), c.getBotApproxPointIDOffset());
		}
		else if (abs(height_axis.dot(n)) <= SIN_TOL) {
			PPAir line = cylinderLengthwiseLineInDirection(c.getCenter(), height_axis, c.getRadius(), c.getHeight(), n);
			return ContactArea{ {{line.p1.dot(u), line.p1.dot(w)}, {line.p2.dot(u), line.p2.dot(w)}}, {c.getTopEdgeID(), c.getBotEdgeID()}, -1, CYLINDER_BARREL};
		}
		else if (n.dot(height_axis) > 0) {
			mthz::Vec3 p = Cylinder::getExtremaOfDisk(c.getTopDiskCenter(), height_axis, c.getRadius(), n);
			return ContactArea{ {{p.dot(u), p.dot(w)}}, {c.getTopEdgeID()}, -1, EDGE};
		}
		else {
			mthz::Vec3 p = Cylinder::getExtremaOfDisk(c.getBotDiskCenter(), height_axis, c.getRadius(), n);
			return ContactArea{ {{p.dot(u), p.dot(w)}}, {c.getBotEdgeID()}, -1, EDGE};
		}
	}

	static ContactArea findCapsuleContactArea(const Capsule& c, mthz::Vec3 n, mthz::Vec3 u, mthz::Vec3 w) {
		mthz::Vec3 height_axis = c.getHeightAxis();
		if (abs(height_axis.dot(n)) <= SIN_TOL) {
			PPAir line = cylinderLengthwiseLineInDirection(c.getCenter(), c.getHeightAxis(), c.getRadius(), c.getDrumHeight(), n);
			return ContactArea{ {{line.p1.dot(u), line.p1.dot(w)}, {line.p2.dot(u), line.p2.dot(w)}}, {c.getTopCapID(), c.getBotCapID()}, -1, CYLINDER_BARREL};
		}
		else if (n.dot(height_axis) > 0) {
			mthz::Vec3 p = n * c.getRadius() + c.getCenter() + c.getHeightAxis() * c.getDrumHeight() / 2.0;
			return ContactArea{ {{p.dot(u), p.dot(w)}}, {c.getTopCapID()}, -1, EDGE };
		}
		else {
			mthz::Vec3 p = n * c.getRadius() + c.getCenter() - c.getHeightAxis() * c.getDrumHeight() / 2.0;
			return ContactArea{ {{p.dot(u), p.dot(w)}}, {c.getBotCapID()}, -1, EDGE };
		}
	}

	//let p1, p2 be sequential vertices on the counter-clockwise path around a polygon.
	//returns the perpendicular normal vector pointing inwards towards the polygon
	mthz::NVec<2> getInDirOfEdge(mthz::NVec<2> p1, mthz::NVec<2> p2) {
		return mthz::NVec<2>{ p1.v[1] - p2.v[1], p2.v[0] - p1.v[0] }.norm();
	}

	static bool isWindingCounterClockwise(const ContactArea& c) {
		mthz::NVec<2> in_dir_if_counter_clockwise = getInDirOfEdge(c.ps[0], c.ps[1]);
		for (int i = 2; i < c.ps.size(); i++) {
			double d = (c.ps[i] - c.ps[0]).dot(in_dir_if_counter_clockwise);
			if      (d > 0.0000000001) return true;
			else if (d < -0.0000000001) return false;
		}

		//this only would happen if either c has 2 or less points, or all points are colinear. neither should be happening.
		assert(false);
		return false;
	}

	struct ProjectedContactPoint {
		mthz::NVec<2> pos;
		uint64_t magic;
	};

	struct ClipEvaluationPoint {
		mthz::NVec<2> pos;
		int32_t source_id;
		uint64_t magic_id;
		bool edge_can_be_skipped_when_clipping;
	};

	std::vector<ClipEvaluationPoint> createClipEvaluationPoly(const ContactArea& c, uint32_t other_area_surface_id, bool flip_magics) {
		std::vector<ClipEvaluationPoint> out;
		out.reserve(c.ps.size());
		for (int i = 0; i < c.ps.size(); i++) {
			uint64_t m = 0;
			if (flip_magics) {
				m |= 0x00000000FFFFFFFF & other_area_surface_id;
				m |= 0xFFFFFFFF00000000 & (uint64_t(c.p_IDs[i]) << 32);
			}
			else {
				m |= 0x00000000FFFFFFFF & c.p_IDs[i];
				m |= 0xFFFFFFFF00000000 & (uint64_t(other_area_surface_id) << 32);
			}

			out.push_back(ClipEvaluationPoint{ c.ps[i], c.p_IDs[i], m, false });
		}
		if (c.ps.size() > 2 && !isWindingCounterClockwise(c)) std::reverse(out.begin(), out.end());

		return out;
	}
	
	ClipEvaluationPoint getEdgeIntersectionWithClippingEdge(ClipEvaluationPoint edge1, ClipEvaluationPoint edge2, mthz::NVec<2> clip_edge_norm, mthz::NVec<2> clipping_edge_sample_point, int32_t clipping_edge_id, bool flip_magics) {
		mthz::NVec<2> d = edge2.pos - edge1.pos;
		mthz::NVec<2> intersection_pos = edge1.pos + d * (clip_edge_norm.dot(clipping_edge_sample_point - edge1.pos) / clip_edge_norm.dot(d));

		//we want to perserve the id of the vertex that will be eliminated, which is the one behind the clipping edge.
		//the eliminated point will have a lower value when dotted with clip_edge_norm, so we can use d to figure out which one it is.
		bool edge2_is_perserved = d.dot(clip_edge_norm) > 0;
		int32_t perserved_source_id = edge2_is_perserved ? edge1.source_id : edge2.source_id;

		uint64_t m = 0;
		if (flip_magics) {
			m |= 0x00000000FFFFFFFF & clipping_edge_id;
			m |= 0xFFFFFFFF00000000 & (uint64_t(getEdgeID(edge1.source_id, edge2.source_id)) << 32);
		}
		else {
			m |= 0x00000000FFFFFFFF & getEdgeID(edge1.source_id, edge2.source_id);
			m |= 0xFFFFFFFF00000000 & (uint64_t(clipping_edge_id) << 32);
		}

		//if edge1 is perserved, then this will be the p1 of an edge that lies exactly on the clipping edge.
		//Clipping edges are formed from the boundry of the contact area, so we already know all points in that area are going to be perserved.
		bool edge_can_be_skipped_when_clipping = !edge2_is_perserved;
		return ClipEvaluationPoint{ intersection_pos, perserved_source_id, m, edge_can_be_skipped_when_clipping };
	}

	enum ClipResult {
		BOTH_CLIPPED, P1_EXACTLY_ON_EDGE_P2_CLIPPED, P2_EXACTLY_ON_EDGE_P1_CLIPPED, NEITHER_CLIPPED, ONLY_P1_CLIPPED, ONLY_P2_CLIPPED
	};

	ClipResult checkSegment(double p1_v, double p2_v, double clip_v) {
		if (p1_v >= clip_v && p2_v >= clip_v) { return NEITHER_CLIPPED; }
		if (p1_v == clip_v && p2_v < clip_v)  { return P1_EXACTLY_ON_EDGE_P2_CLIPPED; }
		if (p1_v < clip_v && p2_v == clip_v)  { return P2_EXACTLY_ON_EDGE_P1_CLIPPED; }
		if (p1_v < clip_v && p2_v >= clip_v)  { return ONLY_P1_CLIPPED; }
		if (p1_v >= clip_v && p2_v < clip_v)  { return ONLY_P2_CLIPPED; }
		                                        return BOTH_CLIPPED;
	}

	std::vector<ClipEvaluationPoint> getClipEvaluatinPolyAfterClippingByEdge(const std::vector<ClipEvaluationPoint>& c, mthz::NVec<2> clip_maintain_side, mthz::NVec<2> clipping_edge_sample_point, int32_t clipping_edge_id, bool flip_magics) {
		std::vector<ClipEvaluationPoint> out;

		double clip_v = clipping_edge_sample_point.dot(clip_maintain_side);

		if (c.size() >= 3) {
			//polygon case
			int i = 0;
			double p1_v, p2_v;
			ClipEvaluationPoint p1, p2;
			for (int i = 0; i < c.size(); i++) {
				if (i > 0) {
					// we computed these last iteration, no need to check again.
					p1_v = p2_v;
					p1 = p2;
				}
				else {
					// can't use last itr calc as it doesn't exist
					p1 = c[0];
					p1_v = p1.pos.dot(clip_maintain_side);
				}
				p2 = c[(i + 1) % c.size()];
				p2_v = p2.pos.dot(clip_maintain_side);

				ClipResult r = checkSegment(p1_v, p2_v, clip_v);

				if (r == NEITHER_CLIPPED || r == P1_EXACTLY_ON_EDGE_P2_CLIPPED) {
					// p1 is preserved. adding p2 or not is handled in the next iteration
					out.push_back(p1);
				}
				else if (r == ONLY_P2_CLIPPED) {
					// p1 is preserved. add the clipped second point. p2 should be discarded in the next iteration.
					out.push_back(p1);
					out.push_back(getEdgeIntersectionWithClippingEdge(p1, p2, clip_maintain_side, clipping_edge_sample_point, clipping_edge_id, flip_magics));
				}
				else if (r == P2_EXACTLY_ON_EDGE_P1_CLIPPED) {
					// elminate p1 by not adding anything. no need to find intersection as it would duplicate p2. next iteration should add p2.
				}
				else if (r == ONLY_P1_CLIPPED) {
					// add the new value after clipping p2. the next iteration should add p2 to preserve it.
					out.push_back(getEdgeIntersectionWithClippingEdge(p1, p2, clip_maintain_side, clipping_edge_sample_point, clipping_edge_id, flip_magics));
				}
				//else case: both p1 and p2 are eliminated. next iteration should not add p2.
			}

			return out;
		}
		else if (c.size() == 2){
			//line case
			double p1_v = c[0].pos.dot(clip_maintain_side);
			double p2_v = c[1].pos.dot(clip_maintain_side);
			ClipResult r = checkSegment(p1_v, p2_v, clip_v);
			if (r == NEITHER_CLIPPED) {
				return c;
			}
			if (r == P1_EXACTLY_ON_EDGE_P2_CLIPPED) {
				out = { c[0] };
				return out;
			}
			if (r == P2_EXACTLY_ON_EDGE_P1_CLIPPED) {
				out = { c[1] };
				return out;
			}
			if (r == ONLY_P1_CLIPPED) {
				out = { getEdgeIntersectionWithClippingEdge(c[0], c[1], clip_maintain_side, clipping_edge_sample_point, clipping_edge_id, flip_magics), c[1]};
				return out;
			}
			if (r == ONLY_P2_CLIPPED) {
				out = { c[0], getEdgeIntersectionWithClippingEdge(c[0], c[1], clip_maintain_side, clipping_edge_sample_point, clipping_edge_id, flip_magics) };
				return out;
			}
			// both clipped case
			return out;
		}
		else {
			//single point case
			double p1_v = c[0].pos.dot(clip_maintain_side);
			if (p1_v >= clip_v) { return c; } //single point preserved
			return out;                       //single point discarded
		}
	}

	std::vector<ClipEvaluationPoint> clipC1ByAllEdgesOfC2(std::vector<ClipEvaluationPoint> c1, const std::vector<ClipEvaluationPoint>& c2, bool flip_magics) {
		for (int i = 0; i < c2.size(); i++) {
			ClipEvaluationPoint e1 = c2[i];
			if (e1.edge_can_be_skipped_when_clipping) continue;

			ClipEvaluationPoint e2 = c2[(i + 1) % c2.size()];
			mthz::NVec<2> clip_dir = getInDirOfEdge(e1.pos, e2.pos);
			int32_t edge_id = getEdgeID(e1.source_id, e2.source_id);
			c1 = getClipEvaluatinPolyAfterClippingByEdge(c1, clip_dir, e1.pos, edge_id, flip_magics);
			if (c1.size() == 0) { return c1; } //nothing left to clip
		}
		return c1;
	}

	static std::vector<ProjectedContactPoint> clipContacts(const ContactArea& c1, const ContactArea& c2) {
		std::vector<ClipEvaluationPoint> poly1 = createClipEvaluationPoly(c1, c2.surfaceID, false);
		std::vector<ClipEvaluationPoint> poly2 = createClipEvaluationPoly(c2, c1.surfaceID, true);
		std::vector<ClipEvaluationPoint> out_poly;

		//trivial single point cases
		if (poly1.size() == 1) {
			//assert(poly2.size() >= 3);
			out_poly = poly1;
		}
		else if (poly2.size() == 1) {
			//assert(poly1.size() >= 3);
			out_poly = poly2;
		}
		//edge v edge case
		else if (poly1.size() == 2 && poly2.size() == 2) {
			mthz::NVec<2> norm = getInDirOfEdge(poly2[0].pos, poly2[1].pos);
			ClipEvaluationPoint intersection = getEdgeIntersectionWithClippingEdge(poly1[0], poly1[1], norm, poly2[0].pos, getEdgeID(poly2[0].source_id, poly2[1].source_id), false);
			out_poly = { intersection };
		}
		//edge v poly
		else if (poly1.size() == 2) {
			out_poly = clipC1ByAllEdgesOfC2(poly1, poly2, false);
		}
		//poly v edge
		else if (poly2.size() == 2) {
			out_poly = clipC1ByAllEdgesOfC2(poly2, poly1, true);
		}
		//poly v poly
		else {
			poly1 = clipC1ByAllEdgesOfC2(poly1, poly2, false);
			out_poly = clipC1ByAllEdgesOfC2(poly2, poly1, true);
		}

		std::vector<ProjectedContactPoint> out;
		out.reserve(out_poly.size());
		for (ClipEvaluationPoint p : out_poly) {
			out.push_back(ProjectedContactPoint{ p.pos, p.magic_id });
		}

		//assert(out.size() > 0);
		return out;
	}

	ExtremaInfo recenter(const ExtremaInfo& info, double old_ref_value, double new_ref_value) {
		double diff = new_ref_value - old_ref_value;
		return ExtremaInfo{ info.min_pID, info.max_pID, info.min_val + diff, info.max_val + diff};
	}

	ExtremaInfo findExtrema(const Polyhedron& c, mthz::Vec3 axis) {
		ExtremaInfo extrema;

		for (int i = 0; i < c.getPoints().size(); i++) {
			mthz::Vec3 p = c.getPoints()[i];
			double val = p.dot(axis);
			if (val < extrema.min_val) {
				extrema.min_pID = i;
				extrema.min_val = val;
			}
			if (val > extrema.max_val) {
				extrema.max_pID = i;
				extrema.max_val = val;
			}
		}

		return extrema;
	}

	ExtremaInfo getSphereExtrema(const Sphere& s, mthz::Vec3 dir) {
		ExtremaInfo out;
		double center_val = dir.dot(s.getCenter());
		out.max_val = center_val + s.getRadius();
		out.min_val = center_val - s.getRadius();
		out.min_pID = -1;
		out.max_pID = -1;

		return out;
	}

	ExtremaInfo getCylinderExtrema(const Cylinder& c, mthz::Vec3 dir) {
		ExtremaInfo out;
		mthz::Vec3 center = c.getCenter();
		mthz::Vec3 height_axis = c.getHeightAxis();
		double height = c.getHeight();
		double radius = c.getRadius();
		mthz::Vec3 topdisk_center = c.getTopDiskCenter();

		//exploiting symmetry
		mthz::Vec3 topdisk_max = Cylinder::getExtremaOfDisk(topdisk_center, height_axis, radius, dir);
		mthz::Vec3 topdisk_min = 2 * topdisk_center - topdisk_max;
		mthz::Vec3 botdisk_max = 2 * center - topdisk_min;
		mthz::Vec3 botdisk_min = 2 * center - topdisk_max;

		
		double topdisk_max_v = dir.dot(topdisk_max);
		double topdisk_min_v = dir.dot(topdisk_min);
		double botdisk_max_v = dir.dot(botdisk_max);
		double botdisk_min_v = dir.dot(botdisk_min);

		out.max_val = std::max<double>(topdisk_max_v, botdisk_max_v);
		out.min_val = std::min<double>(topdisk_min_v, botdisk_min_v);
		out.min_pID = -1;
		out.max_pID = -1;

		return out;
	}

	ExtremaInfo getCapsuleExtrema(const Capsule& c, mthz::Vec3 dir) {
		ExtremaInfo out;

		mthz::Vec3 cap_offset = c.getHeightAxis() * c.getDrumHeight() / 2.0;
		double ball1_center_val = dir.dot(c.getCenter() + cap_offset);
		double ball1_minval = ball1_center_val - c.getRadius();
		double ball1_max_val = ball1_center_val + c.getRadius();
		double ball2_center_val = dir.dot(c.getCenter() - cap_offset);
		double ball2_minval = ball2_center_val - c.getRadius();
		double ball2_max_val = ball2_center_val + c.getRadius();

		
		out.max_val = std::max<double>(ball1_max_val, ball2_max_val);
		out.min_val = std::min<double>(ball1_minval, ball2_minval);
		out.min_pID = -1;
		out.max_pID = -1;

		return out;
	}

	static CheckNormResults sat_checknorm(const ExtremaInfo& a_info, const ExtremaInfo& b_info, mthz::Vec3 n) {
		double forward_pen_depth = a_info.max_val - b_info.min_val;
		double reverse_pen_depth = b_info.max_val - a_info.min_val;

		if (forward_pen_depth < reverse_pen_depth) {
			return CheckNormResults{ a_info.max_pID, b_info.min_pID, n, forward_pen_depth };
		}
		else {
			return CheckNormResults{ a_info.min_pID, b_info.max_pID, -n, reverse_pen_depth };
		}
	}

	static Manifold SAT_PolyPoly(const Polyhedron& a, int a_id, const Material& a_mat, const Polyhedron& b, int b_id, const Material& b_mat, int surfaceid1, int surfaceid2) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& ag = a.getGaussMap();
		const GaussMap& bg = b.getGaussMap();

		for (const GaussVert& g : ag.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, findExtrema(b, g.v), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		for (const GaussVert& g : bg.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(b.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(findExtrema(a, g.v), recentered_g_extrema, g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		for (GaussArc arc1 : ag.arcs) {
			for (GaussArc arc2 : bg.arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx].v;
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx].v;
				mthz::Vec3 b1 = -bg.face_verts[arc2.v1_indx].v;
				mthz::Vec3 b2 = -bg.face_verts[arc2.v2_indx].v;

				//check arcs arent on opposite hemispheres
				mthz::Vec3 a_avg = a1 + a2;
				if (a_avg.dot(b1) + a_avg.dot(b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if (a_avg.dot(n) < 0) {
					n *= -1;
				}

				CheckNormResults x = sat_checknorm(findExtrema(a, n), findExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getPoints()[min_pen.a_maxPID];
		mthz::Vec3 b_maxP = b.getPoints()[min_pen.b_maxPID];

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		// switching to arrays on the stack rather than std::vector as the main data type would probably speed this up a lot.
		// TODO when trying to optimize this
		ContactArea a_contact = findContactArea(a, norm, a_maxP, min_pen.a_maxPID, u, w, surfaceid1, surfaceid2);
		ContactArea b_contact = findContactArea(b, (-1) * norm, b_maxP, min_pen.b_maxPID, u, w, surfaceid1, surfaceid2);

		if (a_contact.origin == FACE) { out.surfaceid1 = a_contact.surfaceID; }
		else                          { out.surfaceid1 = -1; }
		if (b_contact.origin == FACE) { out.surfaceid2 = b_contact.surfaceID; }
		else                          { out.surfaceid2 = -1; }

		std::vector<ProjectedContactPoint> manifold_pool = clipContacts(a_contact, b_contact);
		assert(manifold_pool.size() > 0);
		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b_mat.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold detectSphereSphere(const Sphere& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat) {
		Manifold out;
		mthz::Vec3 diff = b.getCenter() - a.getCenter();
		double center_distance = diff.mag();
		out.max_pen_depth = a.getRadius() + b.getRadius() - center_distance;
		
		//check if spheres touch or not
		if (out.max_pen_depth < 0) {
			return out;
		}

		out.normal = diff.normalize();

		ContactP cp;
		cp.pos = a.getCenter() + out.normal * a.getRadius();
		cp.pen_depth = out.max_pen_depth;
		
		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);
		cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
		cp.s1_cfm = a_mat.cfm;
		cp.s2_cfm = b_mat.cfm;
		cp.magicID = MagicID{ cID, 0x0 }; //second term is used to identify different points or faces on polyhedron. just using flat 0 for spheres.

		out.points.push_back(cp);
		return out;
	}

	static Manifold detectSphereCapsule(const Sphere& a, int a_id, const Material& a_mat, const Capsule& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		mthz::Vec3 height_axis = b.getHeightAxis();
		mthz::Vec3 diff = b.getCenter() - a.getCenter();
		mthz::Vec3 barrel_axis = (diff - height_axis * height_axis.dot(diff)).normalize();
		{
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, barrel_axis), getCapsuleExtrema(b, barrel_axis), barrel_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		mthz::Vec3 cap1_center = b.getCenter() + height_axis * b.getDrumHeight() / 2.0;
		mthz::Vec3 sphere_to_cap1_axis = (cap1_center - a.getCenter()).normalize();
		{
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, sphere_to_cap1_axis), getCapsuleExtrema(b, sphere_to_cap1_axis), sphere_to_cap1_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		mthz::Vec3 cap2_center = b.getCenter() - height_axis * b.getDrumHeight() / 2.0;
		mthz::Vec3 sphere_to_cap2_axis = (cap2_center - a.getCenter()).normalize();
		{
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, sphere_to_cap2_axis), getCapsuleExtrema(b, sphere_to_cap2_axis), sphere_to_cap2_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		
		out.normal = min_pen.norm;

		ContactP cp;
		cp.pos = a.getCenter() + out.normal * a.getRadius();
		cp.pen_depth = min_pen.pen_depth;
		cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
		cp.s1_cfm = a_mat.cfm;
		cp.s2_cfm = b_mat.cfm;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		cp.magicID = MagicID{ cID, 0 }; //not bothering with featureid

		out.points.push_back(cp);

		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}


	static bool checkCylinderEdgeApproxVsBarrel(const Cylinder& a, const Cylinder& b, mthz::Vec3 barrel_height_axis, const std::vector<mthz::Vec3> edge_approx, CheckNormResults* out) {
		uint32_t n = static_cast<uint32_t>(edge_approx.size());
		for (uint32_t i = 0; i < n; i++) {
			mthz::Vec3 edge_dir = edge_approx[(i + 1) % n] - edge_approx[i];
			mthz::Vec3 dir = edge_dir.cross(barrel_height_axis);
			if (dir.mag() < 0.00000000001) continue;

			mthz::Vec3 dir_normed = dir.normalize();
			CheckNormResults x = sat_checknorm(getCylinderExtrema(a, dir_normed), getCylinderExtrema(b, dir_normed), dir_normed);
			if (x.seprAxisExists()) {
				return true;
			}
			else if (x.pen_depth < out->pen_depth) {
				*out = x;
			}
		}

		return false;
	}

	static Manifold detectCylinderCylinder(const Cylinder& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		mthz::Vec3 a_height_axis = a.getHeightAxis();
		{
			CheckNormResults x = sat_checknorm(getCylinderExtrema(a, a_height_axis), getCylinderExtrema(b, a_height_axis), a_height_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		mthz::Vec3 b_height_axis = b.getHeightAxis();
		{
			CheckNormResults x = sat_checknorm(getCylinderExtrema(a, b_height_axis), getCylinderExtrema(b, b_height_axis), b_height_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		//checking edge vs barrel for both edges on both cylinders
		bool sepr_axis_exists = checkCylinderEdgeApproxVsBarrel(a, b, b_height_axis, a.getTopFaceApprox(), &min_pen);
		if (sepr_axis_exists) {
			out.max_pen_depth = -1;
			return out;
		}
		sepr_axis_exists = checkCylinderEdgeApproxVsBarrel(a, b, b_height_axis, a.getBotFaceApprox(), &min_pen);
		if (sepr_axis_exists) {
			out.max_pen_depth = -1;
			return out;
		}
		sepr_axis_exists = checkCylinderEdgeApproxVsBarrel(a, b, a_height_axis, b.getTopFaceApprox(), &min_pen);
		if (sepr_axis_exists) {
			out.max_pen_depth = -1;
			return out;
		}
		sepr_axis_exists = checkCylinderEdgeApproxVsBarrel(a, b, a_height_axis, b.getBotFaceApprox(), &min_pen);
		if (sepr_axis_exists) {
			out.max_pen_depth = -1;
			return out;
		}

		mthz::Vec3 barrel_barrel_axis = a_height_axis.cross(b_height_axis);
		if (barrel_barrel_axis.magSqrd() > 0.00000001) {
			barrel_barrel_axis = barrel_barrel_axis.normalize();
			CheckNormResults x = sat_checknorm(getCylinderExtrema(a, barrel_barrel_axis), getCylinderExtrema(b, barrel_barrel_axis), barrel_barrel_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		std::vector<mthz::Vec3> a_gauss_verts = a.getGuassVerts();
		std::vector<mthz::Vec3> b_gauss_verts = b.getGuassVerts();
		for (GaussArc arc1 : a.getGuassArcs()) {
			for (GaussArc arc2 : b.getGuassArcs()) {

				mthz::Vec3 a1 = a_gauss_verts[arc1.v1_indx];
				mthz::Vec3 a2 = a_gauss_verts[arc1.v2_indx];
				mthz::Vec3 b1 = -b_gauss_verts[arc2.v1_indx];
				mthz::Vec3 b2 = -b_gauss_verts[arc2.v2_indx];

				//check arcs arent on opposite hemispheres
				mthz::Vec3 a_avg = a1 + a2;
				if (a_avg.dot(b1) + a_avg.dot(b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if (a_avg.dot(n) < 0) {
					n *= -1;
				}

				CheckNormResults x = sat_checknorm(getCylinderExtrema(a, n), getCylinderExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findCylinderContactArea(a, norm, u, w);
		ContactArea b_contact = findCylinderContactArea(b, -norm, u, w);
		
		std::vector<ProjectedContactPoint> manifold_pool;

		if (a_contact.origin == EDGE) {
			manifold_pool = { ProjectedContactPoint{a_contact.ps[0], 0x0} };
		}
		else if (b_contact.origin == EDGE) {
			manifold_pool = { ProjectedContactPoint{ b_contact.ps[0], 0x0} };
		}
		else if (a_contact.origin == CYLINDER_BARREL && b_contact.origin == CYLINDER_BARREL && abs(a_height_axis.dot(b_height_axis)) > 0.995) {
			//since the two contact areas are parralel lines, the general clipping doesn't handle this case well.
			PPAir a_contact_line = cylinderLengthwiseLineInDirection(a.getCenter(), a.getHeightAxis(), a.getRadius(), a.getHeight(), norm);
			PPAir b_contact_line = cylinderLengthwiseLineInDirection(b.getCenter(), b.getHeightAxis(), b.getRadius(), b.getHeight(), -norm);

			//taking advantage of the fact that cylinderLengthwiseLineInDirection() will return p1, p2 such that p1 - p2 is the direction of height axis.
			//ensure b.p1 - b.p2 is in the direction of a_height_axis
			if (b_height_axis.dot(a_height_axis) < 0) std::swap(b_contact_line.p1, b_contact_line.p2);

			//treat the two contacts as 1-dimensional line segments along a_height_axis and find the intersection
			double a_max_v = a_contact_line.p1.dot(a_height_axis);
			double a_min_v = a_contact_line.p2.dot(a_height_axis);
			double b_max_v = b_contact_line.p1.dot(a_height_axis);
			double b_min_v = b_contact_line.p2.dot(a_height_axis);

			mthz::Vec3 intersection_max = (a_max_v > b_max_v) ? b_contact_line.p1 : a_contact_line.p1;
			mthz::Vec3 intersection_min = (a_min_v > b_min_v) ? a_contact_line.p2 : b_contact_line.p2;

			manifold_pool = {
				ProjectedContactPoint{ mthz::NVec<2>{u.dot(intersection_max), w.dot(intersection_max)}, 0x0 },
				ProjectedContactPoint{ mthz::NVec<2>{u.dot(intersection_min), w.dot(intersection_min)}, 0x1 },
			};
		}
		else {
			manifold_pool = clipContacts(a_contact, b_contact);
		}

		mthz::Vec3 a_maxP = a_height_axis.dot(norm) > 0 ?
			Cylinder::getExtremaOfDisk(a.getTopDiskCenter(), a_height_axis, a.getRadius(), norm)
		  :	Cylinder::getExtremaOfDisk(a.getBotDiskCenter(), a_height_axis, a.getRadius(), norm);

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b_mat.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold detectCapsuleCapsule(const Capsule& a, int a_id, const Material& a_mat, const Capsule& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		mthz::Vec3 a_topcap_center = a.getCenter() + a.getHeightAxis() * a.getDrumHeight() / 2.0;
		mthz::Vec3 a_botcap_center = a.getCenter() - a.getHeightAxis() * a.getDrumHeight() / 2.0;
		mthz::Vec3 b_topcap_center = b.getCenter() + b.getHeightAxis() * b.getDrumHeight() / 2.0;
		mthz::Vec3 b_botcap_center = b.getCenter() - b.getHeightAxis() * b.getDrumHeight() / 2.0;


		//dont like the reption here and it looks very inefficient, but ill rewrite it later.
		// a top cap vs b top cap
		{
			mthz::Vec3 n = (b_topcap_center - a_topcap_center).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, n), getCapsuleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// a top cap vs b bot cap
		{
			mthz::Vec3 n = (b_topcap_center - a_botcap_center).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, n), getCapsuleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// a bit cap vs b bot cap
		{
			mthz::Vec3 n = (b_botcap_center - a_botcap_center).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, n), getCapsuleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// a top cap vs b barrel
		{
			mthz::Vec3 diff = b.getCenter() - a_topcap_center;
			mthz::Vec3 barrel_axis = (diff - b.getHeightAxis() * b.getHeightAxis().dot(diff)).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_axis), getCapsuleExtrema(b, barrel_axis), barrel_axis);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// a bot cap vs b barrel
		{
			mthz::Vec3 diff = b.getCenter() - a_botcap_center;
			mthz::Vec3 barrel_axis = (diff - b.getHeightAxis() * b.getHeightAxis().dot(diff)).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_axis), getCapsuleExtrema(b, barrel_axis), barrel_axis);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// a barrel vs b top cap
		{
			mthz::Vec3 diff = b.getCenter() - b_topcap_center;
			mthz::Vec3 barrel_axis = (diff - a.getHeightAxis() * a.getHeightAxis().dot(diff)).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_axis), getCapsuleExtrema(b, barrel_axis), barrel_axis);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// a barrel vs b bot cap
		{
			mthz::Vec3 diff = b.getCenter() - b_botcap_center;
			mthz::Vec3 barrel_axis = (diff - a.getHeightAxis() * a.getHeightAxis().dot(diff)).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_axis), getCapsuleExtrema(b, barrel_axis), barrel_axis);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// barrel v barrel
		mthz::Vec3 barrel_barrel_axis = a.getHeightAxis().cross(b.getHeightAxis());
		if (barrel_barrel_axis.magSqrd() > 0.00000001) {
			barrel_barrel_axis = barrel_barrel_axis.normalize();
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_barrel_axis), getCapsuleExtrema(b, barrel_barrel_axis), barrel_barrel_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findCapsuleContactArea(a, norm, u, w);
		ContactArea b_contact = findCapsuleContactArea(b, -norm, u, w);

		std::vector<ProjectedContactPoint> manifold_pool;

		if (a_contact.origin == CYLINDER_BARREL && b_contact.origin == CYLINDER_BARREL && abs(a.getHeightAxis().dot(b.getHeightAxis())) > 0.995) {
			//todo: dont like this. the threshold of 0.995 to perform this case is way too low, and results in noticeable artifacts
			
			//since the two contact areas are parralel lines, the general clipping doesn't handle this case well.
			PPAir a_contact_line = cylinderLengthwiseLineInDirection(a.getCenter(), a.getHeightAxis(), a.getRadius(), a.getDrumHeight(), norm);
			PPAir b_contact_line = cylinderLengthwiseLineInDirection(b.getCenter(), b.getHeightAxis(), b.getRadius(), b.getDrumHeight(), -norm);

			//taking advantage of the fact that cylinderLengthwiseLineInDirection() will return p1, p2 such that p1 - p2 is the direction of height axis.
			//ensure b.p1 - b.p2 is in the direction of a_height_axis
			if (b.getHeightAxis().dot(a.getHeightAxis()) < 0) std::swap(b_contact_line.p1, b_contact_line.p2);

			//treat the two contacts as 1-dimensional line segments along a_height_axis and find the intersection
			double a_max_v = a_contact_line.p1.dot(a.getHeightAxis());
			double a_min_v = a_contact_line.p2.dot(a.getHeightAxis());
			double b_max_v = b_contact_line.p1.dot(a.getHeightAxis());
			double b_min_v = b_contact_line.p2.dot(a.getHeightAxis());

			mthz::Vec3 intersection_max = (a_max_v > b_max_v) ? b_contact_line.p1 : a_contact_line.p1;
			mthz::Vec3 intersection_min = (a_min_v > b_min_v) ? a_contact_line.p2 : b_contact_line.p2;

			manifold_pool = {
				ProjectedContactPoint{ mthz::NVec<2>{u.dot(intersection_max), w.dot(intersection_max)}, 0x0 },
				ProjectedContactPoint{ mthz::NVec<2>{u.dot(intersection_min), w.dot(intersection_min)}, 0x1 },
			};
		}
		else {
			manifold_pool = clipContacts(a_contact, b_contact);
		}

		mthz::Vec3 a_maxP = a.getHeightAxis().dot(norm) > 0 ?
			a_topcap_center + norm * a.getRadius()
			: a_botcap_center + norm * a.getRadius();

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b_mat.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold SAT_PolySphere(const Polyhedron& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& gauss_map = a.getGaussMap();
		uint32_t a_feature_id;

		//very hacky using the fact that gauss verts have the same order as the corresponding surfaces they are made from. SurfaceID's start at points.size().
		uint32_t corresponding_surface_id = static_cast<uint32_t>(a.getPoints().size());
		for (const GaussVert& g : gauss_map.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, getSphereExtrema(b, g.v), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					a_feature_id = corresponding_surface_id;
					min_pen = x;
				}
			}

			corresponding_surface_id++;
		}
		for (int pID = 0; pID < a.getPoints().size(); pID++) {
			mthz::Vec3 p = a.getPoints()[pID];
			mthz::Vec3 n = (p - b.getCenter()).normalize();
			CheckNormResults x = sat_checknorm(findExtrema(a, n), getSphereExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = pID;
			}
		}
		for (const Edge& e : a.getEdges()) {
			mthz::Vec3 edge_dir = (e.p2() - e.p1()).normalize();
			mthz::Vec3 sample = e.p1() - b.getCenter();
			mthz::Vec3 n = (sample - edge_dir * edge_dir.dot(sample)).normalize();
			CheckNormResults x = sat_checknorm(findExtrema(a, n), getSphereExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = getEdgeID(e.p1_indx, e.p2_indx);
			}
		}

		out.normal = min_pen.norm;
		
		ContactP cp;
		cp.pos = b.getCenter() - out.normal * b.getRadius(); //sat_checknorm ensures normals always point away from a. to get normal pointing away from b - sign added
		cp.pen_depth = min_pen.pen_depth;
		cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
		cp.s1_cfm = a_mat.cfm;
		cp.s2_cfm = b_mat.cfm;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		cp.magicID = MagicID{ cID, a_feature_id }; //not bothering with featureid
 
		out.points.push_back(cp);
		
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold SAT_PolyCapsule(const Polyhedron& a, int a_id, const Material& a_mat, const Capsule &b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& gauss_map = a.getGaussMap();
		uint32_t a_feature_id;

		//very hacky using the fact that gauss verts have the same order as the corresponding surfaces they are made from. SurfaceID's start at points.size().
		uint32_t corresponding_surface_id = static_cast<uint32_t>(a.getPoints().size());
		for (const GaussVert& g : gauss_map.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, getCapsuleExtrema(b, g.v), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					a_feature_id = corresponding_surface_id;
					min_pen = x;
				}
			}

			corresponding_surface_id++;
		}

		mthz::Vec3 height_axis = b.getHeightAxis();
		mthz::Vec3 top_cap_center = b.getCenter() + height_axis * b.getDrumHeight();
		mthz::Vec3 bot_cap_center = b.getCenter() - height_axis * b.getDrumHeight();


		double t_v = top_cap_center.dot(height_axis);
		double b_v = bot_cap_center.dot(height_axis);
		for (int pID = 0; pID < a.getPoints().size(); pID++) {
			mthz::Vec3 p = a.getPoints()[pID];
			double v = p.dot(height_axis);

			mthz::Vec3 n;
			if (b_v <= v && v <= t_v) {
				// point is above the bottom cap but below the top cap. we should only check against the drum
				n = ((p - b.getCenter()) - height_axis * height_axis.dot(p - b.getCenter())).normalize();
			}
			else if (v > t_v) {
				// above the top cap, just compare against that
				n = (p - top_cap_center).normalize();
			}
			else {
				// below the bottom cap
				n = (p - bot_cap_center).normalize();
			}

			CheckNormResults x = sat_checknorm(findExtrema(a, n), getCapsuleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = pID;
			}
		}

		for (const Edge& e : a.getEdges()) {
			mthz::Vec3 edge_dir = (e.p2() - e.p1()).normalize();
			
			{
				// checking against the top cap
				mthz::Vec3 sample = e.p1() - top_cap_center;
				mthz::Vec3 n = (sample - edge_dir * edge_dir.dot(sample)).normalize();
				CheckNormResults x = sat_checknorm(findExtrema(a, n), getCapsuleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
					a_feature_id = getEdgeID(e.p1_indx, e.p2_indx);
				}
			}
			{
				// checking against the bot cap
				mthz::Vec3 sample = e.p1() - top_cap_center;
				mthz::Vec3 n = (sample - edge_dir * edge_dir.dot(sample)).normalize();
				CheckNormResults x = sat_checknorm(findExtrema(a, n), getCapsuleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
					a_feature_id = getEdgeID(e.p1_indx, e.p2_indx);
				}
			}
			{
				// checking against the barrel
				mthz::Vec3 dir = edge_dir.cross(height_axis);
				if (dir.mag() < 0.00000000001) continue;

				mthz::Vec3 n = dir.normalize();
				CheckNormResults x = sat_checknorm(findExtrema(a, n), getCapsuleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
					a_feature_id = getEdgeID(e.p1_indx, e.p2_indx);
				}
			}
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getPoints()[min_pen.a_maxPID];

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findContactArea(a, norm, a_maxP, min_pen.a_maxPID, u, w);
		ContactArea b_contact = findCapsuleContactArea(b, -norm, u, w);

		std::vector<ProjectedContactPoint> manifold_pool;

		manifold_pool = clipContacts(a_contact, b_contact);

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b_mat.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold SAT_PolyCylinder(const Polyhedron& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& ag = a.getGaussMap();

		//check poly face axis
		for (const GaussVert& g : ag.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, getCylinderExtrema(b, g.v), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		//check cylinder face axis
		mthz::Vec3 b_height_axis = b.getHeightAxis();
		{
			CheckNormResults x = sat_checknorm(findExtrema(a, b_height_axis), getCylinderExtrema(b, b_height_axis), b_height_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		//check edge collisions against the round body of the cylinder
		for (Edge e : a.getEdges()) {
			mthz::Vec3 edge_dir = e.p2() - e.p1();
			mthz::Vec3 dir = edge_dir.cross(b_height_axis);
			if (dir.mag() < 0.00000000001) continue;

			mthz::Vec3 dir_normed = dir.normalize();
			CheckNormResults x = sat_checknorm(findExtrema(a, dir_normed), getCylinderExtrema(b, dir_normed), dir_normed);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		//check vertex against cylinder
		for (mthz::Vec3 p : a.getPoints()) {
			mthz::Vec3 diff = p - b.getCenter();
			mthz::Vec3 n = (diff - b_height_axis * b_height_axis.dot(diff)).normalize();
			CheckNormResults x = sat_checknorm(findExtrema(a, n), getCylinderExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		//check edge edge
		for (GaussArc arc1 : ag.arcs) {
			for (GaussArc arc2 : b.getGuassArcs()) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx].v;
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx].v;
				mthz::Vec3 b1 = -b.getGuassVerts()[arc2.v1_indx];
				mthz::Vec3 b2 = -b.getGuassVerts()[arc2.v2_indx];

				//check arcs arent on opposite hemispheres
				mthz::Vec3 a_avg = a1 + a2;
				if (a_avg.dot(b1) + a_avg.dot(b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if (a_avg.dot(n) < 0) {
					n *= -1;
				}

				CheckNormResults x = sat_checknorm(findExtrema(a, n), getCylinderExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getPoints()[min_pen.a_maxPID];

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findContactArea(a, norm, a_maxP, min_pen.a_maxPID, u, w);
		ContactArea b_contact = findCylinderContactArea(b, -norm, u, w);

		std::vector<ProjectedContactPoint> manifold_pool;

		if (b_contact.origin == EDGE) {
			manifold_pool = { ProjectedContactPoint{ b_contact.ps[0], 0x0} };
		}
		else {
			manifold_pool = clipContacts(a_contact, b_contact);
		}

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b_mat.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	//still some room from optomization
	static Manifold detectSphereCylinder(const Sphere& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		mthz::Vec3 height_axis = b.getHeightAxis();
		{
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, height_axis), getCylinderExtrema(b, height_axis), height_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		mthz::Vec3 diff = b.getCenter() - a.getCenter();
		mthz::Vec3 barrel_axis = (diff - height_axis * height_axis.dot(diff)).normalize();
		{
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, barrel_axis), getCylinderExtrema(b, barrel_axis), barrel_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		
		mthz::Vec3 topdisk_center = b.getCenter() + height_axis * 0.5 * b.getHeight();
		mthz::Vec3 topdisk_close_point = Cylinder::getExtremaOfDisk(topdisk_center, height_axis, b.getRadius(), -diff.normalize());
		{
			mthz::Vec3 edge_axis = (topdisk_close_point - a.getCenter()).normalize();
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		mthz::Vec3 botdisk_center = b.getCenter() + height_axis * 0.5 * b.getHeight();
		mthz::Vec3 botdisk_close_point = Cylinder::getExtremaOfDisk(botdisk_center, height_axis, b.getRadius(), -diff.normalize());
		{
			mthz::Vec3 edge_axis = (botdisk_close_point - a.getCenter()).normalize();
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		out.normal = min_pen.norm;

		ContactP cp;
		cp.pos = a.getCenter() + out.normal * a.getRadius();
		cp.pen_depth = min_pen.pen_depth;
		cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
		cp.s1_cfm = a_mat.cfm;
		cp.s2_cfm = b_mat.cfm;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		cp.magicID = MagicID{ cID, 0 }; //not bothering with featureid

		out.points.push_back(cp);

		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold detectCapsuleCylinder(const Capsule& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		mthz::Vec3 a_height_axis = a.getHeightAxis();
		mthz::Vec3 b_height_axis = b.getHeightAxis();
		// cylinder faces
		{
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, b_height_axis), getCylinderExtrema(b, b_height_axis), b_height_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		//checking edge vs barrel
		uint32_t n = static_cast<uint32_t>(b.getTopFaceApprox().size());
		for (uint32_t i = 0; i < n; i++) {
			mthz::Vec3 edge_dir = b.getTopFaceApprox()[(i + 1) % n] - b.getTopFaceApprox()[i];
			mthz::Vec3 dir = edge_dir.cross(a.getHeightAxis());
			if (dir.mag() < 0.00000000001) continue;

			mthz::Vec3 dir_normed = dir.normalize();
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, dir_normed), getCylinderExtrema(b, dir_normed), dir_normed);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		mthz::Vec3 a_topcap_center = a.getCenter() + a.getHeightAxis() * a.getDrumHeight() / 2.0;
		mthz::Vec3 a_botcap_center = a.getCenter() - a.getHeightAxis() * a.getDrumHeight() / 2.0;

		//checking top edge vs top cap
		mthz::Vec3 topdisk_center = b.getCenter() + b_height_axis * 0.5 * b.getHeight();
		mthz::Vec3 topdisk_close_point = Cylinder::getExtremaOfDisk(topdisk_center, b_height_axis, b.getRadius(), (a_topcap_center - b.getCenter()).normalize());
		{
			mthz::Vec3 edge_axis = (topdisk_close_point - a_topcap_center).normalize();
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		//checking top edge vs bot cap
		topdisk_close_point = Cylinder::getExtremaOfDisk(topdisk_center, b_height_axis, b.getRadius(), (a_botcap_center - b.getCenter()).normalize());
		{
			mthz::Vec3 edge_axis = (topdisk_close_point - a_botcap_center).normalize();
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		//checking bot edge vs top cap
		mthz::Vec3 botdisk_center = b.getCenter() - b_height_axis * 0.5 * b.getHeight();
		mthz::Vec3 botdisk_close_point = Cylinder::getExtremaOfDisk(botdisk_center, b_height_axis, b.getRadius(), (a_topcap_center - b.getCenter()).normalize());
		{
			mthz::Vec3 edge_axis = (botdisk_close_point - a_topcap_center).normalize();
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		//checking bot edge vs bot cap
		botdisk_close_point = Cylinder::getExtremaOfDisk(botdisk_center, b_height_axis, b.getRadius(), (a_botcap_center - b.getCenter()).normalize());
		{
			mthz::Vec3 edge_axis = (botdisk_close_point - a_botcap_center).normalize();
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		// top cap vs b barrel
		{
			mthz::Vec3 diff = b.getCenter() - a_topcap_center;
			mthz::Vec3 barrel_axis = (diff - b.getHeightAxis() * b.getHeightAxis().dot(diff)).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_axis), getCylinderExtrema(b, barrel_axis), barrel_axis);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		// bot cap vs b barrel
		{
			mthz::Vec3 diff = b.getCenter() - a_botcap_center;
			mthz::Vec3 barrel_axis = (diff - b.getHeightAxis() * b.getHeightAxis().dot(diff)).normalize();
			{
				CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_axis), getCylinderExtrema(b, barrel_axis), barrel_axis);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		//barrel vs barrel
		mthz::Vec3 barrel_barrel_axis = a_height_axis.cross(b_height_axis);
		if (barrel_barrel_axis.magSqrd() > 0.00000001) {
			barrel_barrel_axis = barrel_barrel_axis.normalize();
			CheckNormResults x = sat_checknorm(getCapsuleExtrema(a, barrel_barrel_axis), getCylinderExtrema(b, barrel_barrel_axis), barrel_barrel_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findCapsuleContactArea(a, norm, u, w);
		ContactArea b_contact = findCylinderContactArea(b, -norm, u, w);

		std::vector<ProjectedContactPoint> manifold_pool;

		if (b_contact.origin == EDGE) {
			manifold_pool = { ProjectedContactPoint{ b_contact.ps[0], 0x0} };
		}
		else if (a_contact.origin == CYLINDER_BARREL && b_contact.origin == CYLINDER_BARREL && abs(a_height_axis.dot(b_height_axis)) > 0.995) {
			//since the two contact areas are parralel lines, the general clipping doesn't handle this case well.
			PPAir a_contact_line = cylinderLengthwiseLineInDirection(a.getCenter(), a.getHeightAxis(), a.getRadius(), a.getDrumHeight(), norm);
			PPAir b_contact_line = cylinderLengthwiseLineInDirection(b.getCenter(), b.getHeightAxis(), b.getRadius(), b.getHeight(), -norm);

			//taking advantage of the fact that cylinderLengthwiseLineInDirection() will return p1, p2 such that p1 - p2 is the direction of height axis.
			//ensure b.p1 - b.p2 is in the direction of a_height_axis
			if (b_height_axis.dot(a_height_axis) < 0) std::swap(b_contact_line.p1, b_contact_line.p2);

			//treat the two contacts as 1-dimensional line segments along a_height_axis and find the intersection
			double a_max_v = a_contact_line.p1.dot(a_height_axis);
			double a_min_v = a_contact_line.p2.dot(a_height_axis);
			double b_max_v = b_contact_line.p1.dot(a_height_axis);
			double b_min_v = b_contact_line.p2.dot(a_height_axis);

			mthz::Vec3 intersection_max = (a_max_v > b_max_v) ? b_contact_line.p1 : a_contact_line.p1;
			mthz::Vec3 intersection_min = (a_min_v > b_min_v) ? a_contact_line.p2 : b_contact_line.p2;

			manifold_pool = {
				ProjectedContactPoint{ mthz::NVec<2>{u.dot(intersection_max), w.dot(intersection_max)}, 0x0 },
				ProjectedContactPoint{ mthz::NVec<2>{u.dot(intersection_min), w.dot(intersection_min)}, 0x1 },
			};
		}
		else {
			manifold_pool = clipContacts(a_contact, b_contact);
		}

		mthz::Vec3 a_maxP = a.getHeightAxis().dot(norm) > 0 ?
			a_topcap_center + norm * a.getRadius()
			: a_botcap_center + norm * a.getRadius();

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b_mat.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	TransformedTriangle initTriangle(const StaticMeshGeometry& geom, const StaticMeshFace& og_triangle, bool transformation_required, mthz::Mat3  rot, mthz::Vec3 trans) {
		TransformedTriangle out;
		out.original_triangle_id = geom.getTriangleId(og_triangle.self_index);
		out.material = og_triangle.material;
		if (transformation_required) {
			out.normal = rot * og_triangle.normal;
			for (int i = 0; i < 3; i++) {
				out.vertices[i] = geom.get_transformed_vertex(og_triangle.vertex_indices[i], rot, trans, mthz::Vec3());
				out.edges[i] = geom.get_transformed_half_edge(og_triangle.half_edge_indices[i], rot, trans);
			}
		}
		else {
			for (int i = 0; i < 3; i++) {
				out.normal = og_triangle.normal;
				out.vertices[i] = geom.get_vertex(og_triangle.vertex_indices[i]);
				out.edges[i] = geom.get_half_edge(og_triangle.half_edge_indices[i]);
			}
		}
		return out;
	}

	
	static ExtremaInfo findTriangleExtrema(const TransformedTriangle& tri, mthz::Vec3 dir) {
		ExtremaInfo extrema;

		for (int i = 0; i < 3; i++) {
			const StaticMeshVertex& v = tri.vertices[i];
			double val = v.p.dot(dir);
			if (val < extrema.min_val) {
				extrema.min_pID = i;
				extrema.min_val = val;
			}
			if (val > extrema.max_val) {
				extrema.max_pID = i;
				extrema.max_val = val;
			}
		}

		return extrema;
	}

	static Manifold SAT_PolyTriangle(const Polyhedron& a, int a_id, const Material& a_mat, const TransformedTriangle& b) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& ag = a.getGaussMap();

		// backface culling
		if (b.normal.dot(a.interior_point - b.vertices[0].p) < 0) {
			out.max_pen_depth = -1;
			return out;
		}

		ExtremaInfo poly_info = findExtrema(a, -b.normal);
		CheckNormResults b_norm_x = sat_checknorm(poly_info, findTriangleExtrema(b, -b.normal), -b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		if (b_norm_x.pen_depth < min_pen.pen_depth) {
			min_pen = b_norm_x;
		}

		for (const GaussVert& g : ag.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, findTriangleExtrema(b, g.v), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		
		struct SimpleArc {
			mthz::Vec3 gv1, gv2;
		};

		std::vector<SimpleArc> triangle_arcs =
		{
			SimpleArc{b.normal, b.edges[0].out_direction}, SimpleArc{b.edges[0].out_direction, -b.normal},
			SimpleArc{b.normal, b.edges[1].out_direction}, SimpleArc{b.edges[1].out_direction, -b.normal},
			SimpleArc{b.normal, b.edges[2].out_direction}, SimpleArc{b.edges[2].out_direction, -b.normal},
		};

		for (const GaussArc& arc1 : ag.arcs) {
			for (SimpleArc sa : triangle_arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx].v;
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx].v;
				mthz::Vec3 b1 = -sa.gv1;
				mthz::Vec3 b2 = -sa.gv2;

				//check arcs arent on opposite hemispheres
				mthz::Vec3 a_avg = a1 + a2;
				if (a_avg.dot(b1) + a_avg.dot(b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if (a_avg.dot(n) < 0) {
					n *= -1;
				}

				ExtremaInfo poly_extrema = findExtrema(a, n);
				CheckNormResults x = sat_checknorm(poly_extrema, findTriangleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		ContactAreaOrigin triangle_closest_feature_type;
		int closest_feature_index;
		findTriangleContactFeature(b, min_pen.norm, min_pen.b_maxPID, &triangle_closest_feature_type, &closest_feature_index);

		if (triangle_closest_feature_type != FACE) {
			mthz::Vec3 snapped_norm;
			if (triangle_closest_feature_type == VERTEX) {
				const StaticMeshVertex& v = b.vertices[closest_feature_index];
				snapped_norm = v.valid_normal_gauss_map.size() > 0 ? -acceptOrSnapNormalAgainstVertexGaussMap(v, -min_pen.norm) : -b.normal;
			}
			else if (triangle_closest_feature_type == EDGE) {
				const StaticMeshHalfEdge& e = b.edges[closest_feature_index];
				snapped_norm = e.has_gauss_arc? -acceptOrSnapNormalAgainstEdgeGaussArc(e, -min_pen.norm) : -b.normal;
			}
			ExtremaInfo poly_extrema = findExtrema(a, snapped_norm);
			min_pen = sat_checknorm(poly_extrema, findTriangleExtrema(b, snapped_norm), snapped_norm);
		}

		//ContactArea b_contact = findTriangleContactArea(b, -min_pen.norm, b_maxP, min_pen.b_maxPID, u, w);

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getPoints()[min_pen.a_maxPID];
		//todo: make this not terrible
		mthz::Vec3 b_maxP = b.vertices[min_pen.b_maxPID].p;
		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		bool did_closest_feature_satisfy_gauss_map;
		ContactArea b_contact = findTriangleContactAreaAndCheckGaussMapSatisfied(b, -min_pen.norm, b_maxP, min_pen.b_maxPID, u, w, &did_closest_feature_satisfy_gauss_map);
		ContactArea a_contact = findContactArea(a, min_pen.norm, a_maxP, min_pen.a_maxPID, u, w);
		
		std::vector<ProjectedContactPoint> manifold_pool = clipContacts(a_contact, b_contact);

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.original_triangle_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b.material.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b.material.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b.material.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b.material.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	//unoptimizeed, but just doing a minimal refactor of this existing (also not particularly optimized) code to get it working
	static Manifold SAT_SphereTriangle(const Sphere& a, int a_id, const Material& a_mat, const TransformedTriangle& b) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		ContactAreaOrigin triangle_closest_feature_type;
		int closest_edge_index = -1; int closest_vertex_index = -1;
		// backface culling
		if (b.normal.dot(a.getCenter() - b.vertices[0].p) < 0) {
			out.max_pen_depth = -1;
			return out;
		}

		// check triangle norm
		ExtremaInfo sphere_extrema = getSphereExtrema(a, -b.normal);
		CheckNormResults b_norm_x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, -b.normal), -b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		if (b_norm_x.pen_depth < min_pen.pen_depth) {
			triangle_closest_feature_type = FACE;
			min_pen = b_norm_x;
		}

		// check against vertices
		for (int i = 0; i < 3; i++) {
			const StaticMeshVertex& v = b.vertices[i];

			mthz::Vec3 p = b.vertices[i].p;
			mthz::Vec3 n = (p - a.getCenter()).normalize();
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n);
			CheckNormResults x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}

			if (v.valid_normal_gauss_map.empty()) continue; // no valid collisions with this vertex
			if (x.pen_depth < min_pen.pen_depth) {
				triangle_closest_feature_type = VERTEX;
				closest_vertex_index = i;
				min_pen = x;
			}
		}

		//check against edges
		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p1 = b.vertices[i].p;
			mthz::Vec3 p2 = b.vertices[(i + 1) % 3].p;
			const StaticMeshHalfEdge& e = b.edges[i];

			mthz::Vec3 edge_dir = (p2 - p1).normalize();
			mthz::Vec3 sample = p1 - a.getCenter();
			mthz::Vec3 n = (sample - edge_dir * edge_dir.dot(sample)).normalize();
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n);
			CheckNormResults x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}

			if (!e.has_gauss_arc) { continue; } // cant collide against this edge
			if (x.pen_depth < min_pen.pen_depth) {
				triangle_closest_feature_type = EDGE;
				closest_edge_index = i;
				min_pen = x;
			}
		}

		// if our min_pen axis comes from a vertex or edge, snap it to a valid normal according to the gauss map
		if (triangle_closest_feature_type == VERTEX) {
			mthz::Vec3 n_snapped = -acceptOrSnapNormalAgainstVertexGaussMap(b.vertices[closest_vertex_index], -min_pen.norm);
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n_snapped);
			min_pen = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n_snapped), n_snapped);
		}
		else if (triangle_closest_feature_type == EDGE) {
			mthz::Vec3 n_snapped = -acceptOrSnapNormalAgainstEdgeGaussArc(b.edges[closest_edge_index], -min_pen.norm);
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n_snapped);
			min_pen = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n_snapped), n_snapped);
		}

		out.normal = min_pen.norm;

		ContactP cp;
		cp.pos = a.getCenter() + min_pen.norm * a.getRadius();
		cp.pen_depth = min_pen.pen_depth;
		cp.restitution = std::max<double>(a_mat.restitution, b.material.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b.material.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b.material.static_friction_coeff) / 2.0;
		cp.s1_cfm = a_mat.cfm;
		cp.s2_cfm = b.material.cfm;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.original_triangle_id) << 32);

		cp.magicID = MagicID{ cID, static_cast<uint64_t>(- 1)}; //not bothering with featureid

		out.points.push_back(cp);

		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold SAT_CapsuleTriangle(const Capsule& a, int a_id, const Material& a_mat, const TransformedTriangle& b) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		mthz::Vec3 a_height_axis = a.getHeightAxis();
		mthz::Vec3 a_topcap_center = a.getCenter() + a_height_axis * a.getDrumHeight() / 2.0;
		mthz::Vec3 a_botcap_center = a.getCenter() + a_height_axis * a.getDrumHeight() / 2.0;

		// checking triangle norm
		ExtremaInfo poly_info = getCapsuleExtrema(a, b.normal);
		CheckNormResults b_norm_x = sat_checknorm(poly_info, findTriangleExtrema(b, b.normal), b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		if (b_norm_x.pen_depth < min_pen.pen_depth) {
			min_pen = b_norm_x;
		}

		// check edge against the drum section
		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p1 = b.vertices[i].p;
			mthz::Vec3 p2 = b.vertices[(i + 1) % 3].p;
			const StaticMeshHalfEdge& e = b.edges[i];

			mthz::Vec3 edge_dir = p2 - p1;
			mthz::Vec3 dir = edge_dir.cross(a_height_axis);
			if (dir.mag() < 0.00000000001) continue;

			mthz::Vec3 dir_normed = dir.normalize();
			ExtremaInfo cyl_extrema = getCapsuleExtrema(a, dir_normed);
			CheckNormResults x = sat_checknorm(cyl_extrema, findTriangleExtrema(b, dir_normed), dir_normed);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}

			if (!e.has_gauss_arc) { continue; } // cant collide against this edge
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		//check vertex collisions against the body of the cylinder
		for (int i = 0; i < 3; i++) {
			const StaticMeshVertex& v = b.vertices[i];
			mthz::Vec3 p = v.p;
			mthz::Vec3 diff = p - a.getCenter();
			mthz::Vec3 n = (diff - a_height_axis * a_height_axis.dot(diff)).normalize();
			ExtremaInfo cyl_extrema = getCapsuleExtrema(a, n);
			CheckNormResults x = sat_checknorm(cyl_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}

			if (v.valid_normal_gauss_map.empty()) continue; // no valid collisions with this vertex
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		//check vertex collisions against the caps
		for (int i = 0; i < 3; i++) {
			const StaticMeshVertex& v = b.vertices[i];
			mthz::Vec3 p = v.p;
			// determine which cap this vertex could collide against
			double vh = (p - a.getCenter()).dot(a_height_axis);
			mthz::Vec3 collidable_cap_center;
			if (vh >= a.getDrumHeight() / 2.0)       { collidable_cap_center = a_topcap_center; }
			else if (vh <= -a.getDrumHeight() / 2.0) { collidable_cap_center = a_botcap_center; }
			else                                     { continue; } // vertex is inbetween the two caps, so can't collide against either

			mthz::Vec3 n = (p - collidable_cap_center).normalize();
			ExtremaInfo capsule_extrema = getCapsuleExtrema(a, n);
			CheckNormResults x = sat_checknorm(capsule_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}

			if (v.valid_normal_gauss_map.empty()) continue; // no valid collisions with this vertex
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		//edge against the caps
		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p1 = b.vertices[i].p;
			mthz::Vec3 p2 = b.vertices[(i + 1) % 3].p;
			const StaticMeshHalfEdge& e = b.edges[i];


			mthz::Vec3 edge_dir = (p2 - p1).normalize();

			// check whether the edge could 
			mthz::Vec3 n;
			mthz::Vec3 topcap_sample = p1 - a_topcap_center;
			mthz::Vec3 botcap_sample = p1 - a_botcap_center;
			if (mthz::Vec3 n_top = topcap_sample - edge_dir * edge_dir.dot(topcap_sample); n_top.dot(a_height_axis) <= 0)      { n = n_top; }
			else if (mthz::Vec3 n_bot = botcap_sample - edge_dir * edge_dir.dot(botcap_sample); n_bot.dot(a_height_axis) >= 0) { n = n_bot; }
			else                                                                                                               { continue; }

			n = n.normalize();
			ExtremaInfo capsule_extrema = getCapsuleExtrema(a, n);
			CheckNormResults x = sat_checknorm(capsule_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		
		ContactAreaOrigin triangle_closest_feature_type;
		int closest_feature_index;
		findTriangleContactFeature(b, min_pen.norm, min_pen.b_maxPID, &triangle_closest_feature_type, &closest_feature_index);

		// if our min_pen axis comes from a vertex or edge, snap it to a valid normal according to the gauss map
		if (triangle_closest_feature_type != FACE) {
			mthz::Vec3 snapped_norm;
			if (triangle_closest_feature_type == VERTEX) {
				const StaticMeshVertex& v = b.vertices[closest_feature_index];
				snapped_norm = v.valid_normal_gauss_map.size() > 0 ? -acceptOrSnapNormalAgainstVertexGaussMap(v, -min_pen.norm) : -b.normal;
			}
			else if (triangle_closest_feature_type == EDGE) {
				const StaticMeshHalfEdge& e = b.edges[closest_feature_index];
				snapped_norm = e.has_gauss_arc ? -acceptOrSnapNormalAgainstEdgeGaussArc(e, -min_pen.norm) : -b.normal;
			}
			ExtremaInfo poly_extrema = getCapsuleExtrema(a, snapped_norm);
			min_pen = sat_checknorm(poly_extrema, findTriangleExtrema(b, snapped_norm), snapped_norm);
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getHeightAxis().dot(norm) > 0 ?
			a_topcap_center + norm * a.getRadius()
			: a_botcap_center + norm * a.getRadius();
		mthz::Vec3 b_maxP = b.vertices[min_pen.b_maxPID].p;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findCapsuleContactArea(a, min_pen.norm, u, w);
		bool did_closest_feature_satisfy_gauss_map;
		ContactArea b_contact = findTriangleContactAreaAndCheckGaussMapSatisfied(b, -min_pen.norm, b_maxP, min_pen.b_maxPID, u, w, &did_closest_feature_satisfy_gauss_map);

		std::vector<ProjectedContactPoint> manifold_pool;
		if (a_contact.origin == EDGE) {
			manifold_pool = { ProjectedContactPoint{a_contact.ps[0], 0x0} };
		}
		else {
			manifold_pool = clipContacts(a_contact, b_contact);
		}

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.original_triangle_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b.material.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b.material.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b.material.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b.material.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold SAT_CylinderTriangle(const Cylinder& a, int a_id, const Material& a_mat, const TransformedTriangle& b) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		//check cylinder face axis
		mthz::Vec3 a_height_axis = a.getHeightAxis();
		{
			ExtremaInfo cyl_extrema = getCylinderExtrema(a, a_height_axis);
			CheckNormResults x = sat_checknorm(cyl_extrema, findTriangleExtrema(b, a_height_axis), a_height_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		ExtremaInfo poly_info = getCylinderExtrema(a, b.normal);
		CheckNormResults b_norm_x = sat_checknorm(poly_info, findTriangleExtrema(b, b.normal), b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		if (b_norm_x.pen_depth < min_pen.pen_depth) {
			min_pen = b_norm_x;
		}

		//check edge collisions against the round body of the cylinder
		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p1 = b.vertices[i].p;
			mthz::Vec3 p2 = b.vertices[(i + 1) % 3].p;
			const StaticMeshHalfEdge& e = b.edges[i];

			mthz::Vec3 edge_dir = p2 - p1;
			mthz::Vec3 dir = edge_dir.cross(a_height_axis);
			if (dir.mag() < 0.00000000001) continue;

			mthz::Vec3 dir_normed = dir.normalize();
			ExtremaInfo cyl_extrema = getCylinderExtrema(a, dir_normed);
			CheckNormResults x = sat_checknorm(cyl_extrema, findTriangleExtrema(b, dir_normed), dir_normed);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		//check vertex collisions against the body of the cylinder
		for (int i = 0; i < 3; i++) {
			const StaticMeshVertex& v = b.vertices[i];
			mthz::Vec3 p = v.p;
			mthz::Vec3 diff = p - a.getCenter();
			mthz::Vec3 n = (diff - a_height_axis * a_height_axis.dot(diff)).normalize();
			if (n == mthz::Vec3(0, 0, 0)) { continue; }
			ExtremaInfo cyl_extrema = getCylinderExtrema(a, n);
			CheckNormResults x = sat_checknorm(cyl_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}

			if (v.valid_normal_gauss_map.empty()) continue; // no valid collisions with this vertex
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		struct SimpleArc {
			mthz::Vec3 gv1, gv2;
		};

		std::vector<SimpleArc> triangle_arcs =
		{
			SimpleArc{b.normal, b.edges[0].out_direction}, SimpleArc{b.edges[0].out_direction, -b.normal},
			SimpleArc{b.normal, b.edges[1].out_direction}, SimpleArc{b.edges[1].out_direction, -b.normal},
			SimpleArc{b.normal, b.edges[2].out_direction}, SimpleArc{b.edges[2].out_direction, -b.normal},
		};

		for (const GaussArc& arc1 : a.getGuassArcs()) {
			for (SimpleArc sa : triangle_arcs) {

				mthz::Vec3 a1 = a.getGuassVerts()[arc1.v1_indx];
				mthz::Vec3 a2 = a.getGuassVerts()[arc1.v2_indx];
				mthz::Vec3 b1 = -sa.gv1;
				mthz::Vec3 b2 = -sa.gv2;

				//check arcs arent on opposite hemispheres
				mthz::Vec3 a_avg = a1 + a2;
				if (a_avg.dot(b1) + a_avg.dot(b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if (a_avg.dot(n) < 0) {
					n *= -1;
				}

				ExtremaInfo poly_extrema = getCylinderExtrema(a, n);
				CheckNormResults x = sat_checknorm(poly_extrema, findTriangleExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		ContactAreaOrigin triangle_closest_feature_type;
		int closest_feature_index;
		findTriangleContactFeature(b, min_pen.norm, min_pen.b_maxPID, &triangle_closest_feature_type, &closest_feature_index);

		// if our min_pen axis comes from a vertex or edge, snap it to a valid normal according to the gauss map
		if (triangle_closest_feature_type != FACE) {
			mthz::Vec3 snapped_norm;
			if (triangle_closest_feature_type == VERTEX) {
				const StaticMeshVertex& v = b.vertices[closest_feature_index];
				snapped_norm = v.valid_normal_gauss_map.size() > 0 ? -acceptOrSnapNormalAgainstVertexGaussMap(v, -min_pen.norm) : -b.normal;
			}
			else if (triangle_closest_feature_type == EDGE) {
				const StaticMeshHalfEdge& e = b.edges[closest_feature_index];
				snapped_norm = e.has_gauss_arc ? -acceptOrSnapNormalAgainstEdgeGaussArc(e, -min_pen.norm) : -b.normal;
			}
			ExtremaInfo poly_extrema = getCylinderExtrema(a, snapped_norm);
			min_pen = sat_checknorm(poly_extrema, findTriangleExtrema(b, snapped_norm), snapped_norm);
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a_height_axis.dot(norm) > 0 ?
			Cylinder::getExtremaOfDisk(a.getTopDiskCenter(), a_height_axis, a.getRadius(), norm)
			: Cylinder::getExtremaOfDisk(a.getBotDiskCenter(), a_height_axis, a.getRadius(), norm);
		mthz::Vec3 b_maxP = b.vertices[min_pen.b_maxPID].p;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findCylinderContactArea(a, min_pen.norm, u, w);
		bool did_closest_feature_satisfy_gauss_map;
		ContactArea b_contact = findTriangleContactAreaAndCheckGaussMapSatisfied(b, -min_pen.norm, b_maxP, min_pen.b_maxPID, u, w, &did_closest_feature_satisfy_gauss_map);

		std::vector<ProjectedContactPoint> manifold_pool;
		if (a_contact.origin == EDGE) {
			manifold_pool = { ProjectedContactPoint{a_contact.ps[0], 0x0}};
		}
		else {
			manifold_pool = clipContacts(a_contact, b_contact);
		}

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.original_triangle_id) << 32);

		out.points.reserve(manifold_pool.size());
		for (ProjectedContactPoint p : manifold_pool) {
			ContactP cp;
			cp.pos = u * p.pos.v[0] + w * p.pos.v[1] + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b.material.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b.material.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b.material.static_friction_coeff) / 2.0;
			cp.s1_cfm = a_mat.cfm;
			cp.s2_cfm = b.material.cfm;
			cp.magicID = MagicID{ cID, p.magic };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	// TODO: there is a lot of room for caching optimizations here. for kinematic objects the transformation to world coordinates of each vertex and edge can be calculated once and reused for an entire substep.
	//       whithin each check between a primitive and the mesh, the same edges, vertices, etc may be checked multiple times. the result of of checking that feature against the primitive can be cached and reused.
	static std::vector<Manifold> SAT_PolyMesh(const Polyhedron& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		mthz::Mat3 local_to_world_rot = b_world_orientation.getRotMatrix();

		std::vector<unsigned int> tri_candidates;
		bool local_transformation_required = b_world_position != mthz::Vec3(0,0,0) || b_world_orientation != mthz::Quaternion(1,0,0,0);
		if (!local_transformation_required) {
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(a_aabb);
		}
		else {
			//local basis transformed to world coordinates
			mthz::Vec3 u = local_to_world_rot * mthz::Vec3(1, 0, 0);
			mthz::Vec3 v = local_to_world_rot * mthz::Vec3(0, 1, 0);
			mthz::Vec3 w = local_to_world_rot * mthz::Vec3(0, 0, 1);

			AABB local_coord_aabb = AABB::conformNewBasis(a_aabb, u, v, w, b_world_position);
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(local_coord_aabb);
		}

		std::vector<Manifold> manifolds_out;

		for (unsigned int i : tri_candidates) {
			TransformedTriangle tri = initTriangle(b, b.getTriangles()[i], local_transformation_required, local_to_world_rot, b_world_position);

			Manifold m = SAT_PolyTriangle(a, a_id, a_mat, tri);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
				//m = SAT_PolyTriangle(a, a_id, a_mat, tri);
				manifolds_out.push_back(m);
			}
		}

		return manifolds_out;
	}

	static std::vector<Manifold> SAT_SphereMesh(const Sphere& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		mthz::Mat3 local_to_world_rot = b_world_orientation.getRotMatrix();

		std::vector<unsigned int> tri_candidates;
		bool local_transformation_required = b_world_position != mthz::Vec3() || b_world_orientation != mthz::Quaternion();
		if (!local_transformation_required) {
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(a_aabb);
		}
		else {
			//local basis transformed to world coordinates
			mthz::Vec3 u = local_to_world_rot * mthz::Vec3(1, 0, 0);
			mthz::Vec3 v = local_to_world_rot * mthz::Vec3(0, 1, 0);
			mthz::Vec3 w = local_to_world_rot * mthz::Vec3(0, 0, 1);

			AABB local_coord_aabb = AABB::conformNewBasis(a_aabb, u, v, w, b_world_position);
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(local_coord_aabb);
		}

		std::vector<Manifold> manifolds_out;

		for (unsigned int i : tri_candidates) {
			TransformedTriangle tri = initTriangle(b, b.getTriangles()[i], local_transformation_required, local_to_world_rot, b_world_position);
			Manifold m = SAT_SphereTriangle(a, a_id, a_mat, tri);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
				//m = SAT_SphereTriangle(a, a_id, a_mat, tri);
				manifolds_out.push_back(m);
			}
		}

		return manifolds_out;
	}

	static std::vector<Manifold> SAT_CapsuleMesh(const Capsule &a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		mthz::Mat3 local_to_world_rot = b_world_orientation.getRotMatrix();

		std::vector<unsigned int> tri_candidates;
		bool local_transformation_required = b_world_position != mthz::Vec3() || b_world_orientation != mthz::Quaternion();
		if (!local_transformation_required) {
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(a_aabb);
		}
		else {
			//local basis transformed to world coordinates
			mthz::Vec3 u = local_to_world_rot * mthz::Vec3(1, 0, 0);
			mthz::Vec3 v = local_to_world_rot * mthz::Vec3(0, 1, 0);
			mthz::Vec3 w = local_to_world_rot * mthz::Vec3(0, 0, 1);

			AABB local_coord_aabb = AABB::conformNewBasis(a_aabb, u, v, w, b_world_position);
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(local_coord_aabb);
		}

		std::vector<Manifold> manifolds_out;

		for (unsigned int i : tri_candidates) {
			TransformedTriangle tri = initTriangle(b, b.getTriangles()[i], local_transformation_required, local_to_world_rot, b_world_position);
			Manifold m = SAT_CapsuleTriangle(a, a_id, a_mat, tri);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
				//m = SAT_SphereTriangle(a, a_id, a_mat, tri);
				manifolds_out.push_back(m);
			}
		}
		return manifolds_out;
	}

	static std::vector<Manifold> SAT_CylinderMesh(const Cylinder& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		mthz::Mat3 local_to_world_rot = b_world_orientation.getRotMatrix();

		std::vector<unsigned int> tri_candidates;
		bool local_transformation_required = b_world_position != mthz::Vec3() || b_world_orientation != mthz::Quaternion();
		if (!local_transformation_required) {
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(a_aabb);
		}
		else {
			//local basis transformed to world coordinates
			mthz::Vec3 u = local_to_world_rot * mthz::Vec3(1, 0, 0);
			mthz::Vec3 v = local_to_world_rot * mthz::Vec3(0, 1, 0);
			mthz::Vec3 w = local_to_world_rot * mthz::Vec3(0, 0, 1);

			AABB local_coord_aabb = AABB::conformNewBasis(a_aabb, u, v, w, b_world_position);
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(local_coord_aabb);
		}

		std::vector<Manifold> manifolds_out;

		for (unsigned int i : tri_candidates) {
			TransformedTriangle tri = initTriangle(b, b.getTriangles()[i], local_transformation_required, local_to_world_rot, b_world_position);
			Manifold m = SAT_CylinderTriangle(a, a_id, a_mat, tri);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
				//m = SAT_SphereTriangle(a, a_id, a_mat, tri);
				manifolds_out.push_back(m);
			}
		}
		return manifolds_out;
	}

	Manifold merge_manifold(const Manifold& m1, const Manifold& m2) {
		Manifold out = { std::vector<ContactP>(m1.points.size() + m2.points.size()), mthz::Vec3(), std::max<double>(m1.max_pen_depth, m2.max_pen_depth), m1.surfaceid1, m1.surfaceid2};
		out.normal = (m1.normal + m2.normal).normalize();

		for (int i = 0; i < m1.points.size(); i++) {
			out.points[i] = m1.points[i];
		}
		uint32_t off = static_cast<uint32_t>(m1.points.size());
		for (int i = 0; i < m2.points.size(); i++) {
			out.points[i + off] = m2.points[i];
		}

		return out;
	}

	Manifold cull_manifold(const Manifold& m, int new_size) {
		if (new_size >= m.points.size()) {
			return m;
		}
		Manifold out = { std::vector<ContactP>(new_size), m.normal, 0, m.surfaceid1, m.surfaceid2 };
		std::vector<bool> p_available(m.points.size(), true);

		mthz::Vec3 u, w;
		m.normal.getPerpendicularBasis(&u, &w);

		for (int i = 0; i < new_size; i++) {
			double vu = cos(2 * M_PI * i / new_size);
			double vw = sin(2 * M_PI * i / new_size);
			mthz::Vec3 target_dir = u * vu + w * vw;

			ContactP max_p;
			int max_indx;
			double max_v = -std::numeric_limits<double>::infinity();
			for (int j = 0; j < m.points.size(); j++) {
				if (p_available[j]) {
					ContactP p = m.points[j];
					double val = p.pos.dot(target_dir);
					if (val > max_v) {
						max_v = val;
						max_p = p;
						max_indx = j;
					}
				}
			}
			out.points[i] = max_p;
			out.max_pen_depth = std::max<double>(out.max_pen_depth, max_p.pen_depth);
			p_available[max_indx] = false;
		}

		return out;
	}
}