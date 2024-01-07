#include "CollisionDetect.h"
#include "ConvexPrimitive.h"
#include "Geometry.h"

#include <cassert>

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

	static Manifold SAT_PolyPoly(const Polyhedron& a, int a_id, const Material& a_mat, const Polyhedron& b, int b_id, const Material& b_mat);
	static Manifold detectSphereSphere(const Sphere& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat);
	static Manifold detectCylinderCylinder(const Cylinder& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat);
	static Manifold SAT_PolySphere(const Polyhedron& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat);
	static Manifold SAT_PolyCylinder(const Polyhedron& a, int a_id, const Material& a_mat, const Cylinder&b, int b_id, const Material& b_mat);
	static Manifold detectSphereCylinder(const Sphere& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat);
	static std::vector<Manifold> SAT_PolyMesh(const Polyhedron& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);
	static std::vector<Manifold> SAT_SphereMesh(const Sphere& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);
	static std::vector<Manifold> SAT_CylinderMesh(const Cylinder& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);

	Manifold detectCollision(const ConvexPrimitive& a, const ConvexPrimitive& b) {
		switch (a.getType()) {
		case POLYHEDRON:
			switch (b.getType()) {
			case POLYHEDRON:
				return SAT_PolyPoly((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Polyhedron&)*b.getGeometry(), b.getID(), b.material);
			case SPHERE:
				return SAT_PolySphere((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Sphere&)*b.getGeometry(), b.getID(), b.material);
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
			case CYLINDER:
				return detectSphereCylinder((const Sphere&)*a.getGeometry(), a.getID(), a.material, (const Cylinder&)*b.getGeometry(), b.getID(), b.material);
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
			case CYLINDER:
				return detectCylinderCylinder((const Cylinder&)*a.getGeometry(), a.getID(), a.material, (const Cylinder&)*b.getGeometry(), b.getID(), b.material);
			}
		}
		
	}

	std::vector<Manifold> detectCollision(const ConvexPrimitive& a, AABB a_aabb, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		switch (a.getType()) {
		case POLYHEDRON:
			return SAT_PolyMesh((const Polyhedron&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		case SPHERE:
			return SAT_SphereMesh((const Sphere&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		case CYLINDER:
			return SAT_CylinderMesh((const Cylinder&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		}
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

	static ContactArea findContactArea(const Polyhedron& c, mthz::Vec3 n, mthz::Vec3 p, int p_ID, mthz::Vec3 u, mthz::Vec3 w) {

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

	static inline ContactArea projectTriangleFace(const StaticMeshFace& s, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<mthz::NVec<2>>(3), std::vector<int>(3), s.id, FACE };

		for (int i = 0; i < 3; i++) {
			mthz::Vec3 v = s.vertices[i].p;
			out.ps[i] = mthz::NVec<2>{ v.dot(u), v.dot(w) };
			out.p_IDs[i] = i;
		}

		return out;
	}

	static inline ContactArea projectTriangleEdge(const StaticMeshEdge& e, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<mthz::NVec<2>>(), std::vector<int>(), -1, EDGE };

		out.ps = { mthz::NVec<2>{ e.p1.dot(u), e.p1.dot(w) }, mthz::NVec<2>{ e.p2.dot(u), e.p2.dot(w) } };
		out.p_IDs = { e.id, e.id };

		return out;
	}

	static ContactArea findTriangleContactArea(const StaticMeshFace& t, mthz::Vec3 n, mthz::Vec3 p, int p_ID, mthz::Vec3 u, mthz::Vec3 w) {

		double cos_ang = t.normal.dot(n);
		if (1 - cos_ang <= COS_TOL) {
			return projectTriangleFace(t, u, w);
		}

		for (int i = 0; i < 3; i++) {
			if (i != p_ID && (i + 1) % 3 != p_ID) {
				continue;
			}

			StaticMeshEdge e = t.edges[i];
			double sin_ang = abs((e.p2 - e.p1).normalize().dot(n));
			if (sin_ang <= SIN_TOL) {
				return projectTriangleEdge(e, n, p, u, w);
			}
		}

		return ContactArea{
			{ mthz::NVec<2>{ p.dot(u), p.dot(w) } },
			{ p_ID },
			-1,
			VERTEX
		};

	}

	static inline ContactArea projectCylinderFace(const std::vector<mthz::Vec3> face_verts, mthz::Vec3 u, mthz::Vec3 w, int face_id, int point_id_offset) {
		int n_points = face_verts.size();
		ContactArea out = { std::vector<mthz::NVec<2>>(n_points), std::vector<int>(n_points), face_id, FACE };

		for (int i = 0; i < n_points; i++) {
			out.ps[i] = { face_verts[i].dot(u), face_verts[i].dot(w) };
			out.p_IDs[i] = i + point_id_offset;
		}

		return out;
	}

	struct PPAir {
		mthz::Vec3 p1, p2;
	};

	PPAir cylinderLengthwiseLineInDirection(const Cylinder& c, mthz::Vec3 n) {
		mthz::Vec3 height_axis = c.getHeightAxis();
		double height = c.getHeight();
		mthz::Vec3 barrel_axis = (n - height_axis * height_axis.dot(n)).normalize();
		mthz::Vec3 p1 = c.getCenter() + barrel_axis * c.getRadius() + 0.5 * height * height_axis;
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
			PPAir line = cylinderLengthwiseLineInDirection(c, n);
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
		if (c.ps.size() > 3 && !isWindingCounterClockwise(c)) std::reverse(out.begin(), out.end());

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

	std::vector<ClipEvaluationPoint> getClipEvaluatinPolyAfterClippingEdge(const std::vector<ClipEvaluationPoint>& c, mthz::NVec<2> clip_maintain_side, mthz::NVec<2> clipping_edge_sample_point, int32_t clipping_edge_id, bool flip_magics) {
		std::vector<ClipEvaluationPoint> out;
		for (int i = 0; i < c.size(); i++) {
			ClipEvaluationPoint p1 = c[i];
			ClipEvaluationPoint p2 = c[(i + 1) % c.size()];

			double p1_v = p1.pos.dot(clip_maintain_side); //todo used cached value of previous p2_v
			double p2_v = p2.pos.dot(clip_maintain_side);
			double samp_v = clipping_edge_sample_point.dot(clip_maintain_side);

			if (p1_v >= samp_v && p2_v >= samp_v) {
				//both p1, p2 are not clipped case
				out.push_back(p1);
			}
			else if (p1_v == samp_v && p2_v < samp_v) {
				//p1 on the edge, p2_is clipped
				out.push_back(p1);
			}
			else if (p1_v > samp_v && p2_v < samp_v) {
				//intersection case where p1 is preserved
				out.push_back(p1);
				out.push_back(getEdgeIntersectionWithClippingEdge(p1, p2, clip_maintain_side, clipping_edge_sample_point, clipping_edge_id, flip_magics));
			}
			else if (p1_v < samp_v && p2_v > samp_v) {
				//intersection case where p1 is discarded
				out.push_back(getEdgeIntersectionWithClippingEdge(p1, p2, clip_maintain_side, clipping_edge_sample_point, clipping_edge_id, flip_magics));
			}
			//else case: both p1 and p2 are eliminated
		}

		return out;
	}

	std::vector<ClipEvaluationPoint> clipC1ByAllEdgesOfC2(std::vector<ClipEvaluationPoint> c1, const std::vector<ClipEvaluationPoint>& c2, bool flip_magics) {
		for (int i = 0; i < c2.size(); i++) {
			ClipEvaluationPoint e1 = c2[i];
			if (e1.edge_can_be_skipped_when_clipping) continue;

			ClipEvaluationPoint e2 = c2[(i + 1) % c2.size()];
			mthz::NVec<2> clip_dir = getInDirOfEdge(e1.pos, e2.pos);
			int32_t edge_id = getEdgeID(e1.source_id, e2.source_id);
			c1 = getClipEvaluatinPolyAfterClippingEdge(c1, clip_dir, e1.pos, edge_id, flip_magics);
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

	static Manifold SAT_PolyPoly(const Polyhedron& a, int a_id, const Material& a_mat, const Polyhedron& b, int b_id, const Material& b_mat) {
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
		ContactArea a_contact = findContactArea(a, norm, a_maxP, min_pen.a_maxPID, u, w);
		ContactArea b_contact = findContactArea(b, (-1) * norm, b_maxP, min_pen.b_maxPID, u, w);

		std::vector<ProjectedContactPoint> manifold_pool = clipContacts(a_contact, b_contact);
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


	static bool checkCylinderEdgeApproxVsBarrel(const Cylinder& a, const Cylinder& b, mthz::Vec3 barrel_height_axis, const std::vector<mthz::Vec3> edge_approx, CheckNormResults* out) {
		int n = edge_approx.size();
		for (int i = 0; i < n; i++) {
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
			PPAir a_contact_line = cylinderLengthwiseLineInDirection(a, norm);
			PPAir b_contact_line = cylinderLengthwiseLineInDirection(b, -norm);

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

	static Manifold SAT_PolySphere(const Polyhedron& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& gauss_map = a.getGaussMap();
		uint32_t a_feature_id;

		//very hacky using the fact that gauss verts have the same order as the corresponding surfaces they are made from. SurfaceID's start at points.size().
		int corresponding_surface_id = a.getPoints().size();
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

	static ExtremaInfo findTriangleExtrema(const StaticMeshFace& tri, mthz::Vec3 dir) {
		ExtremaInfo extrema;

		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p = tri.vertices[i].p;
			double val = p.dot(dir);
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

	//consider neighboring triangles when picking the axis of least penetration
	static bool normalDirectionValid(const StaticMeshFace& s, mthz::Vec3 normal) {
		//return true;
		double EPS = 0.0001;

		switch (s.concave_neighbor_count) {
		case 0:
		case 1:
			//the three points in counter-clockwise widning define a region on the surface of the sphere. the normal should lie in that surface to be valid
			assert(s.gauss_region.size() == 3);
			for (int i = 0; i < s.gauss_region.size(); i++) {
				mthz::Vec3 inner_region_direction = s.gauss_region[i].cross(s.gauss_region[(i + 1) % s.gauss_region.size()]);
				if (normal.dot(inner_region_direction) < -EPS) return false;
			}
			break;
		case 2:
		{
			//the normal should lie on the arc defined by the two points
			assert(s.gauss_region.size() == 2);
			mthz::Vec3 arc_normal = s.gauss_region[0].cross(s.gauss_region[1]);
			//check vector lies close to the plane
			if (abs(normal.dot(arc_normal)) > EPS) return false;
			mthz::Vec3 v0_up = arc_normal.cross(s.gauss_region[0]);

			//check vector doesnt lie outside the arc within the plane
			if (normal.dot(v0_up) < -EPS) return false;
			mthz::Vec3 v1_down = s.gauss_region[1].cross(arc_normal);
			if (normal.dot(v1_down) < -EPS) return false;
			break;
		}
		case 3:
			assert(s.gauss_region.size() == 1);
			if (normal.dot(s.normal) < 1 - EPS) return false; //s.normal is only valid direction
			break;
		}
		
		return true;
	}

	static Manifold SAT_PolyTriangle(const Polyhedron& a, int a_id, const Material& a_mat, const StaticMeshFace& b, double non_gauss_valid_penalty) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_gauss_valid_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& ag = a.getGaussMap();

		ExtremaInfo poly_info = findExtrema(a, b.normal);
		CheckNormResults b_norm_x = sat_checknorm(poly_info, findTriangleExtrema(b, b.normal), b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		if (b_norm_x.pen_depth < min_pen.pen_depth) { //no normalDirectionValid check needed for this direction
			min_pen = b_norm_x;
		}
		if (b_norm_x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -b_norm_x.norm)) {
			min_gauss_valid_pen = b_norm_x;
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
				if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
					min_gauss_valid_pen = x;
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
				if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
					min_gauss_valid_pen = x;
				}
			}
		}

		CheckNormResults nongauss_min_pen = min_pen;
		if (min_gauss_valid_pen.pen_depth < min_pen.pen_depth + non_gauss_valid_penalty) min_pen = min_gauss_valid_pen;

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getPoints()[nongauss_min_pen.a_maxPID];
		mthz::Vec3 b_maxP = b.vertices[nongauss_min_pen.b_maxPID].p;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findContactArea(a, nongauss_min_pen.norm, a_maxP, nongauss_min_pen.a_maxPID, u, w);
		ContactArea b_contact = findTriangleContactArea(b, (-1) * nongauss_min_pen.norm, b_maxP, nongauss_min_pen.b_maxPID, u, w);
		std::vector<ProjectedContactPoint> manifold_pool = clipContacts(a_contact, b_contact);

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.id) << 32);

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

	static Manifold SAT_SphereTriangle(const Sphere& a, int a_id, const Material& a_mat, const StaticMeshFace& b, double non_gauss_valid_penalty) {
		Manifold out;
		out.max_pen_depth = -1;
		uint32_t a_feature_id = -1;
		CheckNormResults min_gauss_valid_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		ExtremaInfo sphere_extrema = getSphereExtrema(a, b.normal);
		CheckNormResults b_norm_x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, b.normal), b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		if (b_norm_x.pen_depth < min_pen.pen_depth) {
			min_pen = b_norm_x;
		}
		if (b_norm_x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -b_norm_x.norm)) {
			min_gauss_valid_pen = b_norm_x;
		}

		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p = b.vertices[i].p;
			mthz::Vec3 n = (p - a.getCenter()).normalize();
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n);
			CheckNormResults x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = i;
			}
			if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
				min_gauss_valid_pen = x;
			}
		}
		for (StaticMeshEdge e : b.edges) {
			mthz::Vec3 edge_dir = (e.p2 - e.p1).normalize();
			mthz::Vec3 sample = e.p1 - a.getCenter();
			mthz::Vec3 n = (sample - edge_dir * edge_dir.dot(sample)).normalize();
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n);
			CheckNormResults x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = e.id;
			}
			if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
				min_gauss_valid_pen = x;
			}
		}

		CheckNormResults nongauss_min_pen = min_pen;
		if (min_gauss_valid_pen.pen_depth < min_pen.pen_depth + non_gauss_valid_penalty) min_pen = min_gauss_valid_pen;
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
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.id) << 32);

		cp.magicID = MagicID{ cID, a_feature_id }; //not bothering with featureid

		out.points.push_back(cp);

		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold SAT_CylinderTriangle(const Cylinder& a, int a_id, const Material& a_mat, const StaticMeshFace& b, double non_gauss_valid_penalty) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_gauss_valid_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
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
			if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
				min_gauss_valid_pen = x;
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
		if (b_norm_x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -b_norm_x.norm)) {
			min_gauss_valid_pen = b_norm_x;
		}

		//check edge collisions against the round body of the cylinder
		for (const StaticMeshEdge& e : b.edges) {
			mthz::Vec3 edge_dir = e.p2 - e.p1;
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
			if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
				min_gauss_valid_pen = x;
			}
		}

		//check vertex collisions against the body of the cylinder
		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p = b.vertices[i].p;
			mthz::Vec3 diff = p - a.getCenter();
			mthz::Vec3 n = (diff - a_height_axis * a_height_axis.dot(diff)).normalize();
			ExtremaInfo cyl_extrema = getCylinderExtrema(a, n);
			CheckNormResults x = sat_checknorm(cyl_extrema, findTriangleExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
			if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
				min_gauss_valid_pen = x;
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
				if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
					min_gauss_valid_pen = x;
				}
			}
		}

		CheckNormResults nongauss_min_pen = min_pen;
		if (min_gauss_valid_pen.pen_depth < min_pen.pen_depth + non_gauss_valid_penalty) min_pen = min_gauss_valid_pen;

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a_height_axis.dot(norm) > 0 ?
			Cylinder::getExtremaOfDisk(a.getTopDiskCenter(), a_height_axis, a.getRadius(), norm)
			: Cylinder::getExtremaOfDisk(a.getBotDiskCenter(), a_height_axis, a.getRadius(), norm);
		mthz::Vec3 b_maxP = b.vertices[nongauss_min_pen.b_maxPID].p;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findCylinderContactArea(a, nongauss_min_pen.norm, u, w);
		ContactArea b_contact = findTriangleContactArea(b, -nongauss_min_pen.norm, b_maxP, nongauss_min_pen.b_maxPID, u, w);

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
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.id) << 32);

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
			StaticMeshFace tri = local_transformation_required? b.getTriangles()[i].getTransformed(local_to_world_rot, b_world_position, mthz::Vec3()) : b.getTriangles()[i];
			double non_gauss_valid_normal_penalty = 0.1 * std::min<double>(AABB::longestDimension(a_aabb), AABB::longestDimension(tri.aabb)); //soft penalty to avoid internal collisions

			Manifold m = SAT_PolyTriangle(a, a_id, a_mat, tri, non_gauss_valid_normal_penalty);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
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
			const StaticMeshFace& tri = local_transformation_required? b.getTriangles()[i].getTransformed(local_to_world_rot, b_world_position, mthz::Vec3()) : b.getTriangles()[i];
			double non_gauss_valid_normal_penalty = 0.1 * std::min<double>(AABB::longestDimension(a_aabb), AABB::longestDimension(tri.aabb)); //soft penalty to avoid internal collisions

			Manifold m = SAT_SphereTriangle(a, a_id, a_mat, tri, non_gauss_valid_normal_penalty);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
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
			const StaticMeshFace& tri = local_transformation_required? b.getTriangles()[i].getTransformed(local_to_world_rot, b_world_position, mthz::Vec3()) : b.getTriangles()[i];
			double non_gauss_valid_normal_penalty = 0.1 * std::min<double>(AABB::longestDimension(a_aabb), AABB::longestDimension(tri.aabb)); //soft penalty to avoid internal collisions

			Manifold m = SAT_CylinderTriangle(a, a_id, a_mat, tri, non_gauss_valid_normal_penalty);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
				manifolds_out.push_back(m);
			}
		}

		return manifolds_out;
	}

	Manifold merge_manifold(const Manifold& m1, const Manifold& m2) {
		Manifold out = { std::vector<ContactP>(m1.points.size() + m2.points.size()), mthz::Vec3(), std::max<double>(m1.max_pen_depth, m2.max_pen_depth)};
		out.normal = (m1.normal + m2.normal).normalize();

		for (int i = 0; i < m1.points.size(); i++) {
			out.points[i] = m1.points[i];
		}
		int off = m1.points.size();
		for (int i = 0; i < m2.points.size(); i++) {
			out.points[i + off] = m2.points[i];
		}

		return out;
	}

	Manifold cull_manifold(const Manifold& m, int new_size) {
		if (new_size >= m.points.size()) {
			return m;
		}
		Manifold out = { std::vector<ContactP>(new_size), m.normal, 0 };
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