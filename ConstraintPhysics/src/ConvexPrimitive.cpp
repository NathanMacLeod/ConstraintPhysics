#include "ConvexPrimitive.h"

#include <limits>
#include <algorithm>
#include <cassert>

namespace phyz {

	static int next_id;
	
	ConvexPrimitive::ConvexPrimitive(const ConvexPrimitive& c)
		: material(c.material), type(c.type), id(next_id++)
	{
		switch (c.type) {
		case POLYHEDRON:
			geometry = (ConvexGeometry*) new Polyhedron((const Polyhedron&)*c.geometry);
			break;
		case SPHERE:
			geometry = (ConvexGeometry*)new Sphere((const Sphere&)*c.geometry);
			break;
		}
	}

	ConvexPrimitive::ConvexPrimitive(const ConvexGeometry& geometry_primitive, Material material) 
		: material(material), type(geometry_primitive.getType()), id(next_id++)
	{
		switch (geometry_primitive.getType()) {
		case POLYHEDRON:
			geometry = (ConvexGeometry*)new Polyhedron((const Polyhedron&)geometry_primitive);
			break;
		case SPHERE:
			geometry = (ConvexGeometry*)new Sphere((const Sphere&)geometry_primitive);
			break;
		}
	}

	ConvexPrimitive ConvexPrimitive::getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point) const {
		ConvexPrimitive copy(*this);
		switch (type) {
		case POLYHEDRON:
			*(Polyhedron*)copy.geometry = ((Polyhedron*)geometry)->getRotated(q, pivot_point);
			break;
		case SPHERE:
			*(Sphere*)copy.geometry = ((Sphere*)geometry)->getRotated(q, pivot_point);
			break;
		}
		
		return copy;
	};

	ConvexPrimitive ConvexPrimitive::getTranslated(mthz::Vec3 t) const {
		ConvexPrimitive copy(*this);
		switch (type) {
		case POLYHEDRON:
			*(Polyhedron*)copy.geometry = ((Polyhedron*)geometry)->getTranslated(t);
			break;
		case SPHERE:
			*(Sphere*)copy.geometry = ((Sphere*)geometry)->getTranslated(t);
			break;
		}

		return copy;
	};

	ConvexPrimitive ConvexPrimitive::getScaled(double d, mthz::Vec3 center_of_dialtion) const {
		ConvexPrimitive copy(*this);
		switch (type) {
		case POLYHEDRON:
			*(Polyhedron*)copy.geometry = ((Polyhedron*)geometry)->getScaled(d, center_of_dialtion);
			break;
		case SPHERE:
			*(Sphere*)copy.geometry = ((Sphere*)geometry)->getScaled(d, center_of_dialtion);
			break;
		}

		return copy;
	}

	ConvexPrimitive::RayHitInfo ConvexPrimitive::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const {
		switch (type) {
		case POLYHEDRON:	return ((Polyhedron*)geometry)->testRayIntersection(ray_origin, ray_dir);
		case SPHERE:		return ((Sphere*)geometry)->testRayIntersection(ray_origin, ray_dir);
		}
	}

	StaticMesh::StaticMesh(const StaticMesh& c) 
		: aabb_tree(0)
	{

	}

	StaticMesh::StaticMesh(const std::vector<mthz::Vec3>& points, const std::vector<TriIndices>& triangle_indices)
		: aabb_tree(0)
	{
		struct Edge {
			unsigned int p1_indx, p2_indx;
			unsigned int opposite_point_indx;

			bool isCompliment(Edge e) {
				return p1_indx == e.p2_indx && p2_indx == e.p1_indx;
			}
		};

		struct TriangleGraphNode {
			unsigned int p1_indx, p2_indx, p3_indx;
			int tri_neighbor_indices[3]; //neighbor1 shares p1 p2 edge, neighbor2 shared p2 p3 edge, neighbor3 shares p3 p1 edge
			Edge edges[3];
			mthz::Vec3 normal;
		};

		std::vector<TriangleGraphNode> neighbor_graph;
		neighbor_graph.reserve(triangle_indices.size());
		for (TriIndices t : triangle_indices) {
			mthz::Vec3 v1 = points[t.i2] - points[t.i1];
			mthz::Vec3 v2 = points[t.i3] - points[t.i1];
			mthz::Vec3 normal = v1.cross(v2).normalize();
			neighbor_graph.push_back(TriangleGraphNode{ t.i1, t.i2, t.i3, {-1, -1, -1}, {Edge{t.i1, t.i2, t.i3}, Edge{t.i2, t.i3, t.i1}, Edge{t.i3, t.i1, t.i2}}, normal});
		}

		//brute force determine all neighbors
		for (int i = 0; i < neighbor_graph.size(); i++) {
			for (int j = i + 1; j < neighbor_graph.size(); j++) {

				for (int k = 0; k < 3; k++) {
					for (int w = 0; w < 3; w++) {

						if (neighbor_graph[i].edges[k].isCompliment(neighbor_graph[j].edges[w])) {
							assert(neighbor_graph[i].tri_neighbor_indices[k] == -1 && neighbor_graph[j].tri_neighbor_indices[w] == -1);
							neighbor_graph[i].tri_neighbor_indices[k] = j;
							neighbor_graph[j].tri_neighbor_indices[w] = i;
						}

					}
				}

			}
		}

		//using the neighbor graph to compute all the finalized StaticMeshTri objects
		//neighbor info is needed to determine the gauss arcs for valid edge collisions.
		triangles.reserve(neighbor_graph.size());
		for (TriangleGraphNode t : neighbor_graph) {
			StaticMeshTri tri;
			tri.normal = t.normal;
			tri.p1 = points[t.p1_indx]; tri.p2 = points[t.p2_indx]; tri.p1 = points[t.p3_indx];
	
			unsigned int gauss_vert_indx = 1;
			std::vector<GaussVert> gauss_map_verticies = { GaussVert{t.normal} };
			std::vector<GaussArc> gauss_arcs;
			for (int i = 0; i < 3; i++) {
				if (t.tri_neighbor_indices[i] == -1) {
					//no neighbor on this edge, so need to expose the entire 180 edge. Gauss arc doesnt really work for 180 degree arc, so we split it into 2 90 arcs
					mthz::Vec3 edge_dir = points[t.edges[i].p2_indx] - points[t.edges[i].p1_indx];
					mthz::Vec3 edge_out_dir = edge_dir.cross(t.normal).normalize();

					gauss_map_verticies.push_back(GaussVert{ edge_out_dir });
					gauss_map_verticies.push_back(GaussVert{ -t.normal });
					gauss_arcs.push_back(GaussArc{ 0, gauss_vert_indx });
					gauss_arcs.push_back(GaussArc{ gauss_vert_indx, gauss_vert_indx + 1 });

					gauss_vert_indx += 2;
				}
				else {
					//neighbors version of the same edge
					Edge complimentary_edge;
					for (int j = 0; j < 3; j++) {
						Edge neighbor_edge = neighbor_graph[t.tri_neighbor_indices[i]].edges[j];
						if (t.edges[i].isCompliment(neighbor_edge)) {
							complimentary_edge = neighbor_edge;
						}
					}

					mthz::Vec3 this_opposite_tip = points[t.edges[i].opposite_point_indx];
					mthz::Vec3 neighbor_opposite_tip = points[complimentary_edge.opposite_point_indx];

					bool edge_concave = (neighbor_opposite_tip - this_opposite_tip).dot(t.normal) >= 0;
					if (edge_concave) continue; //Geometry can never collide with this edge

					mthz::Vec3 neighbor_normal = neighbor_graph[t.tri_neighbor_indices[i]].normal;
					gauss_map_verticies.push_back(GaussVert{ neighbor_normal });
					gauss_arcs.push_back(GaussArc{ 0, gauss_vert_indx });

					gauss_vert_indx++;
				}
			}
		}
	}

	StaticMesh getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point = mthz::Vec3(0, 0, 0)) const;

	StaticMesh getTranslated(mthz::Vec3 t) const;

	StaticMesh getScaled(double d, mthz::Vec3 center_of_dialtion) const;

	AABB gen_AABB() const override;

	ConvexPrimitive::RayHitInfo testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir);

	Sphere::Sphere(const Sphere& c) 
		: center(c.center), radius(c.radius)
	{}

	Sphere::Sphere(mthz::Vec3 center, double radius) 
		: center(center), radius(radius)
	{}

	Sphere Sphere::getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point) const {
		return Sphere(pivot_point + q.applyRotation(pivot_point - pivot_point), radius);
	}

	Sphere Sphere::getTranslated(mthz::Vec3 t) const {
		return Sphere(center + t, radius);
	}

	Sphere Sphere::getScaled(double d, mthz::Vec3 center_of_dialation) const {
		return Sphere((center - center_of_dialation) * d + center_of_dialation, radius * d);
	}

	void Sphere::recomputeFromReference(const ConvexGeometry& reference_geometry, const mthz::Mat3& rot, mthz::Vec3 trans) {
		assert(getType() == reference_geometry.getType());
		const Sphere& reference = (const Sphere&)reference_geometry;
		center = trans + rot * reference.center;
	}

	AABB Sphere::gen_AABB() const {
		mthz::Vec3 min = center - mthz::Vec3(radius, radius, radius);
		mthz::Vec3 max = center + mthz::Vec3(radius, radius, radius);
		return AABB{ min, max };
	}

	ConvexPrimitive::RayHitInfo Sphere::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
		assert(abs(1 - ray_dir.mag()) < 0.0001); //should be unit length

		mthz::Vec3 rel_org = ray_origin - center;
		//solve quadratic
		double a = ray_dir.magSqrd();
		double b = 2 * rel_org.dot(ray_dir);
		double c = rel_org.magSqrd() - radius * radius;

		double t;

		double radical = b * b - 4 * a * c;
		if (radical < 0) {
			return { false }; //ray doesnt intersect circle
		}
		else if (radical == 0) {
			t = -0.5 * b / a;

			if (t < 0) return { false }; //intersection occurs behind origin
		}
		else {
			double t1 = (-b + sqrt(radical)) / (2 * a);
			double t2 = (-b - sqrt(radical)) / (2 * a);

			if (t1 < 0 && t2 < 0) return { false }; //intersection occurs behind origin
			else if (t1 < 0) t = t2;
			else if (t2 < 0) t = t1;
			else t = std::min<double>(t1, t2);
		}
		
		return ConvexPrimitive::RayHitInfo{ true, ray_origin + t * ray_dir, t };
	}

	Polyhedron::Polyhedron(const Polyhedron& c)
		: gauss_map(c.gauss_map), points(c.points), interior_point(c.interior_point), adjacent_faces_to_vertex(c.adjacent_faces_to_vertex), adjacent_edges_to_vertex(c.adjacent_edges_to_vertex)
	{
		for (int i = 0; i < c.surfaces.size(); i++) {
			surfaces.push_back(Surface(c.surfaces[i], this));
		}
		for (int i = 0; i < c.edges.size(); i++) {
			edges.push_back(Edge(c.edges[i], this));
		}
	}

	Polyhedron::Polyhedron(const std::vector<mthz::Vec3>& points, const std::vector<std::vector<int>>& surface_vertex_indices)
		: points(points), adjacent_faces_to_vertex(points.size()), adjacent_edges_to_vertex(points.size())
	{
		assert(points.size() >= 4);

		//create interior point
		interior_point = mthz::Vec3(0, 0, 0);
		for (const mthz::Vec3& p : points) {
			interior_point += p;
		}
		interior_point /= points.size();

		//create surfaces
		int surfaceID = points.size();
		surfaces.reserve(surface_vertex_indices.size());
		for (int i = 0; i < surface_vertex_indices.size(); i++) {
			const std::vector<int>& s = surface_vertex_indices[i];
			for (int j : s) {
				assert(j >= 0 && j < points.size());
				adjacent_faces_to_vertex[j].push_back(i);
			}
			surfaces.push_back(Surface(s, this, interior_point, surfaceID++));
		}

		//create edges
		struct Pair {
			int p1;
			int p2;

			bool operator==(const Pair p) {
				return (p1 == p.p1 && p2 == p.p2) || (p2 == p.p1 && p1 == p.p2);
			}
		};
		std::vector<Pair> seen_pairs;
		for (const Surface& s : surfaces) {
			int n = s.point_indexes.size();
			for (int i = 0; i < n; i++) {
				Pair p = Pair{ s.point_indexes[i], s.point_indexes[(i + 1) % n] };

				bool seen = false;
				for (const Pair pair : seen_pairs) {
					if (p == pair) {
						seen = true;
						break;
					}
				}

				if (!seen) {
					seen_pairs.push_back(p);
				}
			}
		}
		edges = std::vector<Edge>(seen_pairs.size());
		for (int i = 0; i < seen_pairs.size(); i++) {
			Pair pair = seen_pairs[i];
			edges[i] = Edge(pair.p1, pair.p2, this);
			adjacent_edges_to_vertex[pair.p1].push_back(i);
			adjacent_edges_to_vertex[pair.p2].push_back(i);
		}

		gauss_map = computeGaussMap();
	}

	AABB Polyhedron::gen_AABB() const {
		const double inf = std::numeric_limits<double>::infinity();
		mthz::Vec3 min(inf, inf, inf);
		mthz::Vec3 max(-inf, -inf, -inf);

		for (const mthz::Vec3& p : points) {
			min.x = std::min<double>(min.x, p.x);
			min.y = std::min<double>(min.y, p.y);
			min.z = std::min<double>(min.z, p.z);

			max.x = std::max<double>(max.x, p.x);
			max.y = std::max<double>(max.y, p.y);
			max.z = std::max<double>(max.z, p.z);
		}

		return { min, max };
	}

	Polyhedron Polyhedron::getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point) const {
		Polyhedron copy(*this);

		mthz::Mat3 rotMat = q.getRotMatrix();
		for (int i = 0; i < points.size(); i++) {
			copy.points[i] = pivot_point + rotMat * (points[i] - pivot_point);
		}
		copy.interior_point = pivot_point + rotMat * (interior_point - pivot_point);

		for (int i = 0; i < gauss_map.face_verts.size(); i++) {
			copy.gauss_map.face_verts[i].v = rotMat * gauss_map.face_verts[i].v;
		}
		return copy;
	}

	Polyhedron Polyhedron::getTranslated(mthz::Vec3 t) const {
		Polyhedron copy(*this);

		for (int i = 0; i < points.size(); i++) {
			copy.points[i] += t;
		}
		copy.interior_point += t;

		return copy;
	}

	Polyhedron Polyhedron::getScaled(double d, mthz::Vec3 center_of_dialation) const {
		Polyhedron copy(*this);

		for (int i = 0; i < points.size(); i++) {
			copy.points[i] = (copy.points[i] - center_of_dialation) * d + center_of_dialation;
		}
		copy.interior_point = (copy.interior_point - center_of_dialation) * d + center_of_dialation;;

		return copy;
	}

	void Polyhedron::recomputeFromReference(const ConvexGeometry& reference_geometry, const mthz::Mat3& rot, mthz::Vec3 trans) {
		assert(getType() == reference_geometry.getType());
		const Polyhedron& reference = (const Polyhedron&) reference_geometry;
		assert(reference.points.size() == points.size());

		for (int i = 0; i < points.size(); i++) {
			points[i] = trans + rot * reference.points[i];
		}
		interior_point = trans + rot * reference.interior_point;

		for (int i = 0; i < gauss_map.face_verts.size(); i++) {
			gauss_map.face_verts[i].v = rot * reference.gauss_map.face_verts[i].v;
		}

	}

	ConvexPrimitive::RayHitInfo Polyhedron::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
		for (const Surface& s : surfaces) {
			mthz::Vec3 sp = s.getPointI(0);
			mthz::Vec3 n = s.normal();

			if (ray_dir.dot(n) > 0 || (ray_origin - sp).dot(n) < 0) continue; //disqualifies ray for being on wrong side or facing wrong direction

			double t = n.dot(sp - ray_origin) / n.dot(ray_dir);
			mthz::Vec3 intersection_point = ray_origin + t * ray_dir;
			for (int i = 0; i < s.n_points(); i++) {
				mthz::Vec3 edge_p1 = s.getPointI(i);
				mthz::Vec3 edge_p2 = s.getPointI(i + 1 == s.n_points() ? 0 : i + 1);

				mthz::Vec3 out_dir = (edge_p2 - edge_p1).cross(n);
				if (out_dir.dot(intersection_point - edge_p1) > 0) continue; //the intersection of the ray with the surfaces plane does not lie within the surface
			}

			return ConvexPrimitive::RayHitInfo{ true, intersection_point, t };
		}

		return { false };
	}

	Edge::Edge(int p1_indx, int p2_indx, Polyhedron* poly) {
		this->poly = poly;
		this->p1_indx = p1_indx;
		this->p2_indx = p2_indx;
	}

	Edge::Edge(const Edge& e, Polyhedron* poly) {
		this->poly = poly;
		p1_indx = e.p1_indx;
		p2_indx = e.p2_indx;
	}
	Edge::Edge() {
		p1_indx = 0;
		p2_indx = 0;
		poly = nullptr;
	}

	Surface::Surface(const std::vector<int>& point_indexes, Polyhedron* poly, mthz::Vec3 interior_point, int surfaceID)
		: point_indexes(point_indexes), poly(poly), surfaceID(surfaceID)
	{
		assert(point_indexes.size() >= 3);

		findNormalCalcInfo(poly->points[point_indexes[0]] - interior_point);
		setWindingAntiClockwise(normal());
	}

	Surface::Surface(const Surface& s, Polyhedron* poly)
		: point_indexes(s.point_indexes), normalDirection(s.normalDirection), poly(poly), surfaceID(s.surfaceID),
		normal_calc_index(s.normal_calc_index)
	{}

	Surface::Surface() {
		poly = nullptr;
	}

	void Surface::findNormalCalcInfo(const mthz::Vec3& normalish) {
		mthz::Vec3 p0 = poly->points[point_indexes[0]];
		mthz::Vec3 p1 = poly->points[point_indexes[1]];
		normal_calc_index = 2;
		mthz::Vec3 v1 = (p1 - p0).normalize();
		mthz::Vec3 v2 = (poly->points[point_indexes[normal_calc_index]] - p0).normalize();
		while (abs(v1.dot(v2)) > 0.99 && normal_calc_index + 1 < point_indexes.size()) {
			v2 = (poly->points[point_indexes[++normal_calc_index]] - p0).normalize();
			assert(normal_calc_index < point_indexes.size());
		}

		normalDirection = 1; //initial guess
		if (normalish.dot(normal()) < 0) {
			normalDirection = -1;
		}
	}

	mthz::Vec3 Surface::normal() const {
		mthz::Vec3 p1 = poly->points[point_indexes[0]];
		mthz::Vec3 p2 = poly->points[point_indexes[1]];
		mthz::Vec3 p3 = poly->points[point_indexes[normal_calc_index]];

		return (p2 - p1).cross(p3 - p2).normalize() * normalDirection;
	}

	int Surface::n_points() const {
		return point_indexes.size();
	}


	mthz::Vec3 Surface::getPointI(int i) const {
		assert(i >= 0 && i < point_indexes.size());
		return poly->points[point_indexes[i]];
	}

	void Surface::setWindingAntiClockwise(const mthz::Vec3& normal) {
		 //if winding is anti-clockwise to normal, greens theorem will yield a positive area
		mthz::Vec3 u, w;
		normal.getPerpendicularBasis(&u, &w);
		double area = 0;
		for (int i = 0; i < n_points(); i++) {
			int i1 = i;
			int i2 = (i + 1 == n_points()) ? 0 : i+1;

			double p1_u, p1_w, p2_u, p2_w;
			mthz::Vec3 p1 = getPointI(i1);
			mthz::Vec3 p2 = getPointI(i2);
			
			p1_u = p1.dot(u);
			p1_w = p1.dot(w);
			p2_u = p2.dot(u);
			p2_w = p2.dot(w);

			area += (p2_w - p1_w) * (p2_u + p1_u) / 2.0;
		}

		//reverse if winding was clockwise
		if (area < 0) {
			for (int i = 0; i < point_indexes.size() / 2; i++) {
				int j = point_indexes.size() - 1 - i;

				int tmp = point_indexes[j];
				point_indexes[j] = point_indexes[i];
				point_indexes[i] = tmp;
			}
			findNormalCalcInfo(normal);
 		}
	}

	GaussMap Polyhedron::computeGaussMap() const {
		GaussMap g;
		for (int i = 0; i < surfaces.size(); i++) {
			const Surface& s1 = surfaces[i];

			bool redundant = false;
			for (GaussVert& g : g.face_verts) {
				if (s1.normal().dot(g.v) < -0.999) {
					redundant = true;
					break;
				}
			}
			int reference_point_index = s1.point_indexes[0];
			g.face_verts.push_back(GaussVert{ s1.normal(), redundant, findExtrema(*this, s1.normal()), reference_point_index, s1.normal().dot(points[reference_point_index]) });

			for (int j = i + 1; j < surfaces.size(); j++) {
				const Surface& s2 = surfaces[j];

				for (int k = 0; k < s1.n_points(); k++) {
					for (int w = 0; w < s2.n_points(); w++) {
						int s1_indx1 = s1.point_indexes[k];
						int s1_indx2 = s1.point_indexes[(k + 1) % s1.n_points()];
						int s2_indx1 = s2.point_indexes[w];
						int s2_indx2 = s2.point_indexes[(w + 1) % s2.n_points()];

						//if si and sj share an edge, there exists an arc between v1 and v2 on the gauss map
						if ((s1_indx1 == s2_indx1 && s1_indx2 == s2_indx2) || (s1_indx1 == s2_indx2 && s1_indx2 == s2_indx1)) {
							g.arcs.push_back(GaussArc{ (unsigned int)i, (unsigned int)j });
							break;
						}
					}
				}
			}
		}
		return g;
	}
}