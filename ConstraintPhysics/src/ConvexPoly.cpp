#include "ConvexPoly.h"

#include <limits>
#include <algorithm>
#include <cassert>

namespace phyz {

	static int next_id;
	
	ConvexPoly::ConvexPoly(const ConvexPoly& c)
		: points(c.points), interior_point(c.interior_point), id(next_id++), material(c.material)
	{
		for (int i = 0; i < c.surfaces.size(); i++) {
			surfaces.push_back(Surface(c.surfaces[i], this));
		}
		for (int i = 0; i < c.edges.size(); i++) {
			edges.push_back(Edge(c.edges[i], this));
		}
	}

	static std::vector<SurfaceDef> default_surface_defs(const std::vector<std::vector<int>>& surface_vertex_indices) {
		std::vector<SurfaceDef> out;
		out.reserve(surface_vertex_indices.size());
		for (const std::vector<int>& indices : surface_vertex_indices) {
			out.push_back(SurfaceDef{ indices, false });
		}
		return out;
	}

	ConvexPoly::ConvexPoly(const std::vector<mthz::Vec3>& points, const std::vector<std::vector<int>>& surface_vertex_indices, Material material)
		: ConvexPoly(points, default_surface_defs(surface_vertex_indices), material)
	{}

	ConvexPoly::ConvexPoly(const std::vector<mthz::Vec3>& points, const std::vector<SurfaceDef>& surface_vertex_indices, Material material)
		: points(points), id(next_id++), material(material)
	{
		assert(points.size() >= 4);

		//create interior point (inefficient but not performance critical)
		interior_point = mthz::Vec3(0, 0, 0);
		for (const mthz::Vec3& p : points) {
			interior_point += p;
		}
		interior_point /= points.size();

		//create surfaces
		int surfaceID = points.size();
		surfaces.reserve(surface_vertex_indices.size());
		for (const SurfaceDef& s : surface_vertex_indices) {
			#ifndef NDEBUG
			for (int i : s.surface_vertex_indices) {
				assert(i >= 0 && i < points.size());
			}
			#endif
			surfaces.push_back(Surface(s.surface_vertex_indices, this, interior_point, surfaceID++, s.internal_surface));
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
		}
	}

	AABB ConvexPoly::gen_AABB() const {
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

	ConvexPoly ConvexPoly::getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point) const {
		ConvexPoly copy(*this);

		mthz::Mat3 rotMat = q.getRotMatrix();
		for (int i = 0; i < points.size(); i++) {
			copy.points[i] = pivot_point + rotMat * (points[i] - pivot_point);
		}
		copy.interior_point = pivot_point + rotMat * (interior_point - pivot_point);

		return copy;
	}

	ConvexPoly ConvexPoly::getTranslated(mthz::Vec3 t) const {
		ConvexPoly copy(*this);

		for (int i = 0; i < points.size(); i++) {
			copy.points[i] += t;
		}
		copy.interior_point += t;

		return copy;
	}

	GaussMap ConvexPoly::computeGaussMap() const {
		GaussMap g;
		for (int i = 0; i < surfaces.size(); i++) {
			const Surface& s1 = surfaces[i];

			bool redundant = false;
			for (GaussVert& g : g.face_verts) {
				if (abs(s1.normal().dot(g.v)) > 0.999) {
					if (g.internal_face) {
						g.SAT_redundant = true; //in case where one face is internal (aka normal is not valid for collision resolution)
												//then the internal one should be the one marked redundant to make sure that normal is still tested
					}
					else {
						redundant = true;
					}
					break;
				}
			}
			g.face_verts.push_back(GaussVert{ s1.normal(), redundant, s1.internal_surface });

			for (int j = i + 1; j < surfaces.size(); j++) {
				const Surface& s2 = surfaces[j];

				for (int k = 0; k < s1.n_points(); k++) {
					for (int w = 0; w < s2.n_points(); w++) {
						mthz::Vec3 x1 = s1.getPointI(k);
						mthz::Vec3 x2 = s1.getPointI((k + 1) % s1.n_points());
						mthz::Vec3 y1 = s2.getPointI(w);
						mthz::Vec3 y2 = s2.getPointI((w + 1) % s2.n_points());

						//if si and sj share an edge, there exists an arc between v1 and v2 on the gauss map
						if ((x1 == y1 && x2 == y2) || (x1 == y2 && x2 == y1)) {
							g.arcs.push_back({ (unsigned int)i, (unsigned int)j });
						}
					}
				}
			}
		}
		return g;
	}

	Edge::Edge(int p1_indx, int p2_indx, ConvexPoly* poly) {
		this->poly = poly;
		this->p1_indx = p1_indx;
		this->p2_indx = p2_indx;
	}

	Edge::Edge(const Edge& e, ConvexPoly* poly) {
		this->poly = poly;
		p1_indx = e.p1_indx;
		p2_indx = e.p2_indx;
	}
	Edge::Edge() {
		p1_indx = 0;
		p2_indx = 0;
		poly = nullptr;
	}

	Surface::Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, mthz::Vec3 interior_point, int surfaceID, bool internal_surface)
		: point_indexes(point_indexes), poly(poly), surfaceID(surfaceID), internal_surface(internal_surface)
	{
		assert(point_indexes.size() >= 3);

		findNormalCalcInfo(poly->points[point_indexes[0]] - interior_point);
		setWindingAntiClockwise(normal());
	}

	Surface::Surface(const Surface& s, ConvexPoly* poly)
		: point_indexes(s.point_indexes), normalDirection(s.normalDirection), poly(poly), surfaceID(s.surfaceID),
		normal_calc_index(s.normal_calc_index), internal_surface(s.internal_surface)
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
		while (abs(v1.dot(v2)) > 0.99) {
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

}