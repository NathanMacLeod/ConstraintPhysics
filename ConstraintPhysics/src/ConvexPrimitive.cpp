#include "ConvexPrimitive.h"
#include "CollisionDetect.h"

#include <map>
#include <limits>
#include <algorithm>
#include <cassert>

namespace phyz {

	static const double EPS = 0.0000000001;

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
		case CYLINDER:
			geometry = (ConvexGeometry*)new Cylinder((const Cylinder &) * c.geometry);
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
		case CYLINDER:
			geometry = (ConvexGeometry*)new Cylinder((const Cylinder&)geometry_primitive);
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
		case CYLINDER:
			*(Cylinder*)copy.geometry = ((Cylinder*)geometry)->getRotated(q, pivot_point);
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
		case CYLINDER:
			*(Cylinder*)copy.geometry = ((Cylinder*)geometry)->getTranslated(t);
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
		case CYLINDER:
			*(Cylinder*)copy.geometry = ((Cylinder*)geometry)->getScaled(d, center_of_dialtion);
			break;
		}

		return copy;
	}

	RayQueryReturn ConvexPrimitive::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const {
		switch (type) {
		case POLYHEDRON:	return ((Polyhedron*)geometry)->testRayIntersection(ray_origin, ray_dir);
		case SPHERE:		return ((Sphere*)geometry)->testRayIntersection(ray_origin, ray_dir);
		case CYLINDER:		return ((Cylinder*)geometry)->testRayIntersection(ray_origin, ray_dir);
		}

		assert(false);
		return {};
	}

	Cylinder::Cylinder(const Cylinder& c) 
		: center(c.center), height_axis(c.height_axis), radius(c.radius), height(c.height), gauss_verts(c.gauss_verts), gauss_arcs(c.gauss_arcs),
		top_face_approximation(c.top_face_approximation), bot_face_approximation(c.bot_face_approximation)
	{}

	Cylinder::Cylinder(mthz::Vec3 center, double radius, double height, mthz::Vec3 height_axis, uint32_t edge_approximation_detail, uint32_t face_approximation_detail)
		: center(center), height_axis(height_axis), radius(radius), height(height), gauss_verts(2 + edge_approximation_detail), gauss_arcs(2 * edge_approximation_detail),
		top_face_approximation(face_approximation_detail), bot_face_approximation(face_approximation_detail)
	{
		mthz::Mat3 rot;
		mthz::Vec3 unrotated_height_axis = mthz::Vec3(0, 1, 0);
		if (height_axis == unrotated_height_axis) {
			rot = mthz::Mat3::iden();
		}
		else {
			rot = mthz::Quaternion(acos(height_axis.dot(unrotated_height_axis)), unrotated_height_axis.cross(height_axis).normalize()).getRotMatrix();
		}

		gauss_verts[0] = rot * mthz::Vec3(0, 1, 0);
		gauss_verts[1] = rot * mthz::Vec3(0, -1, 0);

		{
			double dTheta = 2 * PI / edge_approximation_detail;
			for (unsigned int i = 0; i < edge_approximation_detail; i++) {
				double theta = i * dTheta;
				gauss_verts[i + 2] = rot * mthz::Vec3(sin(theta), 0, cos(theta));
				gauss_arcs[2 * i] = { 0, i }; //arc connecting top to side
				gauss_arcs[2 * i + 1] = { 1, i }; //arc connecting side to bottom.
			}
		}
		{
			double dTheta = 2 * PI / face_approximation_detail;
			double encapsulate_radius = radius / cos(PI / face_approximation_detail);
			for (unsigned int i = 0; i < face_approximation_detail; i++) {
				double theta = i * dTheta;
				top_face_approximation[i] = center + rot *  mthz::Vec3(encapsulate_radius * sin(theta), height/2.0, encapsulate_radius * cos(theta));
				bot_face_approximation[i] = center + rot * mthz::Vec3(encapsulate_radius * sin(theta),-height/2.0, encapsulate_radius * cos(theta));
			}
		}
	}

	Cylinder Cylinder::getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point) const {
		mthz::Vec3 new_center = pivot_point + q.applyRotation(center - pivot_point);
		mthz::Vec3 new_height_axis = q.applyRotation(height_axis);
		return Cylinder(new_center, radius, height, new_height_axis, static_cast<uint32_t>(gauss_verts.size()) - 2, static_cast<uint32_t>(top_face_approximation.size()));
	}

	Cylinder Cylinder::getTranslated(mthz::Vec3 t) const {
		return Cylinder(center + t, radius, height, height_axis, static_cast<uint32_t>(gauss_verts.size()) - 2, static_cast<uint32_t>(top_face_approximation.size()));
	}

	Cylinder Cylinder::getScaled(double d, mthz::Vec3 center_of_dialtion) const {
		return Cylinder(d * (center - center_of_dialtion) + center_of_dialtion, radius * d, height * d, height_axis, static_cast<uint32_t>(gauss_verts.size()) - 2, static_cast<uint32_t>(top_face_approximation.size()));
	}

	void Cylinder::recomputeFromReference(const ConvexGeometry& reference_geometry, const mthz::Mat3& rot, mthz::Vec3 trans) {
		assert(getType() == reference_geometry.getType());
		const Cylinder& reference = (const Cylinder&)reference_geometry;
		assert(reference.gauss_verts.size() == gauss_verts.size());

		center = rot * reference.center + trans;
		height_axis = rot * reference.height_axis;
		for (int i = 0; i < gauss_verts.size(); i++) {
			gauss_verts[i] = rot * reference.gauss_verts[i];
		}
		for (int i = 0; i < top_face_approximation.size(); i++) {
			top_face_approximation[i] = rot * reference.top_face_approximation[i] + trans;
			bot_face_approximation[i] = top_face_approximation[i] - height * height_axis;
		}
		
	}

	AABB Cylinder::gen_AABB() const {
		std::vector<mthz::Vec3> point_cloud;
		point_cloud.reserve(2 * top_face_approximation.size());
		for (int i = 0; i < top_face_approximation.size(); i++) {
			point_cloud.push_back(top_face_approximation[i]);
			point_cloud.push_back(bot_face_approximation[i]);
		}

		return AABB::encapsulatePointCloud(point_cloud);
	}

	RayQueryReturn checkDiskIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir, mthz::Vec3 disk_center, mthz::Vec3 disk_normal, double radius) {
		double t = disk_normal.dot(disk_center - ray_origin) / disk_normal.dot(ray_dir);
		mthz::Vec3 intersection_point = ray_origin + t * ray_dir;

		if (t < 0 || (intersection_point - disk_center).magSqrd() > radius * radius) return { false };
		return RayQueryReturn{ true, intersection_point,  disk_normal, t };
	}

	RayQueryReturn Cylinder::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
		RayQueryReturn out = { false };
		mthz::Vec3 height_axis = getHeightAxis();
		mthz::Vec3 u, v;
		height_axis.getPerpendicularBasis(&u, &v);

		double org_u = u.dot(ray_origin);
		double org_v = v.dot(ray_origin);
		double center_u = u.dot(center);
		double center_v = v.dot(center);
		double dir_u = u.dot(ray_dir);
		double dir_v = v.dot(ray_dir);

		bool ray_parralel_to_side = dir_u * dir_u + dir_v * dir_v < 0.000000001;
		if (!ray_parralel_to_side) {
			//check for intersection with the side. First treat like a circle and check for intersection, then verify it intersects at the correct height;
			//solve quadratic ax^2 + bx + c = 0;
			double a = dir_u * dir_u + dir_v * dir_v;
			double b = 2 * (dir_u * org_u + dir_v * org_v - dir_u * center_u - dir_v * center_v);
			double c = (org_u - center_u) * (org_u - center_u) + (org_v - center_v) * (org_v - center_v) - radius * radius;

			double radical = b * b - 4 * a * c;
			if (radical < 0) goto exit_side_test; //radical < 0 means no intersection occurs

			double t1 = (-b + sqrt(radical)) / (2 * a);
			double t2 = (-b - sqrt(radical)) / (2 * a);

			double intersect_dist;
			if (t1 < 0 && t2 < 0) goto exit_side_test; //intersection occurs behind origin
			else if (t1 < 0) intersect_dist = t2;
			else if (t2 < 0) intersect_dist = t1;
			else intersect_dist = std::min<double>(t1, t2);

			mthz::Vec3 intersect_point = ray_origin + intersect_dist * ray_dir;
			double intersect_height = intersect_point.dot(height_axis);
			if (std::abs(intersect_height - center.dot(height_axis)) < height / 2.0) {
				mthz::Vec3 norm = ((intersect_point - center) - height_axis * height_axis.dot(intersect_point - center)).normalize();
				out = RayQueryReturn{ true, intersect_point, norm, intersect_dist};
			}
		}
	exit_side_test:
		//check intersection with the caps
		RayQueryReturn top_intersection = checkDiskIntersection(ray_origin, ray_dir, center + height_axis * (height / 2.0), height_axis, radius);
		RayQueryReturn bot_intersection = checkDiskIntersection(ray_origin, ray_dir, center + height_axis * (height / 2.0), height_axis, radius);
		if (top_intersection.did_hit && (!out.did_hit || top_intersection.intersection_dist < out.intersection_dist)) out = top_intersection;
		if (bot_intersection.did_hit && (!out.did_hit || bot_intersection.intersection_dist < out.intersection_dist)) out = bot_intersection;

		return out;
	}

	mthz::Vec3 Cylinder::getExtremaOfDisk(mthz::Vec3 disk_center, mthz::Vec3 disk_normal, double radius, mthz::Vec3 target_direction) {
		//both the parralel and perpendicular cases would involve division by 0
		if (std::abs(disk_normal.dot(target_direction)) > 0.99999999) return disk_center;
		if (std::abs(disk_normal.dot(target_direction)) < 0.00000001) return disk_center + target_direction * radius;

		mthz::Vec3 u, v, w; //u, v, w equivalent of x, y, z in basis where target_direction is z axis
		w = target_direction; target_direction.getPerpendicularBasis(&u, &v);

		mthz::Vec3 uvw_center = mthz::Vec3(
			disk_center.dot(u),
			disk_center.dot(v),
			disk_center.dot(w)
		);

		mthz::Vec3 uvw_norm = mthz::Vec3(
			disk_normal.dot(u),
			disk_normal.dot(v),
			disk_normal.dot(w)
		);

		//partial derivatives given plane defined by the norm
		double dwdu = -uvw_norm.x / uvw_norm.z;
		double dwdv = -uvw_norm.y / uvw_norm.z; 

		double gradient_mag = sqrt(dwdu * dwdu + dwdv * dwdv);

		double gradient_u = dwdu / gradient_mag;
		double gradient_v = dwdv / gradient_mag;

		//du^2 + dv^2 
		double uv_mag_sqrd = radius * radius / (1 + gradient_mag * gradient_mag);
		double dw = sqrt(radius * radius - uv_mag_sqrd);
		double du = gradient_u * sqrt(uv_mag_sqrd);
		double dv = gradient_v * sqrt(uv_mag_sqrd);

		return disk_center + u * du + v * dv + w * dw;
	}

	Sphere::Sphere(const Sphere& c) 
		: center(c.center), radius(c.radius)
	{}

	Sphere::Sphere(mthz::Vec3 center, double radius) 
		: center(center), radius(radius)
	{}

	Sphere Sphere::getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point) const {
		return Sphere(pivot_point + q.applyRotation(center - pivot_point), radius);
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

	RayQueryReturn Sphere::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
		assert(std::abs(1 - ray_dir.mag()) < 0.0001); //should be unit length

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
		
		mthz::Vec3 hit_p = ray_origin + t * ray_dir;
		mthz::Vec3 norm = (hit_p - center).normalize();
		return RayQueryReturn{ true, hit_p, norm, t };
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

	Polyhedron Polyhedron::getPolyAfterFindMergedCoplanarFaces(const Polyhedron& p) {
		struct SurfaceGroup {
			mthz::Vec3 shared_norm;
			std::vector<Surface> surfaces;
		};

		std::vector<SurfaceGroup> groups;
		for (const Surface& s : p.getSurfaces()) {
			bool not_added_to_group = true;
			for (SurfaceGroup& g : groups) {
				if (1 - g.shared_norm.dot(s.normal()) < EPS) {
					g.surfaces.push_back(s);
					not_added_to_group = false;
					break;
				}
			}

			if (not_added_to_group) {
				groups.push_back(SurfaceGroup{ s.normal(), {s} });
			}
		}

		//some vertices might be deleted, so indices are not going to remain accurate.
		std::map<uint32_t, uint32_t> new_vertex_indices;
		std::vector<mthz::Vec3> remaining_vertices;
		std::vector<std::vector<uint32_t>> surface_vertex_indices;

		for (const SurfaceGroup& g : groups) {
			std::vector<uint32_t> extrema_point_indices;

			if (g.surfaces.size() == 1) extrema_point_indices = g.surfaces[0].point_indexes;
			else {
				//since p is convex, then all surfaces are also convex. Treat vertices as a point cloud, and use gift wrapping to generate the outer surface
				struct FaceCoordP {
					double u, v;
					uint32_t original_index;
				};
				std::vector<FaceCoordP> all_points;
				mthz::Vec3 u, v;
				g.shared_norm.getPerpendicularBasis(&u, &v);

				for (const Surface& s : g.surfaces) {
					for (uint32_t vert_indx : s.point_indexes) {
						mthz::Vec3 pos = p.getPoints()[vert_indx];
						all_points.push_back(FaceCoordP{ u.dot(pos), v.dot(pos), vert_indx });
					}
				}

				FaceCoordP min_u = all_points[0];
				for (FaceCoordP p : all_points) {
					if (std::abs(p.u - min_u.u) < EPS) min_u = p.v < min_u.v ? p : min_u;
					else if (p.u < min_u.u) min_u = p;
				}
				std::vector<FaceCoordP> hull_points = { min_u };
				double anglefrom_u = 0; double anglefrom_v = 1; //for calculating angle via dot product

				int max_itr = 50000;
				int itr = 0;
				while (itr++ < max_itr) {
					FaceCoordP prev_hullP = hull_points.back();

					//find the point that is has the leftmost angle to the prev point (see: https://en.wikipedia.org/wiki/Gift_wrapping_algorithm)
					FaceCoordP min_angle_p;
					double min_angle_dist = 0; //for tiebreaking colinear points
					double min_angle_angle = std::numeric_limits<double>::infinity();
					for (FaceCoordP p : all_points) {
						if (p.original_index == prev_hullP.original_index) continue;

						double rel_pos_u = p.u - prev_hullP.u;
						double rel_pos_v = p.v - prev_hullP.v;
						double dist = sqrt(rel_pos_u * rel_pos_u + rel_pos_v * rel_pos_v);
						double normed_dot_value = (anglefrom_u * rel_pos_u + anglefrom_v * rel_pos_v) / dist;
						double angle = acos(normed_dot_value);

						if (std::abs(min_angle_angle - angle) < EPS) {
							//points colinear- tiebreaker based on distance 
							if (dist > min_angle_dist) {
								min_angle_p = p;
								min_angle_dist = dist;
								min_angle_angle = angle;
							}
						}
						else if (angle < min_angle_angle) {
							min_angle_p = p;
							min_angle_dist = dist;
							min_angle_angle = angle;
						}
					}

					if (min_angle_p.original_index == hull_points[0].original_index) {
						break;
					}
					
					hull_points.push_back(min_angle_p);
					double rel_pos_u = min_angle_p.u - prev_hullP.u;
					double rel_pos_v = min_angle_p.v - prev_hullP.v;
					double dist = sqrt(rel_pos_u * rel_pos_u + rel_pos_v * rel_pos_v);
					anglefrom_u = rel_pos_u / dist; anglefrom_v = rel_pos_v / dist;
					
				}
				assert(itr != max_itr);
				

				for (FaceCoordP p : hull_points) extrema_point_indices.push_back(p.original_index);
			}

			std::vector<uint32_t> merged_surface_indices;

			for (uint32_t i : extrema_point_indices) {
				if (new_vertex_indices.find(i) == new_vertex_indices.end()) {
					new_vertex_indices[i] = static_cast<uint32_t>(remaining_vertices.size());
					remaining_vertices.push_back(p.getPoints()[i]);
				}
				merged_surface_indices.push_back(new_vertex_indices[i]);
			}

			surface_vertex_indices.push_back(merged_surface_indices);
		}

		return Polyhedron(remaining_vertices, surface_vertex_indices);
	}

	Polyhedron::Polyhedron(const std::vector<mthz::Vec3>& points, const std::vector<std::vector<uint32_t>>& surface_vertex_indices)
		: points(points), adjacent_faces_to_vertex(points.size()), adjacent_edges_to_vertex(points.size())
	{
		assert(points.size() >= 4);

		//create interior point
		interior_point = mthz::Vec3(0, 0, 0);
		for (const mthz::Vec3& p : points) {
			interior_point += p;
		}
		interior_point /= static_cast<double>(points.size());

		//create surfaces
		int32_t surfaceID = static_cast<int32_t>(points.size());
		surfaces.reserve(surface_vertex_indices.size());
		for (uint32_t i = 0; i < surface_vertex_indices.size(); i++) {
			const std::vector<uint32_t>& s = surface_vertex_indices[i];
			for (uint32_t j : s) {
				assert(j >= 0 && j < points.size());
				adjacent_faces_to_vertex[j].push_back(i);
			}
			surfaces.push_back(Surface(s, this, interior_point, surfaceID++));
		}

		//create edges
		struct Pair {
			uint32_t p1;
			uint32_t p2;

			bool operator==(const Pair p) {
				return (p1 == p.p1 && p2 == p.p2) || (p2 == p.p1 && p1 == p.p2);
			}
		};
		std::vector<Pair> seen_pairs;
		for (const Surface& s : surfaces) {
			uint32_t n = static_cast<uint32_t>(s.point_indexes.size());
			for (uint32_t i = 0; i < n; i++) {
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

#ifndef ndebug
		//confirm all surfaces are actually flat
		for (const phyz::Surface& s : getSurfaces()) {
			mthz::Vec3 n = s.normal();
			mthz::Vec3 ref_p = s.getPointI(0);
			double ref_val = ref_p.dot(n);
			for (uint32_t i = 1; i < s.n_points(); i++) {
				double error = std::abs(s.getPointI(i).dot(n) - ref_val);
				double dist = (s.getPointI(i) - ref_p).mag();
				double ang = atan(error / dist) * 180 / PI;
				assert(ang < 0.1);
			}
		}
#endif
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

	RayQueryReturn Polyhedron::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
		for (const Surface& s : surfaces) {
			mthz::Vec3 sp = s.getPointI(0);
			mthz::Vec3 n = s.normal();
			double eps = 0.0001;

			if (ray_dir.dot(n) > -eps || (ray_origin - sp).dot(n) < 0) continue; //disqualifies ray for being on wrong side or facing wrong direction

			double t = n.dot(sp - ray_origin) / n.dot(ray_dir);
			mthz::Vec3 intersection_point = ray_origin + t * ray_dir;
			for (uint32_t i = 0; i < s.n_points(); i++) {
				mthz::Vec3 edge_p1 = s.getPointI(i);
				mthz::Vec3 edge_p2 = s.getPointI(i + 1 == s.n_points() ? 0 : i + 1);

				mthz::Vec3 out_dir = (edge_p2 - edge_p1).cross(n);
				if (out_dir.dot(intersection_point - edge_p1) > 0) goto next_surface; //the intersection of the ray with the surfaces plane does not lie within the surface
			}

			return RayQueryReturn{ true, intersection_point, n, t };

			next_surface: continue;
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

	mthz::Vec3 Edge::p1() const { return poly->points[p1_indx]; }
	mthz::Vec3 Edge::p2() const { return poly->points[p2_indx]; }

	Surface::Surface(const std::vector<uint32_t>& point_indexes, Polyhedron* poly, mthz::Vec3 interior_point, int32_t surfaceID)
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
		while (std::abs(v1.dot(v2)) > 0.99 && normal_calc_index + 1 < point_indexes.size()) {
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

	uint32_t Surface::n_points() const {
		return static_cast<uint32_t>(point_indexes.size());
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
		for (uint32_t i = 0; i < n_points(); i++) {
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
			for (uint32_t i = 0; i < static_cast<uint32_t>(point_indexes.size()) / 2; i++) {
				uint32_t j = static_cast<uint32_t>(point_indexes.size()) - 1 - i;

				uint32_t tmp = point_indexes[j];
				point_indexes[j] = point_indexes[i];
				point_indexes[i] = tmp;
			}
			findNormalCalcInfo(normal);
 		}
	}

	GaussMap Polyhedron::computeGaussMap() const {
		GaussMap g;
		for (uint32_t i = 0; i < surfaces.size(); i++) {
			const Surface& s1 = surfaces[i];

			bool redundant = false;
			for (GaussVert& g : g.face_verts) {
				if (s1.normal().dot(g.v) < -1 + EPS) {
					redundant = true;
					break;
				}
			}
			uint32_t reference_point_index = s1.point_indexes[0];
			g.face_verts.push_back(GaussVert{ s1.normal(), redundant, findExtrema(*this, s1.normal()), reference_point_index, s1.normal().dot(points[reference_point_index]) });

			for (uint32_t j = i + 1; j < surfaces.size(); j++) {
				const Surface& s2 = surfaces[j];

				for (uint32_t k = 0; k < s1.n_points(); k++) {
					for (uint32_t w = 0; w < s2.n_points(); w++) {
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