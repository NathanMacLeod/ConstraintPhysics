#pragma once

#include "../Math/Vec3.h"
#include "../Math/Quaternion.h"
#include "../Math/Mat3.h"
#include <Vector>

class Surface;

struct ConvexPoly {
	ConvexPoly() {}
	ConvexPoly(const ConvexPoly& c);

	void rotate(const Quaternion q, Vec3 pivot_point);

	std::vector<Vec3> points;
	std::vector<Surface> surfaces;
};

ConvexPoly getRect(double x, double y, double z, double dx, double dy, double dz);

class Surface {
public:
	Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, Vec3 interior_point);
	Surface(const Surface& s, ConvexPoly* poly);

	int n_points() const;
	Vec3 normal() const;
	Vec3 getPointI(int i) const;
private:
	ConvexPoly* poly;
	std::vector<int> point_indexes;
	int normalDirection;
};

class RigidBody {
public:
	RigidBody(const std::vector<ConvexPoly>& geometry, double density);

	Quaternion orientation;
	Vec3 com;
	Mat3 tensor;
	double mass;
	std::vector<ConvexPoly> geometry;
	std::vector<ConvexPoly> reference_geometry;
};