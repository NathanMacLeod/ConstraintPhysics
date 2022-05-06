#pragma once

#include "../Math/Vec3.h"
#include "../Math/Quaternion.h"
#include "../Math/Mat3.h"
#include <Vector>

class Surface;
class Edge;

struct Manifold {
	std::vector<Vec3> points;
	Vec3 normal;
	double pen_depth;
};

struct ConvexPoly {
	ConvexPoly(const ConvexPoly& c);
	ConvexPoly() {}

	Manifold SAT(const ConvexPoly& c, int max_man_size) const;
	void rotate(const Quaternion q, Vec3 pivot_point);
	void gen_interiorP();
	void compute_edges();
	
	std::vector<Vec3> points;
	std::vector<Surface> surfaces;
	std::vector<Edge> edges;
	Vec3 interior_point;

};

ConvexPoly getRect(double x, double y, double z, double dx, double dy, double dz);

class Edge {
public:
	Edge(int p1_indx, int p2_indx, ConvexPoly* poly);
	Edge(const Edge& e, ConvexPoly* poly);
	Edge();

	Vec3 p1() const;
	Vec3 p2() const;

	friend class ConvexPoly;
private:
	ConvexPoly* poly;
	int p1_indx;
	int p2_indx;
};

class Surface {
public:
	Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, Vec3 interior_point);
	Surface(const Surface& s, ConvexPoly* poly);
	Surface();

	int n_points() const;
	Vec3 normal() const;
	Vec3 getPointI(int i) const;

	friend class ConvexPoly;
private:
	ConvexPoly* poly;
	std::vector<int> point_indexes;
	int normalDirection;
};

class RigidBody {
public:
	RigidBody(const std::vector<ConvexPoly>& geometry, double density);
	void applyImpulse(Vec3 impulse, Vec3 position, Mat3* invTensor=nullptr);

	Quaternion orientation;
	Vec3 com;
	Vec3 vel;
	Vec3 ang_vel;

	Mat3 tensor;
	double mass;
	std::vector<ConvexPoly> geometry;
	std::vector<ConvexPoly> reference_geometry;
};