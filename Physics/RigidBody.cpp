#include "RigidBody.h"
#include <limits>
#define M_PI 3.14159265358979323846
#include <algorithm>

ConvexPoly::ConvexPoly(const ConvexPoly& c) {
	points = c.points;
	for (int i = 0; i < c.surfaces.size(); i++) {
		surfaces.push_back(Surface(c.surfaces[i], this));
	}
	for (int i = 0; i < c.edges.size(); i++) {
		edges.push_back(Edge(c.edges[i], this));
	}
}

static struct Pair {
	int p1;
	int p2;

	bool operator==(const Pair p) {
		return (p1 == p.p1 && p2 == p.p2) || (p2 == p.p1 && p1 == p.p2);
	}
};

void ConvexPoly::compute_edges() {
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

void ConvexPoly::gen_interiorP() {
	Vec3 avg;
	for (const Vec3 p : points) {
		avg += p;
	}
	interior_point = avg / points.size();
}

void ConvexPoly::rotate(const Quaternion q, Vec3 pivot_point) {
	Mat3 rotMat = q.getRotMatrix();
	for (int i = 0; i < points.size(); i++) {
		points[i] = pivot_point + rotMat * (points[i] - pivot_point);
	}
	interior_point =  pivot_point + rotMat * (interior_point - pivot_point);
}

static struct ExtremaInfo {
	ExtremaInfo() {
		min_val = std::numeric_limits<double>::infinity();
		max_val = -std::numeric_limits<double>::infinity();
	}

	Vec3 min_p;
	Vec3 max_p;
	double min_val;
	double max_val;
};

static ExtremaInfo findExtrema(const ConvexPoly& c, Vec3 axis) {
	ExtremaInfo extrema;
	
	for (Vec3 p : c.points) {
		double val = p.dot(axis);
		if (val < extrema.min_val) {
			extrema.min_p = p;
			extrema.min_val = val;
		}
		if (val > extrema.max_val) {
			extrema.max_p = p;
			extrema.max_val = val;
		}
	}

	return extrema;
}

static struct ProjP {
	double u;
	double w;
};

//kinda brute forcey might redo later
static std::vector<ProjP> findContactArea(const ConvexPoly& c, Vec3 n, Vec3 p, Vec3 u, Vec3 w) {
	const int COS_TOL = 0.00015; //~1 degree
	const int SIN_TOL = 0.015; //~1 degree

	std::vector<ProjP> out;

	for (const Surface& s : c.surfaces) {
		if (1 - s.normal().dot(n) <= COS_TOL) {
			int n_points = s.n_points();
			out = std::vector<ProjP>(n_points);
			for (int i = 0; i < n_points; i++) {
				Vec3 v = s.getPointI(i);
				out[i] = ProjP{ v.dot(u), v.dot(w) };
			}
			return out;
		}
	}

	for (Edge e : c.edges) {
		Vec3 p1 = e.p1();
		Vec3 p2 = e.p2();
		double d = abs((e.p2() - e.p1()).normalize().dot(n));
		if ((e.p1() == p || e.p2() == p) && abs((e.p2() - e.p1()).normalize().dot(n)) <= SIN_TOL) {
			out = { ProjP{ e.p1().dot(u), e.p1().dot(w) }, ProjP{ e.p2().dot(u), e.p2().dot(w) } };
			return out;
		}
	}

	return out = { ProjP{ p.dot(u), p.dot(w) } };
}

static bool pInside(const std::vector<ProjP> poly, ProjP p) {
	int left = 0;
	int right = 0;

	for (int i = 0; i < poly.size(); i++) {
		ProjP v1 = poly[i];
		ProjP v2 = poly[(i + 1) % poly.size()];


		if (v1.w == p.w) {
			//c2 will be checked for this check next iteration
			if (v1.u > p.u) {
				right++;
			}
			else {
				left++;
			}
		}
		else if ((v1.w - p.w) * (v2.w - p.w) < 0) {
			double intr_u = v1.u + (p.w - v1.w) * (v2.u - v1.u) / (v2.w - v1.w);
			if (intr_u > p.u) {
				right++;
			}
			else {
				left++;
			}
		}
	}

	return (left % 2 != 0) && (right % 2 != 0);
}

static bool findIntersection(ProjP a1, ProjP a2, ProjP b1, ProjP b2, ProjP* out) {
	double ma = (a2.w - a1.w) / (a2.u - a1.u);
	double mb = (b2.w - b1.w) / (b2.u - b1.u);
	
	//edge cases
	if (a1.u == a2.u) {
		double du = a1.u - b1.u;
		double wIntr = b1.w + du * mb;
		if ((b1.u - a1.u) * (b2.u - a1.u) <= 0 && (a1.w - wIntr) * (a2.w - wIntr) <= 0) {
			*out = ProjP{ a1.u, b1.w + (a1.u - b1.u) * (b2.w - b1.w) / (b2.u - b1.u) };
			return true;
		}
		return false;
	}
	if (b1.u == b2.u) {
		double du = b1.u - a1.u;
		double wIntr = a1.w + du * ma;
		if ((a1.u - b1.u) * (a2.u - b1.u) <= 0 && (b1.w - wIntr) * (b2.w - wIntr) <= 0) {
			*out = ProjP{ b1.u, a1.w + (b1.u - a1.u) * (a2.w - a1.w) / (a2.u - a1.u) };
			return true;
		}
		return false;
	}

	double dw0 = b1.w + mb * (a1.u - b1.u) - a1.w;
	double du = dw0 / (ma - mb);
	*out = ProjP{ a1.u + du, a1.w + ma * du };

	//check intersection is in bounds
	if ((out->u - a1.u) * (out->u - a2.u) >= 0 || (out->u - b1.u) * (out->u - b2.u) >= 0) {
		return false;
	}
	return true;
}

/*static void sat_facecheck(const ConvexPoly& facer, const ConvexPoly& edger, Vec3* norm, Vec3* facer_maxP, Vec3* edger_maxP, double* pen_depth, bool* sepr_axis, bool* first_check) {
	for (const Surface& s : facer.surfaces) {
		Vec3 n = s.normal();
		ExtremaInfo e1 = findExtrema(facer, n);
		ExtremaInfo e2 = findExtrema(edger, n);

		double pen = e1.max_val - e2.min_val;
		if (pen < 0) {
			*sepr_axis = true;
			return;
		}

		if (*first_check || pen < *pen_depth) {
			*first_check = false;
			*facer_maxP = e2.min_p;
			*edger_maxP = e1.max_p;
			*norm = n;
			*pen_depth = pen;
		}

	}
}*/

static void sat_checknorm(const ConvexPoly& a, const ConvexPoly& b, Vec3 n, Vec3* norm_out, Vec3* a_maxP, Vec3* b_maxP, double* pen_depth, bool* sepr_axis, bool* first_check) {
	ExtremaInfo e1 = findExtrema(a, n);
	ExtremaInfo e2 = findExtrema(b, n);

	double pen = e1.max_val - e2.min_val;
	if (pen < 0) {
		*sepr_axis = true;
		return;
	}

	if (*first_check || pen < *pen_depth) {
		*first_check = false;
		*a_maxP = e1.max_p;
		*b_maxP = e2.min_p;
		*pen_depth = pen;
		*norm_out = n;
	}
}

Manifold ConvexPoly::SAT(const ConvexPoly& c, int max_man_size) const {
	Manifold out;
	bool sepr_axis = false; //assumed false at first
	bool first_check = true;
	Vec3 norm;
	Vec3 this_maxP, c_maxP;
	double pen_depth;

	for (const Surface& s : surfaces) {
		Vec3 n = s.normal();
		sat_checknorm(*this, c, n, &norm, &this_maxP, &c_maxP, &pen_depth, &sepr_axis, &first_check);
		if (sepr_axis) {
			return out;
		}
	}

	for (const Surface& s : c.surfaces) {
		Vec3 n = s.normal();
		sat_checknorm(*this, c, n, &norm, &this_maxP, &c_maxP, &pen_depth, &sepr_axis, &first_check);
		if (sepr_axis) {
			return out;
		}
	}

	for (const Edge e1 : edges) {
		for (const Edge e2 : c.edges) {

			Vec3 v1 = e1.p2() - e1.p1();
			Vec3 v2 = e2.p2() - e2.p1();

			Vec3 n = v1.cross(v2);
			if (n.magSqrd() == 0) {
				continue;
			}
			n = n.normalize();
			if ((e1.p1() - interior_point).dot(n) < 0) {
				n *= -1;
			}

			sat_checknorm(*this, c, n, &norm, &this_maxP, &c_maxP, &pen_depth, &sepr_axis, &first_check);
			if (sepr_axis) {
				return out;
			}
		}
	}

	out.pen_depth = pen_depth;
	out.normal = norm;

	//generate manifold
	//make arbitrary perp vector
	const Vec3 temp1 = Vec3(1, 0, 0), temp2 = Vec3(0, 1, 0);
	Vec3 u = (norm.dot(temp1) < norm.dot(temp2)) ? norm.cross(temp1).normalize() : norm.cross(temp2).normalize();
	Vec3 w = norm.cross(u);
	std::vector<ProjP> this_contact = findContactArea(*this, norm, this_maxP, u, w);
	std::vector<ProjP> c_contact = findContactArea(c, norm*(-1), c_maxP, u, w);

	std::vector<ProjP> man_pool;
	for (ProjP p : this_contact) {
		if (pInside(c_contact, p)) {
			man_pool.push_back(p);
		}
	}
	for (ProjP p : c_contact) {
		if (pInside(this_contact, p)) {
			man_pool.push_back(p);
		}
	}
	if (this_contact.size() >= 2 && c_contact.size() >= 2) {
		int itr_max_this = (this_contact.size() == 2) ? 1 : this_contact.size();
		int itr_max_c = (c_contact.size() == 2) ? 1 : c_contact.size();
		for (int i = 0; i < itr_max_this; i++) {
			ProjP a1 = this_contact[i];
			ProjP a2 = this_contact[(i + 1) % this_contact.size()];

			for (int j = 0; j < itr_max_c; j++) {
				ProjP b1 = c_contact[j];
				ProjP b2 = c_contact[(j + 1) % c_contact.size()];

				ProjP out;
				if (findIntersection(a1, a2, b1, b2, &out)) {
					man_pool.push_back(out);
				}
			}
		}
	}

	Vec3 n_offset = norm * this_maxP.dot(norm);


	if (man_pool.size() > max_man_size) {
		for (int i = 0; i < max_man_size; i++) {
			double vu = cos(2 * M_PI * i / max_man_size);
			double vw = sin(2 * M_PI * i / max_man_size);

			ProjP max;
			int max_i;
			double max_v = -std::numeric_limits<double>::infinity();
			for (int j = 0; j < man_pool.size(); j++) {
				ProjP p = man_pool[j];
				double val = vu * p.u + vw * p.w;
				if (val > max_v) {
					max_v = val;
					max = p;
					max_i = j;
				}
			}
			out.points.push_back(u*max.u + w*max.w + n_offset);
			man_pool.erase(man_pool.begin() + max_i); //delete to prevent same point from being selected twice
		}
	}
	else {
		for (ProjP p : man_pool) {
			out.points.push_back(u * p.u + w * p.w + n_offset);
		}
		
	}

	return out;
}

ConvexPoly getRect(double x, double y, double z, double dx, double dy, double dz) {
	ConvexPoly out;

	out.points = std::vector<Vec3>(8);
	out.surfaces = std::vector<Surface>(6);
	
	out.points[0] = (Vec3(x, y, z)); //0
	out.points[1] = (Vec3(x + dx, y, z)); //1
	out.points[2] = (Vec3(x + dx, y + dy, z)); //2
	out.points[3] = (Vec3(x, y+dy, z)); //3

	out.points[4] = (Vec3(x, y, z + dz)); //4
	out.points[5] = (Vec3(x + dx, y, z + dz)); //5
	out.points[6] = (Vec3(x + dx, y + dy, z + dz)); //6
	out.points[7] = (Vec3(x, y + dy, z + dz)); //7

	Vec3 mid = Vec3(x + dx / 2.0, y + dy / 2.0, z + dz / 2.0);
	std::vector<int> indexes = std::vector<int>(4);

	indexes[0] = 0; indexes[1] = 3; indexes[2] = 2; indexes[3] = 1;
	out.surfaces[0] = (Surface(indexes, &out, mid));

	indexes[0] = 0; indexes[1] = 1; indexes[2] = 5; indexes[3] = 4;
	out.surfaces[1] = (Surface(indexes, &out, mid));

	indexes[0] = 1; indexes[1] = 2; indexes[2] = 6; indexes[3] = 5;
	out.surfaces[2] = (Surface(indexes, &out, mid));

	indexes[0] = 2; indexes[1] = 3; indexes[2] = 7; indexes[3] = 6;
	out.surfaces[3] = (Surface(indexes, &out, mid));

	indexes[0] = 3; indexes[1] = 0; indexes[2] = 4; indexes[3] = 7;
	out.surfaces[4] = (Surface(indexes, &out, mid));

	indexes[0] = 4; indexes[1] = 5; indexes[2] = 6; indexes[3] = 7;
	out.surfaces[5] = (Surface(indexes, &out, mid));

	out.interior_point = mid;
	out.compute_edges();

	return out;
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

Vec3 Edge::p1() const {
	return poly->points[p1_indx];
}

Vec3 Edge::p2() const {
	return poly->points[p2_indx];
}

Surface::Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, Vec3 interior_point) {
	this->point_indexes = point_indexes;
	this->poly = poly;

	if (point_indexes.size() < 3) {
		printf("ERROR: Trying to initialize surface with fewer than 3 points\n");
	}

	normalDirection = 1; //initial guess
	Vec3 norm = normal();
	Vec3 outward = poly->points[point_indexes[0]] - interior_point;
	if (outward.dot(norm) < 0) {
		normalDirection = -1;
	}
}

Surface::Surface(const Surface& s, ConvexPoly* poly) {
	this->poly = poly;
	point_indexes = s.point_indexes;
	normalDirection = s.normalDirection;
}

Surface::Surface() {
	poly = nullptr;
}

Vec3 Surface::normal() const {
	Vec3 p1 = poly->points[point_indexes[0]];
	Vec3 p2 = poly->points[point_indexes[1]];
	Vec3 p3 = poly->points[point_indexes[2]];

	return (p2 - p1).cross(p3 - p2).normalize() * normalDirection;
}

int Surface::n_points() const {
	return point_indexes.size();
}


Vec3 Surface::getPointI(int i) const {
	if (i < 0 || i >= point_indexes.size()) {
		printf("ERROR: Trying to access invalid index of surface: %d, valid range [0, %d]\n", i, point_indexes.size());
	}

	return poly->points[point_indexes[i]];
}

static void calculateMassProperties(const std::vector<ConvexPoly>& geometry, double density, Vec3* com, Mat3* tensor, double* mass);

RigidBody::RigidBody(const std::vector<ConvexPoly>& geometry, double density) {
	calculateMassProperties(geometry, density, &this->com, &this->tensor, &this->mass);
}

void RigidBody::applyImpulse(Vec3 impulse, Vec3 position, Mat3* invTensor) {

	vel += impulse / mass;

	static Mat3 mat;
	if (invTensor == nullptr) {
		invTensor = &mat;
		*invTensor = (orientation.getRotMatrix() * tensor).inverse();
	}

	ang_vel += *invTensor * ((position - com).cross(impulse));

}

static struct IntrgVals {
	IntrgVals() {
		v = 0;
		v2 = 0;
		v2t = 0;
		v3 = 0;
	}

	double v;
	double v2;
	double v2t;
	double v3;
};

static enum Axis {X, Y, Z};
static enum MappedAxis {A, B, C};
static double getAxisVal(Vec3 v, MappedAxis axis, Axis proj_axis) {
	switch (proj_axis) {
	case X:
		switch (axis) {
		case A:
			return v.y;
		case B:
			return v.z;
		case C:
			return v.x;
		}
	case Y:
		switch (axis) {
		case A:
			return v.z;
		case B:
			return v.x;
		case C:
			return v.y;
		}

	case Z:
		switch (axis) {
		case A:
			return v.x;
		case B:
			return v.y;
		case C:
			return v.z;
		}
	}
}

//based off of 'Fast and Accurate Computation of Polyhedral Mass Properties' (Brian Miritch)
static void calculateMassProperties(const std::vector<ConvexPoly>& geometry, double density, Vec3* com, Mat3* tensor, double* mass) {
	*com = Vec3(0, 0, 0);
	*mass = 0;
	*tensor = Mat3(); //default zeroed
	for (const ConvexPoly& g : geometry) {
		double vol = 0, vol_x = 0, vol_y = 0, vol_z = 0, vol_xy = 0, vol_yz = 0, vol_zx = 0, vol_x2 = 0, vol_y2 = 0, vol_z2 = 0;
		
		for (const Surface& s : g.surfaces) {
			Vec3 n = s.normal();
			IntrgVals a, b, c;
			Axis proj_axis;
			
			//project onto most parralel plane
			if (abs(n.x) >= abs(n.y) && abs(n.x) >= abs(n.z)) {
				proj_axis = X;
			}
			else if (abs(n.y) >= abs(n.x) && abs(n.y) >= abs(n.z)) {
				proj_axis = Y;
			}
			else {
				proj_axis = Z;
			}

			//green's theorem
			double g1 = 0, ga = 0, gb = 0, gab = 0, ga2 = 0, gb2 = 0, ga2b = 0, gab2 = 0, ga3 = 0, gb3 = 0;
			for (int i = 0; i < s.n_points(); i++) {
				Vec3 v = s.getPointI(i);
				Vec3 v2 = s.getPointI((i + 1) % s.n_points());

				double va = getAxisVal(v, A, proj_axis);
				double vb = getAxisVal(v, B, proj_axis);
				double dA = getAxisVal(v2, A, proj_axis) - va;
				double dB = getAxisVal(v2, B, proj_axis) - vb;

				//o_o
				g1 += dB * (va + dA/2.0);
				ga += dB * (dA*dA/3.0 + va*dA + va*va) / 2.0;
				gb -= dA * (dB*dB/3.0 + vb*dB + vb*vb) / 2.0;
				gab -= dA * (va*vb*vb + dA*vb*vb/2.0 + va*vb*dB + (va*dB*dB + 2*vb*dA*dB)/3.0 + dA*dB*dB/4.0) / 2.0;
				ga2 += dB * (va*va*va + 3 *va*va*dA/2.0 + va*dA*dA + dA*dA*dA/4.0) / 3.0;
				gb2 -= dA * (vb*vb*vb + 3*vb*vb*dB/2.0 + vb*dB*dB + dB*dB*dB/4.0) / 3.0;
				ga2b -= dA * (va*va*vb*vb + va*va*vb*dB + va*vb*vb*dA + (va*va*dB*dB + vb*vb*dA*dA + 4*va*vb*dA*dB)/3.0
					+ (va*dA*dB*dB + vb*dB*dA*dA)/2.0 + dA*dA*dB*dB/4.0) / 2.0;
				gab2 += dB * (va*va*vb*vb + va*va*vb*dB + va*vb*vb*dA + (va*va*dB*dB + vb*vb*dA*dA + 4*va*vb*dA*dB)/3.0
					+ (va*dA*dB*dB + vb*dB*dA*dA)/2.0 + dA*dA*dB*dB/4.0) / 2.0;
				ga3 += dB * (va*va*va*va + 2*va*va*va*dA + 2*va*va*dA*dA + va*dA*dA*dA + dA*dA*dA*dA/5.0) / 4.0;
				gb3 -= dA * (vb*vb*vb*vb + 2*vb*vb*vb*dB + 2*vb*vb*dB*dB + vb*dB*dB*dB + dB*dB*dB*dB/5.0) / 4.0;
			}

			double na = getAxisVal(n, A, proj_axis);
			double nb = getAxisVal(n, B, proj_axis);
			double nc = getAxisVal(n, C, proj_axis);
			double nc_i = 1.0 / nc;

			Vec3 sample_point = s.getPointI(0);
			//constant from projecting surface integral onto the AB plane
			double k = nc * getAxisVal(sample_point, C, proj_axis) + nb * getAxisVal(sample_point, B, proj_axis) + na * getAxisVal(sample_point, A, proj_axis);

			//value propogation from projected surface integrals to the actual surface integrals
			a.v = nc_i * ga;
			b.v = nc_i * gb;
			c.v = nc_i*nc_i * (k*g1 - na*ga - nb*gb);
			a.v2 = nc_i * ga2;
			b.v2 = nc_i * gb2;
			c.v2 = nc_i*nc_i*nc_i * (k*k*g1 + na*na*ga2 + nb*nb*gb2 + 2*(na*nb*gab - k*na*ga - k*nb*gb));
			a.v2t = nc_i * ga2b;
			b.v2t = nc_i*nc_i * (k*gb2 - na*gab2 - nb*gb3);
			c.v2t = nc_i*nc_i*nc_i * (k*k*ga + na*na*ga3 + nb*nb*gab2 + 2*(na*nb*ga2b - k*na*ga2 - k*nb*gab));
			a.v3 = nc_i * ga3;
			b.v3 = nc_i * gb3;
			c.v3 = nc_i*nc_i*nc_i*nc_i * (k*k*k*g1 - na*na*na*ga3 - nb*nb*nb*gb3 - 3*k*k*na*ga + 3*k*na*na*ga2 - 3*na*na*nb*ga2b - 3*na*nb*nb*gab2
				+ 3*k*nb*nb*gb2 - 3*k*k*nb*gb + 6*k*na*nb*gab);

			//map back from a,b,c to x,y,z
			IntrgVals x, y, z;
			switch (proj_axis) {
			case X:
				x = c;
				y = a;
				z = b;
				break;
			case Y:
				x = b;
				y = c;
				z = a;
				break;
			case Z:
				x = a;
				y = b;
				z = c;
				break;
			}
			//value propogation from surface integral to volume integral (divergence theorem)
			vol += n.x * x.v;
			vol_x += n.x * x.v2 / 2.0;
			vol_y += n.y * y.v2 / 2.0;
			vol_z += n.z * z.v2 / 2.0;
			vol_xy += n.x * x.v2t / 2.0;
			vol_yz += n.y * y.v2t / 2.0;
			vol_zx += n.z * z.v2t / 2.0;
			vol_x2 += n.x * x.v3 / 3.0;
			vol_y2 += n.y * y.v3 / 3.0;
			vol_z2 += n.z * z.v3 / 3.0;
		}

		*mass += density * vol;
		*com += Vec3(vol_x, vol_y, vol_z);
		
		tensor->v[0][0] += vol_y2 + vol_z2;
		tensor->v[0][1] -= vol_xy;
		tensor->v[0][2] -= vol_zx;
		tensor->v[1][1] += vol_x2 + vol_z2;
		tensor->v[1][2] -= vol_yz;
		tensor->v[2][2] += vol_x2 + vol_y2;
	}

	*com /= *mass / density; // volume = mass/density
	*tensor *= density;

	//transposition of inertia tensor to be relative to CoM
	tensor->v[0][0] -= *mass * (com->y*com->y + com->z*com->z);
	tensor->v[0][1] += *mass * com->x * com->y;
	tensor->v[0][2] += *mass * com->z * com->x;
	tensor->v[1][1] -= *mass * (com->x*com->x + com->z*com->z);
	tensor->v[1][2] += *mass * com->y * com->z;
	tensor->v[2][2] -= *mass * (com->x*com->x + com->y*com->y);

	//intertia tensor is symmetric
	tensor->v[1][0] = tensor->v[0][1];
	tensor->v[2][0] = tensor->v[0][2];
	tensor->v[2][1] = tensor->v[1][2];

	
}