#include "RigidBody.h"

ConvexPoly::ConvexPoly(const ConvexPoly& c) {
	points = c.points;
	for (int i = 0; i < c.surfaces.size(); i++) {
		surfaces.push_back(Surface(c.surfaces[i], this));
	}
}

void ConvexPoly::rotate(const Quaternion q, Vec3 pivot_point) {
	Mat3 rotMat = q.getRotMatrix();
	for (int i = 0; i < points.size(); i++) {
		//printf("point before: %f %f %f  ", points[i].x, points[i].y, points[i].z);
		points[i] = pivot_point + rotMat * (points[i] - pivot_point);
		//printf("point after: %f %f %f\n", points[i].x, points[i].y, points[i].z);
	}
}

ConvexPoly getRect(double x, double y, double z, double dx, double dy, double dz) {
	ConvexPoly out;
	
	out.points.push_back(Vec3(x, y, z)); //0
	out.points.push_back(Vec3(x + dx, y, z)); //1
	out.points.push_back(Vec3(x + dx, y + dy, z)); //2
	out.points.push_back(Vec3(x, y+dy, z)); //3

	out.points.push_back(Vec3(x, y, z + dz)); //4
	out.points.push_back(Vec3(x + dx, y, z + dz)); //5
	out.points.push_back(Vec3(x + dx, y + dy, z + dz)); //6
	out.points.push_back(Vec3(x, y + dy, z + dz)); //7

	Vec3 mid = Vec3(x + dx / 2.0, y + dy / 2.0, z + dz / 2.0);
	std::vector<int> indexes = std::vector<int>(4);

	indexes[0] = 0; indexes[1] = 3; indexes[2] = 2; indexes[3] = 1;
	out.surfaces.push_back(Surface(indexes, &out, mid));

	indexes[0] = 0; indexes[1] = 1; indexes[2] = 5; indexes[3] = 4;
	out.surfaces.push_back(Surface(indexes, &out, mid));

	indexes[0] = 1; indexes[1] = 2; indexes[2] = 6; indexes[3] = 5;
	out.surfaces.push_back(Surface(indexes, &out, mid));

	indexes[0] = 2; indexes[1] = 3; indexes[2] = 7; indexes[3] = 6;
	out.surfaces.push_back(Surface(indexes, &out, mid));

	indexes[0] = 3; indexes[1] = 0; indexes[2] = 4; indexes[3] = 7;
	out.surfaces.push_back(Surface(indexes, &out, mid));

	indexes[0] = 4; indexes[1] = 5; indexes[2] = 6; indexes[3] = 7;
	out.surfaces.push_back(Surface(indexes, &out, mid));

	return out;
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

//my implementation of 'Fast and Accurate Computation of Polyhedral Mass Properties' (Brian Miritch)
static void calculateMassProperties(const std::vector<ConvexPoly>& geometry, double density, Vec3* com, Mat3* tensor, double* mass) {
	*com = Vec3(0, 0, 0);
	*mass = 0;
	*tensor = Mat3(); //default zeroed
	for (const ConvexPoly g : geometry) {
		double vol = 0, vol_x = 0, vol_y = 0, vol_z = 0, vol_xy = 0, vol_yz = 0, vol_zx = 0, vol_x2 = 0, vol_y2 = 0, vol_z2 = 0;
		
		for (const Surface s : g.surfaces) {
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