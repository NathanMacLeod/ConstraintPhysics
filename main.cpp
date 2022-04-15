#include <cstdio>
#include <cmath>
#include "./Math/Vec3.h"
#include "./Math/Quaternion.h"
#include "./Physics/RigidBody.h"

double randD(double min, double max) {
	return min + (max - min) * std::rand() / RAND_MAX;
}

int main() {

	/*for (int i = 0; i < 10000; i++) {
		Quaternion q = Quaternion(randD(0, 7), Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1)));
		Vec3 v = Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1));
		Vec3 boomerang = q.conjugate().getRotMatrix() * (q.getRotMatrix() * v);

		if ((v - boomerang).mag() > 0.000001) {
			printf("wtf dude >:(\n");
		}
	}*/

	std::vector<ConvexPoly> geometry;

	double x = -76, y = -125, z = 053;
	double w = 15, h = 32, d = 64;

	geometry.push_back(getRect(x, y, z, w, h, d));

	Quaternion q1 = Quaternion(3.14159265358979323846 / 4, Vec3(1, 54, 1));
	Quaternion q2 = Quaternion(3.14159265358979323846 / 4, Vec3(5, -1, -3));
	Quaternion q = q2* q1;

	geometry[0].rotate(q, Vec3(x, y, z) + Vec3(w, h, d) / 2.0);

	RigidBody r = RigidBody(geometry, 1);

	printf("com: %f, %f, %f\n", r.com.x, r.com.y, r.com.z);
	printf("mass: %f\n", r.mass);
	printf("tensor:\n");
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			printf("%f  ", r.tensor.v[i][j]);
		}
		printf("\n");
	}

	Mat3 norm = (q).conjugate().getRotMatrix() * r.tensor * (q).getRotMatrix();
	//geometry[0].rotate((q2 * q1).conjugate(), r.com);
	printf("\nRotation matrix: \n");
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			printf("%f  ", q.getRotMatrix().v[i][j]);
		}
		printf("\n");
	}
	printf("\ntensor normalized:\n");
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			printf("%f  ", norm.v[i][j]);
		}
		printf("\n");
	}

}