#include <cstdio>
#include <cmath>
#include "./Math/Vec3.h"
#include "./Math/Quaternion.h"
#include "./Physics/RigidBody.h"
#include "./Physics/PhysicsEngine.h"

double randD(double min, double max) {
	return min + (max - min) * std::rand() / RAND_MAX;
}

int main() {

	//std::vector<ConvexPoly> geometry;

	//double x = 0, y = 0, z = 0;
	//double w = 10, h = 20, d = 30;

	/*geometry.push_back(getRect(x, y, z, w, h, d));

	Quaternion q1 = Quaternion(randD(0, 2 * 3.14159265358979323846), Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1)));
	Quaternion q2 = Quaternion(randD(0, 2 * 3.14159265358979323846), Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1)));
	Quaternion q3 = Quaternion(randD(0, 2 * 3.14159265358979323846), Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1)));
	Quaternion q = q3 * q2 * q1;

	geometry[0].rotate(q, Vec3(x, y, z) + Vec3(w, h, d) / 2.0);

	RigidBody r = RigidBody(geometry, 1);

	geometry[0].SAT(geometry[0]);*/

	phyz::PhysicsEngine p;
	p.foo();

	/*Quaternion q1 = Quaternion(3.141592/4.0, Vec3(0, 0, 1));
	Quaternion q2 = Quaternion(3.141592 / 4.0, Vec3(0, 1, 0));

	ConvexPoly c1 = getRect(0, 0, 0, 2, 2, 2);
	c1.rotate(q1, Vec3(1, 1, 1));
	ConvexPoly c2 = getRect(2, 0, 0, 2, 2, 2);
	c2.rotate(q2, Vec3(3, 1, 1));

	Manifold info = c1.SAT(c2, 5);
	int a = 3 + 5;*/

}