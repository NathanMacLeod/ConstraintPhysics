#pragma once
#include "Mesh.h"
#include "../../ConstraintPhysics/src/Geometry.h"
#include <string>
#include <map>

struct MeshColliderOutput {
	std::map<std::string, Mesh> meshes;
	std::map<std::string, phyz::ConvexUnionGeometry> colliders;
};

/*
Expects a specifically formatted obj, that will be read as multiple mesh's and colliders.
groups are identified obj by 'g {name}' lines
if {name} has the format 'col_{subname}', it is presumed to be a valid convex mesh and will be saved as a convex union geometry in colliders, stored with key {subname}
Otherwise it is saved as a Mesh in meshes under name {name}
*/
MeshColliderOutput readMeshAndColliders(std::string formatted_obj_path, double scale=1);
