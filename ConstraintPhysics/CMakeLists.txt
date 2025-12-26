cmake_minimum_required(VERSION 3.23)

add_library(phyz_physics STATIC
    src/CFM.cpp
    src/CollisionDetect.cpp
    src/ConstraintSolver.cpp
    src/ConvexPrimitive.cpp
    src/Geometry.cpp
    src/HACD.cpp
    src/HolonomicBlockSolver.cpp
    src/Octree.cpp
    src/PersistentConstraint.cpp
    src/PhysicsEngine.cpp
    src/RigidBody.cpp
)
target_include_directories(phyz_physics PUBLIC src)
target_link_libraries(phyz_physics PUBLIC mthz_library)

if (CMAKE_CXX_FLAGS)
    target_compile_options(phyz_physics PRIVATE ${CMAKE_CXX_FLAGS})
endif()