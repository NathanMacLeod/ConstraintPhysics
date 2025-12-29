View a Demo of the engine on YouTube here:

 [![Gameplay](https://img.youtube.com/vi/HDZ2AlW5eJw/0.jpg)](https://www.youtube.com/watch?v=8do5ZWbF5Mo&list=PLE_JtgAhcZbBVb4Qu4Nc9QsJwMhErxmXC)

# ConstraintPhysics

This is a 3d physics engine I have been working on, both for my own interest and to learn more about physics simulation.
Building is currently only via MSVC.

Broad phase collision detection is perormed via an AABB tree. Narrow phase collision detection is done by SAT using Gauss spheres to reduce the number of edge vs edge checks.
The supported collision primitives are spheres, cyllinders, and convex polyhedrons. Generic 3d meshes are supported for static and kinematic objects, and can be read in from an obj file.

The solver is PGS based, also supporting a block solver using LDL decomposition for block solving systems of holonomic constraints together.

The engine has its own multithreaded based job system, which is uses to parallize many of the tasks across multiple cores.
