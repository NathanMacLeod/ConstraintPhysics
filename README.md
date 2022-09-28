View a Demo of the engine on YouTube here:

 [![Gameplay](https://img.youtube.com/vi/HDZ2AlW5eJw/0.jpg)](https://www.youtube.com/watch?v=HDZ2AlW5eJw)

# ConstraintPhysics

This is a physics engine I started working on on about a year ago, although most the work happened starting last spring.  
All of the code was written by myself, with the exception of some of the classes in the rendering section,
which come from TheCherno's OpenGL series.

This engine does not borrow any code from the previous 'Simple Physics' engine I wrote. Compared to the previous engine, I tried to do more research
for established algorithms and methods used by real time physics engines.

The engine uses a small linear algebra algebra I wrote which has support for 3 dimension vectors/matrices as well as quaternions. Within the PGS (Projected Gauss-seidel) 
solver there are also arbitrary N-dimensional vector and matrix classes as they were needed there.

As collision-detection algorithms are much more well defined on convex shapes, the geometry of rigid bodies are defined as compounds of convex shapes. A few
primitive shapes are provided, as well as support for defining custom convex shapes via a set of vertices and vertex index lists for surfaces. Functions for transforming
and merging geometry allow for the creation of more complicated shapes. All geometry in the Demo video was created this way.

Caculating the center of mass and inertia tensor uses the same algorithm I used in the simple physics engine, which comes from the paper
"Fast and Accurate Computation of Polyhedral Mass Properties" by Brian Miritch, although I managed to get the code a bit cleaner than last time.

The broad phase collision detection uses an octree to partition all bodies to avoid an O(N^2) comparison of all bodies against each other. Bodies placed in the same octree node
check against each other with an AABB (axis-alligned bounding box) intersection test. Bodies with intersecting bounding boxes are sent to the narrow phase collision check.

Narrow phase collision detection uses SAT (separating axis theorem), which is optomized with gauss maps to reduce O(N^2) edge vs edge checks. If collision occurs,
a manifold (the contact surface between two colliding bodies) is calculated by clipping the colliding surface of both bodies against each other. Multiple manifolds 
can exist between two bodies (e.g. a table would have 4 manifolds against the ground, one for each leg). For improved performance manifolds with similar normals are merged
together, and are then culled to only 4 contact points.

Persistant constraits (Ball and socket joints, hinges, slider joints) and temporary constraints (contact and friction) are stored as edges connecting bodies
in a graph. The previous impulse solution from the last physics tick are also stored in the graph to 'warm start' the current tick in order to converge to the 
solution faster. BFS (breadth first search) queries are used identify islands on the constraint graph. For improved performance, if all bodies in an island 
satisfy sleep criteria (i.e. they have all stopped moving), they are placed into a sleep state which absolves them collision checks against each other as well as 
not sending their constraints into the solver.

Constraint islands are solved by the PGS (Projected Gauss-seidel) solver, which conceptually places the constraints of all bodies into one large matrix and iteratively
converges to the solution. The solver supports collision, friction, ball & socket, hinge, motor, and slider constraints. 

