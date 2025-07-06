## Physics2D

### About

Physics2D is a simple 2D physics simulator for rigid bodies. It tries to accurately simulate rigid bodies behavior and their interactions.
What started as a minimalist ball-to-ball collision solver is evolving towards a complete dynamical/mechanical systems simulator.
The goal would then be to be able to build articulated robotics systems and control them (Hybrid model (impulse + constraint/force based)). 
The engine is written in C++. A demo application is provided for testing purposes. The latter uses ImGui and ImPlot for the interface, as well as SDL2 for rendering.

### Features

- Semi-implicite Euler integration
- Discrete collision detection bw convex shapes
    - SAT, GJK
- Contact manifold calculation
    - SAT, EPA + clipping
- Mass-spring systems with
    - Phase plot
    - Can simulate distance constraints with an "infinte" stiffness

### Installation

#### Dependencies

- ImGui
- ImPlot
- SDL2
- SDL2_gfx

For the moment the simulator is tightly coupled to the demo application. In the future I want to separate them so to provide a real "library" that can be used in any application.
Until I make those changes, you will have to **Install SDL2 and SDL2_gfx**.

#### Build

A Cmake build system is provided:
```
mkdir build
cd build
cmake ..
```

However, if you wish to use g++, a plain Makefile is also provided, with some useful commands.
Just run `make` and you should be good.

### TODO

- More realistic shock/collision propagation
- More realistic angular collision responses (polygons tend to vibrate)
- More realistic friction/contact model (spurious non-zero velocities exist when bodies are supposedly at rest)
- Implement constraints (start with distance and pivot)
- Add a convex shape creation tool in the editor for rigid bodies creation
