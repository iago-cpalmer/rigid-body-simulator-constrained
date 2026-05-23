# Rigid Body Simulator

A small 3D rigid body simulator in C++20 that compares three ways of keeping bodies joined together: explicit penalty springs, implicit penalty springs and hard velocity constraints. You can switch between them while the simulation runs and watch how each one behaves on the same scene.

The scene is a chain of two 2 × 2 × 6 boxes, each with a mass of 1. The first box is pinned to a fixed point in the world and the second hangs from the first by a point joint. Both fall under gravity from rest.

## Requirements

- CMake 3.20 or newer
- A C++20 compiler with `<format>` support (the project has been built with Visual Studio 2022)
- Git, which CMake uses to download the dependencies
- A GPU and driver that support OpenGL 3.3

CMake fetches [Eigen](https://gitlab.com/libeigen/eigen) 3.4.0 and [GLFW](https://github.com/glfw/glfw) 3.4 during configuration, so there is nothing else to install.

## Build and run

```bash
cmake -S . -B build
```

```bash
cmake --build build --config Release
```

With Visual Studio, the program is written to `build/Release`:

```bash
build/Release/rigidbody_sim.exe
```

Single-configuration generators such as Ninja and Makefiles ignore `--config`. For those, pass `-DCMAKE_BUILD_TYPE=Release` when configuring and look for the program directly in `build/`.

## Controls

| Input | Action |
| --- | --- |
| `1` | Use the symplectic integrator |
| `2` | Use the implicit integrator |
| `3` | Use the symplectic integrator with constraints (default) |
| `P` | Pause or resume |
| `R` | Reset the scene, keeping the current integrator |
| Left mouse drag | Orbit the camera |
| Scroll wheel | Zoom |
| `Esc` | Quit |

The window title shows the active integrator and whether the simulation is paused.

## Integrators

The simulation advances at a fixed time step of `h = 0.005` s. In the formulas below, `v` holds the linear and angular velocity of every body, `M` is the mass matrix and `f` is the total force. Body forces include gravity, damping and the gyroscopic torque.

**Symplectic (`1`)** treats each joint as a spring with stiffness 100 and uses semi-implicit Euler:

```text
v' = v + h M⁻¹ (f_body + f_spring)
x' = x + h v'
```

**Implicit (`2`)** uses the same springs, but solves a linearised backward Euler system. This requires the Jacobians of the force with respect to position and velocity, including the geometric stiffness term that appears because a joint's lever arm rotates with its body:

```text
(M - h² ∂f/∂x - h ∂f/∂v) Δv = h (f + h ∂f/∂x v)
```

**Symplectic with constraints (`3`)** replaces the springs with hard point constraints. It predicts the velocity from the body forces, then solves for the impulses that cancel the velocity along the constraints. Baumgarte stabilisation (`β = 0.1`) corrects position drift, and a small regularisation term (`ε = 1e-6`) keeps the system well conditioned:

```text
v* = v + h M⁻¹ f_body
(J M⁻¹ Jᵀ + ε I) λ = J v* + (β / h) C
v' = v* - M⁻¹ Jᵀ λ
```

Here `J` is the constraint Jacobian and `C` is the joint position error.

## Tests

```bash
ctest --test-dir build -C Release --output-on-failure
```

`physics_checks` prints each measured value next to its limit, and covers:

- The analytic constraint and force Jacobians against finite differences
- Angular momentum drift of a spinning free body over 10 s
- Joint error of the constrained chain over 30 s, and how it shrinks at `h/4`
- Energy gain of the constrained and symplectic chains at `h/4`
- Joint stretch of the symplectic and implicit chains, and energy gain of the implicit chain

## Project layout

```text
src/
  main.cpp                Window, input and the fixed-step main loop
  physics/
    rigid_body.*          Body state, inertia and forces
    point_constraint.*    Joint error, Jacobians and spring forces
    world.*               The three integrators
    scene.*               The chain scene
  gfx/
    camera.hpp            Orbit camera
    gl.cpp                OpenGL function loader
    renderer.*            OpenGL 3.3 renderer
tests/
  physics_checks.cpp      Numerical checks of the physics
```

The `physics` library has no graphics dependencies, so the tests build and run without a window.
