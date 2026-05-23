#include "physics/scene.hpp"

// Same layout as Exercise1.unity with z flipped, Unity being left-handed.
World makeChainScene(Integrator integrator)
{
    World world;
    world.integrator = integrator;

    const Eigen::Vector3d size{2.0, 2.0, 6.0};
    const BodyId first = world.addBody({.position = {-4.0, 4.0, -3.0}, .size = size});
    const BodyId second = world.addBody({.position = {-6.0, 6.0, -9.0}, .size = size});

    constexpr double stiffness = 100.0;
    world.addPointConstraint({-3.0, 3.0, 0.0}, first, std::nullopt, stiffness);
    world.addPointConstraint({-5.0, 5.0, -6.0}, first, second, stiffness);

    return world;
}
