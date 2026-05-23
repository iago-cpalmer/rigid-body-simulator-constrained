#include "physics/scene.hpp"

#include <cmath>
#include <format>
#include <iostream>
#include <vector>

namespace {

int failures = 0;

void check(const std::string& name, double value, double limit)
{
    const bool ok = std::isfinite(value) && value <= limit;
    failures += ok ? 0 : 1;
    std::cout << std::format("[{}] {}: {:.3e} (limit {:.1e})\n", ok ? "ok" : "FAIL", name, value, limit);
}

std::vector<RigidBody> twistedBodies()
{
    std::vector<RigidBody> bodies;
    bodies.emplace_back(BodyDesc{.position = {0.3, -0.2, 0.5},
                                 .rotation = Eigen::Quaterniond{Eigen::AngleAxisd{0.7, Eigen::Vector3d{1, 2, 3}.normalized()}},
                                 .size = {2, 1, 4}});
    bodies.emplace_back(BodyDesc{.position = {2.1, 0.4, -1.0},
                                 .rotation = Eigen::Quaterniond{Eigen::AngleAxisd{-1.1, Eigen::Vector3d{3, -1, 2}.normalized()}},
                                 .size = {1, 3, 2}});
    return bodies;
}

PointConstraint stretchedConstraint(std::vector<RigidBody>& bodies)
{
    PointConstraint constraint{bodies, {1.0, 0.5, 0.0}, BodyId{0}, BodyId{1}, 100.0};
    Vector6d nudge;
    nudge << 0.2, -0.1, 0.3, 0.1, 0.2, -0.15;
    bodies[1].advance(nudge);
    return constraint;
}

template <typename Fn>
Eigen::MatrixXd numericJacobian(const std::vector<RigidBody>& bodies, Eigen::Index outputs, Fn&& evaluate)
{
    constexpr double eps = 1e-6;
    const auto dofs = static_cast<Eigen::Index>(bodies.size()) * 6;

    Eigen::MatrixXd jacobian(outputs, dofs);
    for (Eigen::Index col = 0; col < dofs; ++col) {
        const auto perturbed = [&](double amount) {
            auto copy = bodies;
            Vector6d delta = Vector6d::Zero();
            delta[col % 6] = amount;
            copy[static_cast<std::size_t>(col / 6)].advance(delta);
            return Eigen::VectorXd{evaluate(copy)};
        };
        jacobian.col(col) = (perturbed(eps) - perturbed(-eps)) / (2.0 * eps);
    }
    return jacobian;
}

void checkConstraintJacobian()
{
    auto bodies = twistedBodies();
    const auto constraint = stretchedConstraint(bodies);

    Eigen::MatrixXd analytic = Eigen::MatrixXd::Zero(3, 12);
    constraint.addJacobian(bodies, analytic);

    const auto numeric = numericJacobian(bodies, 3, [&](const std::vector<RigidBody>& b) {
        return Eigen::VectorXd{constraint.error(b)};
    });
    check("constraint jacobian vs finite differences", (analytic - numeric).norm(), 1e-6);
}

void checkForceJacobian()
{
    auto bodies = twistedBodies();
    const auto constraint = stretchedConstraint(bodies);

    Eigen::MatrixXd analytic = Eigen::MatrixXd::Zero(12, 12);
    constraint.addForceJacobian(bodies, analytic);

    const auto numeric = numericJacobian(bodies, 12, [&](const std::vector<RigidBody>& b) {
        Eigen::VectorXd f = Eigen::VectorXd::Zero(12);
        constraint.addForce(b, f);
        return f;
    });
    check("force jacobian vs finite differences", (analytic - numeric).norm(), 1e-4);
}

void checkFreeBody()
{
    World world;
    world.integrator = Integrator::Symplectic;
    world.gravity.setZero();
    const BodyId id = world.addBody({.size = {2, 2, 6}});

    Vector6d spin;
    spin << 0, 0, 0, 1.0, 0.1, 0.5;
    auto& body = const_cast<RigidBody&>(world.bodies()[id]);
    body.setVelocity(spin);

    const auto momentum = [&] { return Eigen::Vector3d{body.inertia() * body.velocity().tail<3>()}; };
    const Eigen::Vector3d initial = momentum();
    for (int i = 0; i < 2000; ++i) {
        world.step();
    }
    check("free body angular momentum drift over 10s", (momentum() - initial).norm() / initial.norm(), 0.02);
}

struct ChainRun {
    double jointError = 0.0;
    double energyGain = 0.0;
};

ChainRun runChain(Integrator integrator, double timeStep)
{
    World world = makeChainScene(integrator);
    world.timeStep = timeStep;
    const double initial = world.energy();

    ChainRun run;
    const auto steps = static_cast<int>(30.0 / timeStep);
    for (int i = 0; i < steps; ++i) {
        world.step();
        run.jointError = std::max(run.jointError, world.maxConstraintError());
        run.energyGain = std::max(run.energyGain, world.energy() - initial);
    }
    return run;
}

void checkConstrainedChain()
{
    const auto coarse = runChain(Integrator::SymplecticConstraints, 0.005);
    const auto fine = runChain(Integrator::SymplecticConstraints, 0.00125);
    check("constrained: joint error over 30s", coarse.jointError, 0.05);
    check("constrained: joint error ratio at h/4", fine.jointError / coarse.jointError, 0.25);
    check("constrained: energy gain ratio at h/4", fine.energyGain / coarse.energyGain, 0.5);
}

void checkPenaltyChain()
{
    const auto coarse = runChain(Integrator::Symplectic, 0.005);
    const auto fine = runChain(Integrator::Symplectic, 0.00125);
    check("symplectic: joint stretch over 30s", coarse.jointError, 2.0);
    check("symplectic: energy gain ratio at h/4", fine.energyGain / coarse.energyGain, 0.5);
}

void checkImplicitChain()
{
    const auto run = runChain(Integrator::Implicit, 0.005);
    check("implicit: joint stretch over 30s", run.jointError, 2.0);
    check("implicit: energy gain over 30s", run.energyGain, 1e-3);
}

}

int main()
{
    checkConstraintJacobian();
    checkForceJacobian();
    checkFreeBody();
    checkConstrainedChain();
    checkPenaltyChain();
    checkImplicitChain();
    return failures == 0 ? 0 : 1;
}
