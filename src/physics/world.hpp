#pragma once

#include "physics/point_constraint.hpp"
#include "physics/rigid_body.hpp"

#include <string_view>
#include <vector>

enum class Integrator { Symplectic, Implicit, SymplecticConstraints };

[[nodiscard]] constexpr std::string_view toString(Integrator integrator)
{
    switch (integrator) {
    case Integrator::Symplectic: return "symplectic";
    case Integrator::Implicit: return "implicit";
    case Integrator::SymplecticConstraints: return "symplectic + constraints";
    }
    return "unknown";
}

class World {
public:
    Integrator integrator = Integrator::SymplecticConstraints;
    double timeStep = 0.005;
    double baumgarte = 0.1;
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};

    BodyId addBody(const BodyDesc& desc);
    void addPointConstraint(const Eigen::Vector3d& anchor, std::optional<BodyId> bodyA,
                            std::optional<BodyId> bodyB, double stiffness);
    void step();

    [[nodiscard]] std::span<const RigidBody> bodies() const { return bodies_; }
    [[nodiscard]] std::span<const PointConstraint> constraints() const { return constraints_; }

    [[nodiscard]] double energy() const;
    [[nodiscard]] double maxConstraintError() const;

private:
    [[nodiscard]] Eigen::Index dofs() const { return static_cast<Eigen::Index>(bodies_.size()) * 6; }
    [[nodiscard]] Eigen::Index rows() const { return static_cast<Eigen::Index>(constraints_.size()) * PointConstraint::kRows; }

    [[nodiscard]] Eigen::VectorXd velocities() const;
    [[nodiscard]] Eigen::VectorXd bodyForces() const;
    [[nodiscard]] Eigen::VectorXd springForces() const;
    [[nodiscard]] Eigen::MatrixXd massMatrix() const;
    [[nodiscard]] Eigen::MatrixXd inverseMassMatrix() const;
    void integrate(const Eigen::VectorXd& v);

    void stepSymplectic();
    void stepImplicit();
    void stepSymplecticConstraints();

    std::vector<RigidBody> bodies_;
    std::vector<PointConstraint> constraints_;
};
