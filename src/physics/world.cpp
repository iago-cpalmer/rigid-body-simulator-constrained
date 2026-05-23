#include "physics/world.hpp"

#include <algorithm>

using Eigen::MatrixXd;
using Eigen::VectorXd;

BodyId World::addBody(const BodyDesc& desc)
{
    bodies_.emplace_back(desc);
    return bodies_.size() - 1;
}

void World::addPointConstraint(const Eigen::Vector3d& anchor, std::optional<BodyId> bodyA,
                               std::optional<BodyId> bodyB, double stiffness)
{
    constraints_.emplace_back(bodies_, anchor, bodyA, bodyB, stiffness);
}

void World::step()
{
    switch (integrator) {
    case Integrator::Symplectic: stepSymplectic(); break;
    case Integrator::Implicit: stepImplicit(); break;
    case Integrator::SymplecticConstraints: stepSymplecticConstraints(); break;
    }
}

double World::energy() const
{
    double total = 0.0;
    for (const auto& body : bodies_) {
        total += body.kineticEnergy() - body.mass() * gravity.dot(body.position());
    }
    if (integrator != Integrator::SymplecticConstraints) {
        for (const auto& constraint : constraints_) {
            total += constraint.springEnergy(bodies_);
        }
    }
    return total;
}

double World::maxConstraintError() const
{
    double worst = 0.0;
    for (const auto& constraint : constraints_) {
        worst = std::max(worst, constraint.error(bodies_).norm());
    }
    return worst;
}

VectorXd World::velocities() const
{
    VectorXd v(dofs());
    for (Eigen::Index i = 0; const auto& body : bodies_) {
        v.segment<6>(6 * i++) = body.velocity();
    }
    return v;
}

VectorXd World::bodyForces() const
{
    VectorXd f(dofs());
    for (Eigen::Index i = 0; const auto& body : bodies_) {
        f.segment<6>(6 * i++) = body.force(gravity);
    }
    return f;
}

VectorXd World::springForces() const
{
    VectorXd f = VectorXd::Zero(dofs());
    for (const auto& constraint : constraints_) {
        constraint.addForce(bodies_, f);
    }
    return f;
}

MatrixXd World::massMatrix() const
{
    MatrixXd m = MatrixXd::Zero(dofs(), dofs());
    for (Eigen::Index i = 0; const auto& body : bodies_) {
        m.block<3, 3>(i, i).diagonal().setConstant(body.mass());
        m.block<3, 3>(i + 3, i + 3) = body.inertia();
        i += 6;
    }
    return m;
}

MatrixXd World::inverseMassMatrix() const
{
    MatrixXd m = MatrixXd::Zero(dofs(), dofs());
    for (Eigen::Index i = 0; const auto& body : bodies_) {
        m.block<3, 3>(i, i).diagonal().setConstant(1.0 / body.mass());
        m.block<3, 3>(i + 3, i + 3) = body.inverseInertia();
        i += 6;
    }
    return m;
}

void World::integrate(const VectorXd& v)
{
    for (Eigen::Index i = 0; auto& body : bodies_) {
        const Vector6d bodyVelocity = v.segment<6>(6 * i++);
        body.advance(timeStep * bodyVelocity);
        body.setVelocity(bodyVelocity);
    }
}

void World::stepSymplectic()
{
    const VectorXd f = bodyForces() + springForces();
    integrate(velocities() + timeStep * (inverseMassMatrix() * f));
}

void World::stepImplicit()
{
    const VectorXd v = velocities();
    const VectorXd f = bodyForces() + springForces();

    MatrixXd dFdx = MatrixXd::Zero(dofs(), dofs());
    for (const auto& constraint : constraints_) {
        constraint.addForceJacobian(bodies_, dFdx);
    }

    VectorXd dFdv(dofs());
    for (Eigen::Index i = 0; const auto& body : bodies_) {
        dFdv.segment<6>(6 * i++).setConstant(-body.damping());
    }

    MatrixXd a = massMatrix() - timeStep * timeStep * dFdx;
    a.diagonal() -= timeStep * dFdv;
    const VectorXd b = timeStep * (f + timeStep * (dFdx * v));

    integrate(v + a.partialPivLu().solve(b));
}

void World::stepSymplecticConstraints()
{
    if (constraints_.empty()) {
        stepSymplectic();
        return;
    }

    const MatrixXd massInv = inverseMassMatrix();
    const VectorXd predicted = velocities() + timeStep * (massInv * bodyForces());

    MatrixXd jacobian = MatrixXd::Zero(rows(), dofs());
    VectorXd error(rows());
    for (Eigen::Index row = 0; const auto& constraint : constraints_) {
        constraint.addJacobian(bodies_, jacobian.middleRows<PointConstraint::kRows>(row));
        error.segment<PointConstraint::kRows>(row) = constraint.error(bodies_);
        row += PointConstraint::kRows;
    }

    const MatrixXd massInvJt = massInv * jacobian.transpose();
    MatrixXd a = jacobian * massInvJt;
    a.diagonal().array() += 1e-6;

    const VectorXd rhs = jacobian * predicted + (baumgarte / timeStep) * error;
    const VectorXd lambda = a.ldlt().solve(rhs);

    integrate(predicted - massInvJt * lambda);
}
