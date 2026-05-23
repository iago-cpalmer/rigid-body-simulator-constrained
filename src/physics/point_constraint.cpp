#include "physics/point_constraint.hpp"

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {

using PointJacobian = Eigen::Matrix<double, 3, 6>;

constexpr std::array kSigns{1.0, -1.0};

Eigen::Index dofOf(BodyId id)
{
    return static_cast<Eigen::Index>(id) * 6;
}

PointJacobian pointJacobian(const Vector3d& arm)
{
    PointJacobian g;
    g << Matrix3d::Identity(), -skew(arm);
    return g;
}

}

PointConstraint::PointConstraint(std::span<const RigidBody> bodies, const Vector3d& anchor,
                                 std::optional<BodyId> bodyA, std::optional<BodyId> bodyB, double stiffness)
    : stiffness_{stiffness}
{
    const auto attach = [&](std::optional<BodyId> id) {
        return Attachment{id, id ? bodies[*id].worldToLocal(anchor) : anchor};
    };
    attachments_ = {attach(bodyA), attach(bodyB)};
}

Vector3d PointConstraint::worldPoint(std::span<const RigidBody> bodies, const Attachment& attachment) const
{
    return attachment.body ? bodies[*attachment.body].localToWorld(attachment.point) : attachment.point;
}

Vector3d PointConstraint::error(std::span<const RigidBody> bodies) const
{
    return worldPoint(bodies, attachments_[0]) - worldPoint(bodies, attachments_[1]);
}

Vector3d PointConstraint::midpoint(std::span<const RigidBody> bodies) const
{
    return 0.5 * (worldPoint(bodies, attachments_[0]) + worldPoint(bodies, attachments_[1]));
}

double PointConstraint::springEnergy(std::span<const RigidBody> bodies) const
{
    return 0.5 * stiffness_ * error(bodies).squaredNorm();
}

void PointConstraint::addJacobian(std::span<const RigidBody> bodies, Eigen::Ref<Eigen::MatrixXd> rows) const
{
    for (std::size_t i = 0; i < attachments_.size(); ++i) {
        const auto& a = attachments_[i];
        if (!a.body) {
            continue;
        }
        const Vector3d arm = worldPoint(bodies, a) - bodies[*a.body].position();
        rows.middleCols<6>(dofOf(*a.body)) += kSigns[i] * pointJacobian(arm);
    }
}

void PointConstraint::addForce(std::span<const RigidBody> bodies, Eigen::Ref<Eigen::VectorXd> force) const
{
    const Vector3d pull = -stiffness_ * error(bodies);

    for (std::size_t i = 0; i < attachments_.size(); ++i) {
        const auto& a = attachments_[i];
        if (!a.body) {
            continue;
        }
        const Vector3d arm = worldPoint(bodies, a) - bodies[*a.body].position();
        force.segment<6>(dofOf(*a.body)) += pointJacobian(arm).transpose() * (kSigns[i] * pull);
    }
}

void PointConstraint::addForceJacobian(std::span<const RigidBody> bodies, Eigen::Ref<Eigen::MatrixXd> dFdx) const
{
    const Vector3d pull = -stiffness_ * error(bodies);

    for (std::size_t i = 0; i < attachments_.size(); ++i) {
        const auto& a = attachments_[i];
        if (!a.body) {
            continue;
        }
        const Vector3d armA = worldPoint(bodies, a) - bodies[*a.body].position();
        const PointJacobian gA = pointJacobian(armA);

        for (std::size_t j = 0; j < attachments_.size(); ++j) {
            const auto& b = attachments_[j];
            if (!b.body) {
                continue;
            }
            const Vector3d armB = worldPoint(bodies, b) - bodies[*b.body].position();
            dFdx.block<6, 6>(dofOf(*a.body), dofOf(*b.body))
                -= stiffness_ * kSigns[i] * kSigns[j] * gA.transpose() * pointJacobian(armB);
        }

        // Arm rotates with the body, so the torque picks up a geometric stiffness term.
        dFdx.block<3, 3>(dofOf(*a.body) + 3, dofOf(*a.body) + 3) += skew(kSigns[i] * pull) * skew(armA);
    }
}
