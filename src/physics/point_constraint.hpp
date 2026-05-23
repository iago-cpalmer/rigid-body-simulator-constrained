#pragma once

#include "physics/rigid_body.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <span>

using BodyId = std::size_t;

class PointConstraint {
public:
    static constexpr int kRows = 3;

    PointConstraint(std::span<const RigidBody> bodies, const Eigen::Vector3d& anchor,
                    std::optional<BodyId> bodyA, std::optional<BodyId> bodyB, double stiffness);

    [[nodiscard]] Eigen::Vector3d error(std::span<const RigidBody> bodies) const;
    [[nodiscard]] Eigen::Vector3d midpoint(std::span<const RigidBody> bodies) const;
    [[nodiscard]] double springEnergy(std::span<const RigidBody> bodies) const;

    void addJacobian(std::span<const RigidBody> bodies, Eigen::Ref<Eigen::MatrixXd> rows) const;
    void addForce(std::span<const RigidBody> bodies, Eigen::Ref<Eigen::VectorXd> force) const;
    void addForceJacobian(std::span<const RigidBody> bodies, Eigen::Ref<Eigen::MatrixXd> dFdx) const;

private:
    struct Attachment {
        std::optional<BodyId> body;
        Eigen::Vector3d point;
    };

    [[nodiscard]] Eigen::Vector3d worldPoint(std::span<const RigidBody> bodies, const Attachment& attachment) const;

    std::array<Attachment, 2> attachments_;
    double stiffness_;
};
