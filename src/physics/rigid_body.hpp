#pragma once

#include <Eigen/Dense>

using Vector6d = Eigen::Matrix<double, 6, 1>;

[[nodiscard]] Eigen::Matrix3d skew(const Eigen::Vector3d& v);

struct BodyDesc {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d size = Eigen::Vector3d::Ones();
    double mass = 1.0;
    double damping = 0.0;
};

class RigidBody {
public:
    explicit RigidBody(const BodyDesc& desc);

    [[nodiscard]] const Eigen::Vector3d& position() const { return position_; }
    [[nodiscard]] const Eigen::Quaterniond& rotation() const { return rotation_; }
    [[nodiscard]] const Eigen::Vector3d& size() const { return size_; }
    [[nodiscard]] double mass() const { return mass_; }
    [[nodiscard]] double damping() const { return damping_; }

    [[nodiscard]] Vector6d velocity() const;
    void setVelocity(const Vector6d& velocity);
    void advance(const Vector6d& delta);

    [[nodiscard]] Eigen::Matrix3d inertia() const;
    [[nodiscard]] Eigen::Matrix3d inverseInertia() const;
    [[nodiscard]] Vector6d force(const Eigen::Vector3d& gravity) const;
    [[nodiscard]] double kineticEnergy() const;

    [[nodiscard]] Eigen::Vector3d localToWorld(const Eigen::Vector3d& p) const { return position_ + rotation_ * p; }
    [[nodiscard]] Eigen::Vector3d worldToLocal(const Eigen::Vector3d& p) const { return rotation_.conjugate() * (p - position_); }

private:
    Eigen::Vector3d position_;
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d linearVelocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d size_;
    Eigen::Vector3d bodyInertia_;
    double mass_;
    double damping_;
};
