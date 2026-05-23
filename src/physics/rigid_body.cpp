#include "physics/rigid_body.hpp"

using Eigen::Matrix3d;
using Eigen::Vector3d;

Matrix3d skew(const Vector3d& v)
{
    Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return m;
}

RigidBody::RigidBody(const BodyDesc& desc)
    : position_{desc.position}
    , rotation_{desc.rotation.normalized()}
    , size_{desc.size}
    , mass_{desc.mass}
    , damping_{desc.damping}
{
    const Vector3d s2 = size_.cwiseAbs2();
    bodyInertia_ = mass_ / 12.0 * Vector3d{s2.y() + s2.z(), s2.x() + s2.z(), s2.x() + s2.y()};
}

Vector6d RigidBody::velocity() const
{
    Vector6d v;
    v << linearVelocity_, angularVelocity_;
    return v;
}

void RigidBody::setVelocity(const Vector6d& velocity)
{
    linearVelocity_ = velocity.head<3>();
    angularVelocity_ = velocity.tail<3>();
}

void RigidBody::advance(const Vector6d& delta)
{
    position_ += delta.head<3>();

    const Vector3d rotationVector = delta.tail<3>();
    const double angle = rotationVector.norm();
    if (angle > 1e-12) {
        rotation_ = (Eigen::Quaterniond{Eigen::AngleAxisd{angle, rotationVector / angle}} * rotation_).normalized();
    }
}

Matrix3d RigidBody::inertia() const
{
    const Matrix3d r = rotation_.toRotationMatrix();
    return r * bodyInertia_.asDiagonal() * r.transpose();
}

Matrix3d RigidBody::inverseInertia() const
{
    const Matrix3d r = rotation_.toRotationMatrix();
    return r * bodyInertia_.cwiseInverse().asDiagonal() * r.transpose();
}

Vector6d RigidBody::force(const Vector3d& gravity) const
{
    Vector6d f;
    f.head<3>() = mass_ * gravity - damping_ * linearVelocity_;
    f.tail<3>() = -damping_ * angularVelocity_ - angularVelocity_.cross(inertia() * angularVelocity_);
    return f;
}

double RigidBody::kineticEnergy() const
{
    return 0.5 * mass_ * linearVelocity_.squaredNorm()
         + 0.5 * angularVelocity_.dot(inertia() * angularVelocity_);
}
