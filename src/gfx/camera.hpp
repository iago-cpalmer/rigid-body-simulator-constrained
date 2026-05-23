#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <numbers>

struct OrbitCamera {
    Eigen::Vector3f target{-3.0f, -3.0f, -4.0f};
    float distance = 30.0f;
    float yaw = 1.1f;
    float pitch = 0.15f;
    float fovY = std::numbers::pi_v<float> / 3.0f;

    void orbit(float dx, float dy)
    {
        constexpr float limit = 1.5f;
        yaw -= dx * 0.005f;
        pitch = std::clamp(pitch + dy * 0.005f, -limit, limit);
    }

    void zoom(float amount)
    {
        distance = std::clamp(distance * std::pow(0.9f, amount), 2.0f, 200.0f);
    }

    [[nodiscard]] Eigen::Vector3f eye() const
    {
        const Eigen::Vector3f offset{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
        return target + distance * offset;
    }

    [[nodiscard]] Eigen::Matrix4f view() const
    {
        const Eigen::Vector3f position = eye();
        const Eigen::Vector3f back = (position - target).normalized();
        const Eigen::Vector3f right = Eigen::Vector3f::UnitY().cross(back).normalized();
        const Eigen::Vector3f up = back.cross(right);

        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        m.row(0).head<3>() = right;
        m.row(1).head<3>() = up;
        m.row(2).head<3>() = back;
        m(0, 3) = -right.dot(position);
        m(1, 3) = -up.dot(position);
        m(2, 3) = -back.dot(position);
        return m;
    }

    [[nodiscard]] Eigen::Matrix4f projection(float aspect) const
    {
        constexpr float zNear = 0.1f;
        constexpr float zFar = 500.0f;
        const float f = 1.0f / std::tan(fovY / 2.0f);

        Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
        m(0, 0) = f / aspect;
        m(1, 1) = f;
        m(2, 2) = (zFar + zNear) / (zNear - zFar);
        m(2, 3) = 2.0f * zFar * zNear / (zNear - zFar);
        m(3, 2) = -1.0f;
        return m;
    }
};
