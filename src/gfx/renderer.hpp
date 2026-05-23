#pragma once

#include "gfx/camera.hpp"
#include "physics/world.hpp"

#include <glad/gl.h>

#include <span>
#include <string_view>

class Mesh {
public:
    struct Vertex {
        float position[3];
        float normal[3];
    };

    Mesh(std::span<const Vertex> vertices, std::span<const unsigned> indices);
    ~Mesh();
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    void draw() const;

private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint ebo_ = 0;
    GLsizei indexCount_ = 0;
};

class Shader {
public:
    Shader(std::string_view vertexSource, std::string_view fragmentSource);
    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    void use() const;
    void set(const char* name, const Eigen::Matrix4f& value) const;
    void set(const char* name, const Eigen::Vector3f& value) const;
    void set(const char* name, bool value) const;

private:
    GLuint program_ = 0;
};

class Renderer {
public:
    Renderer();

    void draw(const World& world, const OrbitCamera& camera, int width, int height) const;

private:
    Shader shader_;
    Mesh cube_;
    Mesh sphere_;
};
