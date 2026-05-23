#include "gfx/renderer.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::string_view kVertexShader = R"(#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uViewProjection;

out vec3 vWorld;
out vec3 vNormal;

void main()
{
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorld = world.xyz;
    vNormal = transpose(inverse(mat3(uModel))) * aNormal;
    gl_Position = uViewProjection * world;
}
)";

constexpr std::string_view kFragmentShader = R"(#version 330 core
in vec3 vWorld;
in vec3 vNormal;

uniform vec3 uColor;
uniform vec3 uEye;
uniform bool uChecker;

out vec4 outColor;

void main()
{
    vec3 n = normalize(vNormal);
    vec3 l = normalize(vec3(0.45, 0.8, 0.4));
    vec3 h = normalize(l + normalize(uEye - vWorld));

    vec3 albedo = uColor;
    if (uChecker) {
        vec2 cell = floor(vWorld.xz / 2.0);
        albedo *= mod(cell.x + cell.y, 2.0) < 1.0 ? 1.0 : 0.82;
    }

    float ambient = 0.3 + 0.15 * n.y;
    float diffuse = max(dot(n, l), 0.0);
    float specular = pow(max(dot(n, h), 0.0), 48.0) * 0.2;
    outColor = vec4(albedo * (ambient + 0.75 * diffuse) + specular, 1.0);
}
)";

GLuint compile(GLenum type, std::string_view source)
{
    const GLuint shader = glCreateShader(type);
    const char* data = source.data();
    const auto length = static_cast<GLint>(source.size());
    glShaderSource(shader, 1, &data, &length);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        std::string log(1024, '\0');
        glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
        glDeleteShader(shader);
        throw std::runtime_error{"shader compile failed: " + log};
    }
    return shader;
}

Mesh makeCube()
{
    constexpr std::array<std::array<float, 3>, 6> normals{{
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
    }};

    std::vector<Mesh::Vertex> vertices;
    std::vector<unsigned> indices;

    for (const auto& a : normals) {
        const Eigen::Vector3f n{a[0], a[1], a[2]};
        const Eigen::Vector3f u = n.unitOrthogonal();
        const Eigen::Vector3f v = n.cross(u);
        const auto base = static_cast<unsigned>(vertices.size());

        for (const auto [su, sv] : {std::pair{-1.f, -1.f}, {1.f, -1.f}, {1.f, 1.f}, {-1.f, 1.f}}) {
            const Eigen::Vector3f p = 0.5f * (n + su * u + sv * v);
            vertices.push_back({{p.x(), p.y(), p.z()}, {n.x(), n.y(), n.z()}});
        }
        indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return Mesh{vertices, indices};
}

Mesh makeSphere(int stacks, int slices)
{
    std::vector<Mesh::Vertex> vertices;
    std::vector<unsigned> indices;

    for (int i = 0; i <= stacks; ++i) {
        const float phi = std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(stacks);
        for (int j = 0; j <= slices; ++j) {
            const float theta = 2.0f * std::numbers::pi_v<float> * static_cast<float>(j) / static_cast<float>(slices);
            const float x = std::sin(phi) * std::cos(theta);
            const float y = std::cos(phi);
            const float z = std::sin(phi) * std::sin(theta);
            vertices.push_back({{0.5f * x, 0.5f * y, 0.5f * z}, {x, y, z}});
        }
    }

    const auto stride = static_cast<unsigned>(slices + 1);
    for (unsigned i = 0; i < static_cast<unsigned>(stacks); ++i) {
        for (unsigned j = 0; j < static_cast<unsigned>(slices); ++j) {
            const unsigned a = i * stride + j;
            const unsigned b = a + stride;
            indices.insert(indices.end(), {a, a + 1, b, a + 1, b + 1, b});
        }
    }
    return Mesh{vertices, indices};
}

Eigen::Matrix4f modelMatrix(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& scale)
{
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.topLeftCorner<3, 3>() = rotation.cast<float>().toRotationMatrix() * scale.cast<float>().asDiagonal();
    m.topRightCorner<3, 1>() = position.cast<float>();
    return m;
}

}

Mesh::Mesh(std::span<const Vertex> vertices, std::span<const unsigned> indices)
    : indexCount_{static_cast<GLsizei>(indices.size())}
{
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size_bytes()), vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size_bytes()), indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<const void*>(offsetof(Vertex, normal)));
    glBindVertexArray(0);
}

Mesh::~Mesh()
{
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
}

void Mesh::draw() const
{
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
}

Shader::Shader(std::string_view vertexSource, std::string_view fragmentSource)
{
    const GLuint vertex = compile(GL_VERTEX_SHADER, vertexSource);
    const GLuint fragment = compile(GL_FRAGMENT_SHADER, fragmentSource);

    program_ = glCreateProgram();
    glAttachShader(program_, vertex);
    glAttachShader(program_, fragment);
    glLinkProgram(program_);
    glDeleteShader(vertex);
    glDeleteShader(fragment);

    GLint ok = GL_FALSE;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        std::string log(1024, '\0');
        glGetProgramInfoLog(program_, static_cast<GLsizei>(log.size()), nullptr, log.data());
        glDeleteProgram(program_);
        throw std::runtime_error{"shader link failed: " + log};
    }
}

Shader::~Shader()
{
    glDeleteProgram(program_);
}

void Shader::use() const
{
    glUseProgram(program_);
}

void Shader::set(const char* name, const Eigen::Matrix4f& value) const
{
    glUniformMatrix4fv(glGetUniformLocation(program_, name), 1, GL_FALSE, value.data());
}

void Shader::set(const char* name, const Eigen::Vector3f& value) const
{
    glUniform3fv(glGetUniformLocation(program_, name), 1, value.data());
}

void Shader::set(const char* name, bool value) const
{
    glUniform1i(glGetUniformLocation(program_, name), value ? 1 : 0);
}

Renderer::Renderer()
    : shader_{kVertexShader, kFragmentShader}
    , cube_{makeCube()}
    , sphere_{makeSphere(16, 32)}
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);
}

void Renderer::draw(const World& world, const OrbitCamera& camera, int width, int height) const
{
    glViewport(0, 0, width, height);
    glClearColor(0.13f, 0.15f, 0.18f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const float aspect = static_cast<float>(width) / static_cast<float>(std::max(height, 1));
    const Eigen::Matrix4f viewProjection = camera.projection(aspect) * camera.view();

    shader_.use();
    shader_.set("uViewProjection", viewProjection);
    shader_.set("uEye", Eigen::Vector3f{camera.eye()});

    shader_.set("uChecker", true);
    shader_.set("uColor", Eigen::Vector3f{0.42f, 0.45f, 0.48f});
    shader_.set("uModel", modelMatrix({-3.0, -14.5, -4.0}, Eigen::Quaterniond::Identity(), {80.0, 1.0, 80.0}));
    cube_.draw();

    shader_.set("uChecker", false);

    static constexpr std::array palette{
        std::array{0.25f, 0.55f, 0.85f},
        std::array{0.90f, 0.50f, 0.20f},
        std::array{0.35f, 0.70f, 0.45f},
    };

    for (std::size_t i = 0; const auto& body : world.bodies()) {
        const auto& c = palette[i++ % palette.size()];
        shader_.set("uColor", Eigen::Vector3f{c[0], c[1], c[2]});
        shader_.set("uModel", modelMatrix(body.position(), body.rotation(), body.size()));
        cube_.draw();
    }

    glDisable(GL_DEPTH_TEST);
    shader_.set("uColor", Eigen::Vector3f{0.95f, 0.85f, 0.25f});
    for (const auto& constraint : world.constraints()) {
        shader_.set("uModel", modelMatrix(constraint.midpoint(world.bodies()), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Constant(0.5)));
        sphere_.draw();
    }
    glEnable(GL_DEPTH_TEST);
}
