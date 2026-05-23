#include "gfx/renderer.hpp"
#include "physics/scene.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <exception>
#include <format>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

struct GlfwContext {
    GlfwContext()
    {
        if (glfwInit() != GLFW_TRUE) {
            throw std::runtime_error{"failed to initialize GLFW"};
        }
    }
    ~GlfwContext() { glfwTerminate(); }
    GlfwContext(const GlfwContext&) = delete;
    GlfwContext& operator=(const GlfwContext&) = delete;
};

struct WindowDeleter {
    void operator()(GLFWwindow* window) const { glfwDestroyWindow(window); }
};

struct App {
    World world = makeChainScene(Integrator::SymplecticConstraints);
    OrbitCamera camera;
    bool paused = false;
    bool orbiting = false;
    double lastX = 0.0;
    double lastY = 0.0;
    bool titleDirty = true;

    void select(Integrator integrator)
    {
        world.integrator = integrator;
        titleDirty = true;
    }
};

App& appOf(GLFWwindow* window)
{
    return *static_cast<App*>(glfwGetWindowUserPointer(window));
}

void onKey(GLFWwindow* window, int key, int, int action, int)
{
    if (action != GLFW_PRESS) {
        return;
    }

    auto& app = appOf(window);
    switch (key) {
    case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
    case GLFW_KEY_P:
        app.paused = !app.paused;
        app.titleDirty = true;
        break;
    case GLFW_KEY_R: app.world = makeChainScene(app.world.integrator); break;
    case GLFW_KEY_1: app.select(Integrator::Symplectic); break;
    case GLFW_KEY_2: app.select(Integrator::Implicit); break;
    case GLFW_KEY_3: app.select(Integrator::SymplecticConstraints); break;
    default: break;
    }
}

void onMouseButton(GLFWwindow* window, int button, int action, int)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        auto& app = appOf(window);
        app.orbiting = action == GLFW_PRESS;
        glfwGetCursorPos(window, &app.lastX, &app.lastY);
    }
}

void onCursor(GLFWwindow* window, double x, double y)
{
    auto& app = appOf(window);
    if (app.orbiting) {
        app.camera.orbit(static_cast<float>(x - app.lastX), static_cast<float>(y - app.lastY));
        app.lastX = x;
        app.lastY = y;
    }
}

void onScroll(GLFWwindow* window, double, double dy)
{
    appOf(window).camera.zoom(static_cast<float>(dy));
}

void run()
{
    GlfwContext glfw;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    const std::unique_ptr<GLFWwindow, WindowDeleter> window{
        glfwCreateWindow(1280, 720, "Rigid Body Simulator", nullptr, nullptr)};
    if (!window) {
        throw std::runtime_error{"failed to create window"};
    }

    glfwMakeContextCurrent(window.get());
    glfwSwapInterval(1);
    if (gladLoadGL(glfwGetProcAddress) == 0) {
        throw std::runtime_error{"failed to load OpenGL"};
    }

    App app;
    glfwSetWindowUserPointer(window.get(), &app);
    glfwSetKeyCallback(window.get(), onKey);
    glfwSetMouseButtonCallback(window.get(), onMouseButton);
    glfwSetCursorPosCallback(window.get(), onCursor);
    glfwSetScrollCallback(window.get(), onScroll);

    const Renderer renderer;

    double previous = glfwGetTime();
    double accumulator = 0.0;

    while (glfwWindowShouldClose(window.get()) == GLFW_FALSE) {
        glfwPollEvents();

        const double now = glfwGetTime();
        accumulator = std::min(accumulator + (now - previous), 0.1);
        previous = now;

        if (app.paused) {
            accumulator = 0.0;
        }
        for (; accumulator >= app.world.timeStep; accumulator -= app.world.timeStep) {
            app.world.step();
        }

        if (app.titleDirty) {
            const auto title = std::format("Rigid Body Simulator | {}{}", toString(app.world.integrator), app.paused ? " | paused" : "");
            glfwSetWindowTitle(window.get(), title.c_str());
            app.titleDirty = false;
        }

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window.get(), &width, &height);
        renderer.draw(app.world, app.camera, width, height);
        glfwSwapBuffers(window.get());
    }
}

}

int main()
{
    try {
        run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
