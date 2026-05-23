cmake_minimum_required(VERSION 3.20)
project(RigidBodySimulator LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR do-not-configure)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(eigen glfw)

add_library(eigen_headers INTERFACE)
target_include_directories(eigen_headers SYSTEM INTERFACE ${eigen_SOURCE_DIR})

add_library(project_warnings INTERFACE)
if(MSVC)
    target_compile_options(project_warnings INTERFACE /W4 /permissive- /utf-8)
    target_compile_definitions(project_warnings INTERFACE NOMINMAX _USE_MATH_DEFINES)
else()
    target_compile_options(project_warnings INTERFACE -Wall -Wextra -Wpedantic)
endif()

add_library(physics STATIC
    src/physics/rigid_body.cpp
    src/physics/point_constraint.cpp
    src/physics/world.cpp
    src/physics/scene.cpp)
target_include_directories(physics PUBLIC src)
target_link_libraries(physics PUBLIC eigen_headers PRIVATE project_warnings)

add_library(gfx STATIC
    src/gfx/gl.cpp
    src/gfx/renderer.cpp)
target_include_directories(gfx SYSTEM PUBLIC ${glfw_SOURCE_DIR}/deps)
target_link_libraries(gfx PUBLIC physics glfw PRIVATE project_warnings)

add_executable(rigidbody_sim src/main.cpp)
target_link_libraries(rigidbody_sim PRIVATE gfx project_warnings)

enable_testing()
add_executable(physics_checks tests/physics_checks.cpp)
target_link_libraries(physics_checks PRIVATE physics project_warnings)
add_test(NAME physics_checks COMMAND physics_checks)
