cmake_minimum_required(VERSION 3.21)
project(transformations)

set(CMAKE_CXX_STANDARD 14)
set(proj_name 3DMath)
set(sources
        main.cpp
        src/MathUtils.cpp
        src/EulerAngles.cpp
        src/Quaternion.cpp
        src/Vector3.cpp
        src/RotationMatrix.cpp
        src/Matrix4x3.cpp
        src/Vector2.cpp)

add_executable(${proj_name} ${sources})

target_include_directories(${proj_name} PRIVATE inc)