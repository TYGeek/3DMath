cmake_minimum_required(VERSION 3.21)
project(transformations)

set(CMAKE_CXX_STANDARD 17)

set(sources
        main.cpp
        src/MathUtils.cpp
        src/EulerAngles.cpp
        src/Quaternion.cpp
        src/Vector3.cpp
        src/RotationMatrix.cpp
        src/Matrix4x3.cpp
        src/Vector2.cpp)

include_directories(inc)

add_executable(transformations ${sources})
