cmake_minimum_required(VERSION 3.21)
project(transformations)

set(CMAKE_CXX_STANDARD 17)

add_executable(transformations main.cpp
        mathUtils.cpp mathUtils.h
        EulerAngles.cpp EulerAngles.h
        Quaternion.cpp Quaternion.h
        vector3.cpp vector3.h RotationMatrix.cpp RotationMatrix.h)
