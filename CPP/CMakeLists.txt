cmake_minimum_required(VERSION 3.10)

# set the project name
project(JDE_CPP_Challenge VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_executable(JDE_CPP_Challenge labyrinth.cpp)