cmake_minimum_required(VERSION 3.7)

project(render)

set(CMAKE_CXX_FLAGS "-O3")

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})


add_executable(render src/main.cpp)

target_compile_features(render PRIVATE cxx_std_17)

target_include_directories(render PRIVATE deps/glm-0.9.8.5)
target_include_directories(render PRIVATE deps/tinyply-2.2/source)

target_link_libraries(render SDL2::SDL2)
