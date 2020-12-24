cmake_minimum_required(VERSION 3.5)
project(OGM VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

find_package(Matplot++)

add_executable(OGM main.cpp)

target_link_libraries(OGM PUBLIC Matplot++::matplot)

