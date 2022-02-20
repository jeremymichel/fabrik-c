cmake_minimum_required(VERSION 3.21)
project (ray_game C)

find_package(raylib 4.0.0 REQUIRED)

set(CMAKE_C_STANDARD 17)

add_executable(ray_game main.c)

target_link_libraries(ray_game raylib -lm)