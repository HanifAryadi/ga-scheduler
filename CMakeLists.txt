cmake_minimum_required(VERSION 3.5)

project(ga-scheduler)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Boost REQUIRED COMPONENTS system program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


add_executable(scheduling_alg
    src/main.cc
    src/decoded_solution.cc
    src/decoded_solution.hh
    src/encoded_solution.cc
    src/encoded_solution.hh
    src/genetic_algorithm.cc
    src/genetic_algorithm.hh
    src/problem.cc
    src/problem.hh
    src/stat_recorder.cc
    src/stat_recorder.hh
    src/tabu_search.cc
    src/tabu_search.hh)

target_link_libraries(scheduling_alg ${Boost_LIBRARIES})



