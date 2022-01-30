cmake_minimum_required(VERSION 3.5)

project(ga_scheduler)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_message_files(
  FILES
  Job.msg
  Machine.msg
  MachineTask.msg
  MachineTasks.msg
  Operation.msg
  Problem.msg
  Solution.msg
  TransporterTask.msg
  TransporterTasks.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
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
  src/tabu_search.hh
)
target_link_libraries(${PROJECT_NAME})

add_executable(scheduler_node src/scheduler_node.cc src/rosmsg.cc)
add_dependencies(scheduler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scheduler_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
