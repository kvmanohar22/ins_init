cmake_minimum_required(VERSION 3.15)
project(ins_init)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_CXX_FLAGS "-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wpedantic")
SET(HEADERS include)

find_package(Eigen3 REQUIRED)

set(PROJECT_HEADERS include)
include_directories(
  ${PROJECT_HEADERS}
  ${EIGEN3_INCLUDE_DIRS}
)

list(APPEND SRCS
  src/coarse_init.cpp
)

add_library(ins_init SHARED
  ${SRCS}
)
target_link_libraries(ins_init ${LIBS})

add_executable(test_ins_init test/test_ins_init.cpp)
target_link_libraries(test_ins_init ins_init)

