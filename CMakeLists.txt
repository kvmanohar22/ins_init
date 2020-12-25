SET(USE_ROS TRUE)

CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
PROJECT(ins_init)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_CXX_FLAGS "-Wall -g -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wpedantic")

FIND_PACKAGE(Eigen3 REQUIRED)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
  catkin_package(
    DEPENDS Eigen3
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES ins_init
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

LIST(APPEND SRCS
  src/ekf.cpp
)

IF(USE_ROS)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()

ADD_LIBRARY(ins_init SHARED
  ${SRCS}
)
TARGET_LINK_LIBRARIES(ins_init ${LINK_LIBS})

ADD_EXECUTABLE(test_ins_init_acc test/test_ins_init_acc.cpp)
TARGET_LINK_LIBRARIES(test_ins_init_acc ins_init)

ADD_EXECUTABLE(test_ins_init_vel test/test_ins_init_vel.cpp)
TARGET_LINK_LIBRARIES(test_ins_init_vel ins_init)

