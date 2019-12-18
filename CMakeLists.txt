cmake_minimum_required(VERSION 2.8.3)
project(vrep_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )

find_package(catkin REQUIRED COMPONENTS
  roscpp

)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
)

SET(HEADS
  include/model/rbdl.h
  include/model/model.h
  include/model/robot.h
  include/vrep_test/vrep_bridge.h
)

set(SOURCES
  src/model/rbdl.cpp
  src/controller.cpp
  src/vrep_bridge.cpp
)

add_library(test_library
 ${SOURCES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}  
)
add_definitions(-Ddof=7)

add_executable(vrep_test
  src/main.cpp)

target_link_libraries(test_library
  ${catkin_LIBRARIES}
)
target_link_libraries(vrep_test
  ${catkin_LIBRARIES}
  rbdl
  test_library
)



