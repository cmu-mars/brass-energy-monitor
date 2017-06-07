cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(ignition-math2 QUIET REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${IGNITION-MATH2_CXX_FLAGS} -std=c++11")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH2_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(energy_monitor SHARED energy_monitor.cc)
target_link_libraries(energy_monitor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

