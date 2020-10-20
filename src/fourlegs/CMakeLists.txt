cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#### CMakeLists for collision_map_creator

## Following line is required for Gazebo7 and over
add_compile_options(-std=gnu++14)
set (CMAKE_CXX_FLAGS "-g -Wall")

#### CMakeLists for other plugin tutorial

#FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(SDF sdformat)
endif()
#include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
#link_directories(${CMAKE_CURRENT_BINARY_DIR} ${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
#add_subdirectory(msgs)
include_directories(${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE-Terrain_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

add_library(QuadrupedMotion SHARED src/QuadrupedMotion.cc src/Motion.cc)
target_link_libraries(QuadrupedMotion ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTUBUF_LIBRARIES})

