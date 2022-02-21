cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
## Package Name
project(crawler_robot)

#FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
find_package(Boost REQUIRED COMPONENTS system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(SDF sdformat)
endif()

add_compile_options(-std=gnu++17)
set (CMAKE_CXX_FLAGS "-g -Wall")

# Depend on system install of Gazebo and SDFormat
#find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}" )
#MESSAGE( STATUS ">>>>>> PROJECT_NAME : " ${PROJECT_NAME} )
#MESSAGE( STATUS ">>>>>> CATKIN_DEVEL_PREFIX : " ${CATKIN_DEVEL_PREFIX} )
#MESSAGE( STATUS ">>>>>> PROJECT_BINARY_DIR : " ${PROJECT_BINARY_DIR} )
#MESSAGE( STATUS ">>>>>> CMAKE_CURRENT_LIBRARY_PATH : " ${CMAKE_CURRENT_LIBRARY_PATH} )
#MESSAGE( STATUS ">>>>>> LIBRARY_OUTPUT_PATH : " ${LIBRARY_OUTPUT_PATH} )
#MESSAGE( STATUS ">>>>>> EXECUTABLE_OUTPUT_PATH : " ${EXECUTABLE_OUTPUT_PATH} )
#MESSAGE( STATUS ">>>>>> CMAKE_LIBRARY_PATH : " ${CMAKE_LIBRARY_PATH} )
#MESSAGE( STATUS ">>>>>> CMAKE_CURRENT_BINARY_DIR : " ${CMAKE_CURRENT_BINARY_DIR} )
#You can find more variables in https://cmake.org/Wiki/CMake_Useful_Variables/Logging_Useful_Valiables

################################################
# Build flipper control message
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/msgs)
#link_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(src/msgs)

###############################################
# Build the Crawler-Flipper Arm plugin
add_library(crawler_flipper SHARED src/crawler_flipper.cc)
target_link_libraries(crawler_flipper flipper_control_msgs ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(crawler_flipper flipper_control_msgs)

###############################################
# Build the robot controller
add_executable(teleop_crawler_robot src/teleop_crawler_robot.cc)
target_link_libraries(teleop_crawler_robot flipper_control_msgs ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(teleop_crawler_robot flipper_control_msgs ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} "-pthread")
add_dependencies(teleop_crawler_robot flipper_control_msgs)
