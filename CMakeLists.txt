cmake_minimum_required(VERSION 2.8.3)
project(nav_collision_warning)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${eigen3_INCLUDE_DIRS}
)

add_library(find_nearest_obstacle src/find_nearest_obstacle.cpp)
add_dependencies(find_nearest_obstacle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_nearest_obstacle ${catkin_LIBRARIES} )

add_executable(nav_collision_warning src/nav_collision_warning.cpp)
target_link_libraries(nav_collision_warning ${catkin_LIBRARIES} find_nearest_obstacle)
add_dependencies(nav_collision_warning ${catkin_EXPORTED_TARGETS})

