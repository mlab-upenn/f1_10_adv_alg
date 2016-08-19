cmake_minimum_required(VERSION 2.8.3)
project(trajectory_cost)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  nav_msgs
  map_msgs
  cmake_modules
)

add_message_files( 
  FILES 
  TrajectoryID.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
 INCLUDE_DIRS include
# LIBRARIES trajectory_cost
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs map_msgs message_runtime
  DEPENDS system_lib
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable(trajcost src/trajectory_cost.cpp)
target_link_libraries(trajcost ${catkin_LIBRARIES})
add_dependencies(trajcost trajectory_brain ${catkin_EXPORTED_TARGETS})
