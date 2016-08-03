cmake_minimum_required(VERSION 2.8.3)
project(trajectory_cost)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  map_msgs
  cmake_modules
  fake_trajectory
  message_generation
)

add_message_files(
  FILES
  TrajectoryCostVector.msg
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES trajectory_cost
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs map_msgs message_runtime fake_trajectory
  DEPENDS system_lib
)

include_directories( include/
 ${catkin_INCLUDE_DIRS}
)

add_executable(trajcost src/trajectory_cost.cpp)
target_link_libraries(trajcost ${catkin_LIBRARIES})
add_dependencies(trajcost fake_trajectory ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
