cmake_minimum_required(VERSION 2.8.3)
project(cm_setup_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cm_setup_tf
#  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cm_setup src/cm_broadcast.cpp)
target_link_libraries(cm_setup ${catkin_LIBRARIES})
