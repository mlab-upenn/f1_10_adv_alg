cmake_minimum_required(VERSION 2.8.3)
project(fake_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  random_numbers
)

# Generate messages in the 'msg' folder
add_message_files( DIRECTORY msg FILES 
  TrajectoryVector.msg
  TrajectorySims.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES fake_trajectory
  CATKIN_DEPENDS roscpp geometry_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(faketraj src/fake_traj_pub.cpp)
target_link_libraries(faketraj ${catkin_LIBRARIES})
