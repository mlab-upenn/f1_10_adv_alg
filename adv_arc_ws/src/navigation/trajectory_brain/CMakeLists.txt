cmake_minimum_required(VERSION 2.8.3)
project(trajectory_brain)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  message_generation
  trajectory_cost
)

################################################
## Find OpenMP in order to parallelize loops  ##
################################################

OPTION (USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(DIRECTORY msg
  FILES
  TrajectoryVector.msg
  TrajectorySims.msg
  drive_param.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
  nav_msgs
)

#find_library(rbfa NAMES "rbfa1" PATHS "lib")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs nav_msgs geometry_msgs tf trajectory_cost
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_brain src/trajectory_brain.cpp)
target_link_libraries(trajectory_brain ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/libs/libtrajgen.a)
add_dependencies(trajectory_brain ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(trajectory_brain trajectory_cost ${catkin_EXPORTED_TARGETS})

add_executable(twist_gen src/state_generator.cpp)
target_link_libraries(twist_gen ${catkin_LIBRARIES})
