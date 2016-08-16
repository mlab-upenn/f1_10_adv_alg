cmake_minimum_required(VERSION 2.8.3)
project(dynamic_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  visualization_msgs
  roscpp
  tf
  cmake_modules
  random_numbers
)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_obstacle_detection
  CATKIN_DEPENDS visualization_msgs nav_msgs tf
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")

include_directories( include/
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(dynamic_obstacle_detection src/project_obj.cpp)
target_link_libraries(dynamic_obstacle_detection ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
