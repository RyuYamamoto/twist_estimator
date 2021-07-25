cmake_minimum_required(VERSION 3.0.2)
project(twist_estimator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES twist_estimator
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_estimator_node src/twist_estimator.cpp src/twist_estimator_node.cpp)
target_link_libraries(twist_estimator_node ${catkin_LIBRARIES})
