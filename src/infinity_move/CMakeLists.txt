cmake_minimum_required(VERSION 2.8.3)
project(infinity_move)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(infinity src/infinity.cpp)
target_link_libraries(infinity ${catkin_LIBRARIES})
# add_dependencies(infinity )
