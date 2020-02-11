cmake_minimum_required(VERSION 2.8.3)
project(agv_path_smoothing)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES agv_path_smoothing
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(path_viewer test/path_viewer.cpp)

target_link_libraries(path_viewer
   ${catkin_LIBRARIES}
)

