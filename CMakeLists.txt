cmake_minimum_required(VERSION 2.8.3)
project(agv_path_smoothing)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
)

find_package(Eigen3 REQUIRED)
#if(Eigen3_FOUND)
  #message("Eigen3 was found!")
  #message("Eigen3 Headers DIRECTORY: " ${EIGEN3_INCLUDE_DIR})
#endif()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES agv_path_smoothing
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(path_viewer test/path_viewer.cpp src/Curve_common.cpp)

target_link_libraries(path_viewer
   ${catkin_LIBRARIES}
)

