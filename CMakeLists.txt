cmake_minimum_required(VERSION 2.8.3)
project(agv_path_smoothing)

add_compile_options(-std=c++11)

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp 
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
   src/Curve_common.cpp
   src/Curve_fitting.cpp
)

add_executable(path_viewer test/path_viewer.cpp src/Curve_common.cpp)

target_link_libraries(path_viewer
   ${catkin_LIBRARIES}
)

add_executable(path_viewer_spline test/path_viewer_spline.cpp src/Curve_common.cpp)

target_link_libraries(path_viewer_spline
   ${catkin_LIBRARIES}
)

add_executable(curve_fitting_viewer test/curve_fitting_viewer.cpp src/Curve_common.cpp src/Curve_fitting.cpp)

target_link_libraries(curve_fitting_viewer
   ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

MESSAGE(STATUS "This is include dir: " ${catkin_INCLUDE_DIRS})
MESSAGE(STATUS "This is catkin library: " ${catkin_LIBRARIES})
