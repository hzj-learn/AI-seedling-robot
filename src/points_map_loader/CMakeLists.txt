cmake_minimum_required(VERSION 3.0.2)
project(points_map_loader)

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  velodyne_pointcloud
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")


add_executable(points_map_loader src/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} ${CURL_LIBRARIES} ${PCL_IO_LIBRARIES})
add_dependencies(points_map_loader ${catkin_EXPORTED_TARGETS})

