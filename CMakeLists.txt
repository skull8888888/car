cmake_minimum_required(VERSION 2.8.3)
project(car)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(ndt_omp_DIR /home/team7/catkin_ws/src/ndt_omp/)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  message_generation
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  ndt_omp
)

find_package(PCL REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES a_star_lib
  # CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
	${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
	# include
)

add_executable(pcl_parser_node src/pcl_parser_node.cpp)
add_dependencies(pcl_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_parser_node
  ${catkin_LIBRARIES}
)


add_executable(scan_matching_localizer_node src/scan_matching_localizer_node.cpp)
add_dependencies(scan_matching_localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_matching_localizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)