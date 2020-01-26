cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_convertor)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  geometry_msgs
)

find_package(PCL 1.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud_convertor
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_convertor src/point_cloud_con.cpp)
target_link_libraries(point_cloud_convertor ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

