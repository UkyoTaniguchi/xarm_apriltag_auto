cmake_minimum_required(VERSION 3.0.2)
project(xarm_apriltag_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    moveit_ros_planning_interface
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_ros
    pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# --- 実行ファイルなしなのでここは削除 ---
# add_executable(...)
# target_link_libraries(...)
# add_dependencies(...)

# install も不要なら削除可
