cmake_minimum_required(VERSION 3.0.2)
project(xarm_apriltag_demo)

## ==============================
## === 必要パッケージの検出 ===
## ==============================
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

## ==============================
## === catkin パッケージ設定 ===
## ==============================
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

## ==============================
## === インクルード設定 ===
## ==============================
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## ==============================
## === PointCloud Merge Node ===
## ==============================
add_executable(merge_pointcloud src/merge_pointcloude.cpp)

target_link_libraries(merge_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(merge_pointcloud ${catkin_EXPORTED_TARGETS})

## ==============================
## === インストール設定 ===
## ==============================
install(TARGETS merge_pointcloud
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
