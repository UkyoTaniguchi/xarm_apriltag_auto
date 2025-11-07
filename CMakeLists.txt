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
## === 実行ファイルのインストール ===
## ==============================
catkin_install_python(PROGRAMS
  src/camera_monitor/imu_icp_monitor.py
  src/camera_monitor/camera_supervisor.py
  src/camera_monitor/roi_selector.py
  src/robot/move_to_tag.py
  src/robot/robot_task_manager.py
  src/robot/pick_and_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# --- 実行ファイルなしなのでここは削除 ---
# add_executable(...)
# target_link_libraries(...)
# add_dependencies(...)

# install も不要なら削除可
