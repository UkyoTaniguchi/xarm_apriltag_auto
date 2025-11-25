# xarm_apriltag_auto

複数のRealSenseカメラを自動起動し、統合点群を可視化するROSパッケージ。  
AprilTagを用いた外部キャリブレーションにも対応。

## 概要

このリポジトリは、産業用ロボットセルにおける複数3Dカメラの同期・統合を目的としています。 複数RealSenseカメラを同時起動し、キャリブレーションを行うことで単一座標系（`world`）に統合された点群を可視化します。  

## 機能
- 複数RealSense（D435i/D405など）の自動起動  
- AprilTagによる外部キャリブレーション  
- TFを用いたワールド座標への統一変換  
- `PointCloud2`トピックの統合配信 (`/merged/points`)  

## 動作環境
- Ubuntu 22.04  
- ROS1  
- Python 3.10+  
- Intel RealSense SDK 2.55+  
- `realsense2_camera` ROS2 package  

## セットアップ

```bash
# クローン
git clone git@github.com:riku030502/xarm_apriltag_auto.git
cd <your_ws>
catkin build
source devel/setup.bash
```

## 実行手順

### 1.RealSenseの`serial_number`を確認
```bash
rs-enumerate-devices -S
```
出力された番号を`demo_self_recalibration.launch`に記載  
ハンドアイカメラはcam1に固定してください．  
### 2.キャリブレーション
以下のコマンドを実行しキャリブレーションを行ってください  
xarm6とハンドアイカメラ，周囲固定カメラ，Rvizの起動  
```bash
roslaunch xarm_apriltag_demo demo_self=recalibration.launch.py
```
タスク開始＆カメラ監視開始
```bash
roslaunch xarm_arpriltag_demo save_multi_tag_pose.launch
```
ENTERを押すとタスクが実行される．

### 3.Realsenseの同時起動
```bash
roslaunch xarm_apriltag_demo boot_multi_cam.launch
```