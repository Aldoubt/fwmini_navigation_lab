# Femto Bolt 无底盘手持 RGB-D 建图｜ROS 2 Humble

**实验目标**：仅用 Femto Bolt + RTAB-Map RGB-D visual odometry + RGB-D SLAM，完成第一张室内三维地图，并保存 `rtabmap.db`。此实验不启动 FW-mini 底盘驱动、轮速、AprilTag 或 Nav2。

## P0. 检查安装

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/fwmini_ws/project/install/vendor/setup.bash
source ~/fwmini_ws/project/install/project/setup.bash

ros2 pkg prefix fwmini_perception
ros2 pkg prefix orbbec_camera
ros2 pkg prefix rtabmap_launch
lsusb
lsusb -t
```

若 `orbbec_camera` 缺失，参考 [OrbbecSDK_ROS2 v2-main](https://github.com/orbbec/OrbbecSDK_ROS2) 安装官方驱动。检查 Humble 二进制包：

```bash
sudo apt update
apt-cache policy ros-humble-orbbec-camera
sudo apt install ros-humble-orbbec-camera ros-humble-orbbec-description
```

若该平台的 apt 未提供此包，用单独的 `~/orbbec_ws` 从 `v2-main` 构建；务必按照官方说明安装 udev rules，**不要 sudo ros2 launch**。RTAB-Map 可尝试：`sudo apt install ros-humble-rtabmap-ros`。

## P1. 只启动相机（终端 A）

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/fwmini_ws/project/install/vendor/setup.bash
source ~/fwmini_ws/project/install/project/setup.bash

ros2 launch fwmini_perception femto_bolt.launch.py \
  enable_color:=true enable_depth:=true \
  depth_registration:=true enable_point_cloud:=true \
  publish_tf:=true
```

此时不运行 `fwmini_bringup description.launch.py`，确保 TF 仅由相机驱动维护；因没有底盘，**本次不应存在 `base_link`**。本封装增加 `fwmini` 命名空间，理论上话题为 `/fwmini/camera/...`。实际以查询结果为准。

## P2. 验证话题/时钟/TF（终端 B）

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/fwmini_ws/project/install/vendor/setup.bash
source ~/fwmini_ws/project/install/project/setup.bash
ros2 topic list | grep -E 'color|depth|points|tf'
ros2 topic hz /fwmini/camera/color/image_raw
ros2 topic hz /fwmini/camera/depth/image_raw
ros2 topic echo --once /fwmini/camera/color/image_raw --field header
ros2 topic echo --once /fwmini/camera/depth/image_raw --field header
ros2 topic echo --once /fwmini/camera/color/camera_info
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

检查 color/depth 有连续图像、单位/尺寸合理、时间戳随墙钟递增；相同光学坐标系下 RGB 和注册后的 Depth 应可对齐。若 frame 名不是 `camera_color_optical_frame`，按图像 header 修正；别凭空发布相机外参。

在 RViz 用相机实际光学 frame 作为 Fixed Frame，添加 `PointCloud2`，topic `/fwmini/camera/depth/points`，确认距离与场景几何关系合理。

## P3. RTAB-Map 建图（终端 C）

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/fwmini_ws/project/install/vendor/setup.bash
source ~/fwmini_ws/project/install/project/setup.bash
mkdir -p ~/fwmini_ws/maps

ros2 launch rtabmap_launch rtabmap.launch.py \
  depth:=true \
  visual_odometry:=true \
  rgbd_sync:=true approx_rgbd_sync:=true \
  rgb_topic:=/fwmini/camera/color/image_raw \
  depth_topic:=/fwmini/camera/depth/image_raw \
  camera_info_topic:=/fwmini/camera/color/camera_info \
  frame_id:=camera_link \
  qos:=2 \
  database_path:=$HOME/fwmini_ws/maps/femto_handheld_001.db \
  rtabmap_viz:=true rviz:=false
```

`frame_id:=camera_link` 的前提是相机驱动确实提供 `camera_link -> camera_color_optical_frame`。若相机有不同名称，根据 `ros2 topic echo --once ... --field header` 和 TF 检查调整 frame_id；如果无法获得相机 base TF，可暂时使用 RGB 图像的 optical frame 作为 `frame_id` 来定位问题（地图方向会随 optical 轴约定变化）。

`qos:=2` 使用 Best Effort 订阅，可以与常见传感器 QoS 兼容；若无数据，检查 `ros2 topic info -v`。官方 RTAB-Map Humble/ROS2 launch 支持这些参数，但实际部署还需确认当前安装的包版本：`ros2 launch rtabmap_launch rtabmap.launch.py --show-args`。

## P4. 观察和验收

```bash
ros2 topic list | grep rtabmap
ros2 topic hz /rtabmap/odom
ros2 run tf2_ros tf2_echo map camera_link
```

手持移动相机，保持基本水平，先在纹理丰富的室内场景沿 2–3 米路线慢速移动，环绕有特征的桌子，再回到起点观察闭环优化。避免对着纯白墙、玻璃、强反光面、阳光直射区域或快速甩动相机。首次只做 1–3 分钟；能看到持续位姿变化、实时 RGB-D cloud/graph、回到起点有回环机会，就算通过建图最小验证。

建图后 Ctrl+C 正常结束 RTAB-Map 进程：

```bash
ls -lh ~/fwmini_ws/maps/femto_handheld_001.db
```

**禁止**首次测试使用 `--delete_db_on_start` 反复覆盖已有数据库。每次新实验更换文件名。此时得到的是三维 RTAB-Map 数据库，不等于 Nav2 可用二维 PGM 地图；后者需要水平参考系、地面分割与机器人足迹/可通过性处理。

## 故障定位顺序

- 未找到 `orbbec_camera`：先按 Orbbec 官方教程安装源码或二进制驱动并配置 udev。
- 未找到 `rtabmap_launch`：安装 `ros-humble-rtabmap-ros`。
- camera 有图像、RTAB-Map 无位姿：先查 registered depth / RGB 分辨率匹配、CameraInfo、时间戳及 optical TF。
- `Could not get transform camera_link`：使用实际的相机 frame id，检查 `publish_tf:=true`，不要同时启动机器人描述的重复 TF。
- `No data received`：查询三个话题、QoS、`rgbd_sync` 输出和传输速率。
- `Odometry lost`：放慢运动、增加图像纹理/光照、减少反射面，考虑降低分辨率/帧率而非凭空加入车轮里程计。
