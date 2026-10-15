# FW-mini｜Ubuntu 22.04 + ROS 2 Humble 现场部署清单（2026-10-16）

本文件是部署步骤，不表示已经通过实车验证。**第一天先让底盘受控低速运动并采集 Femto Bolt 数据；RTAB-Map + Nav2 尚未集成进仓库。**

## 1. 现有状态

- 八个 ROS 2 包：fwmini_description、fwmini_interfaces、fwmini_bringup、fwmini_chassis_adapter、fwmini_odom_adapter、fwmini_perception、apriltag_detector_adapter、tag_follow_controller。
- Windows 已执行静态结构测试和 C++ 适配器核心测试：PASS。
- ROS2 Humble 构建、厂家 CAN 驱动、实车运动和相机实机：NOT_RUN。
- 厂家 ROS 2 的 yhs_can_control 和 yhs_can_interfaces 源码已经随 GitHub 仓库一起提供于 `vendor/FW-mini-ros2`；厂商源代码未发现清晰的公开再分发许可，请立即将仓库设为 Private 或确认许可。

## 2. 克隆工程

    source /opt/ros/humble/setup.bash
    mkdir -p ~/fwmini_ws
    cd ~/fwmini_ws
    git clone https://github.com/Aldoubt/fwmini_navigation_lab.git project
    mkdir -p vendor/FW-mini-ros2

从 Windows 原始目录复制这两个 ROS 2 包到 ~/fwmini_ws/project/vendor/FW-mini-ros2：

    Fw-mini导航中间件/01_ros通讯协议解析包/FW-mini-ros2/yhs_can_interfaces
    Fw-mini导航中间件/01_ros通讯协议解析包/FW-mini-ros2/yhs_can_control

不要把厂家源代码提交到公开仓库；不要修改 ROS1 生产环境的底盘驱动。

## 3. 依赖与无运动编译测试

    sudo apt update
    sudo apt install -y python3-colcon-common-extensions python3-rosdep       ros-humble-apriltag-msgs ros-humble-apriltag-ros ros-humble-image-proc       ros-humble-xacro ros-humble-robot-state-publisher ros-humble-tf2-tools       ros-humble-rtabmap-ros ros-humble-navigation2 ros-humble-nav2-bringup

    cd ~/fwmini_ws/project
    rosdep install --from-paths src ~/fwmini_ws/project/vendor/FW-mini-ros2/yhs_can_interfaces       --ignore-src --rosdistro humble -r -y
    export FWM_VENDOR_ROS2_DIR="$HOME/fwmini_ws/vendor/FW-mini-ros2"
    bash tools/test_ros_adapters.sh

上述测试脚本只编译厂家消息接口和本工程适配器、运行测试，不启动 CAN 运动。若失败，记录第一条构建错误。真实厂家 yhs_can_control 需单独在 Humble 环境编译核验；旧版源码存在依赖/CMake 等已知问题，见 docs/vendor_middleware_review.md。

编译成功后在新的终端中加载：

    source /opt/ros/humble/setup.bash
    source ~/fwmini_ws/project/install/vendor/setup.bash
    source ~/fwmini_ws/project/install/project/setup.bash

## 4. 厂家 CAN 驱动：先只看反馈

检查真实 CAN 接口、比特率、底盘控制模式及厂家启动配置：

    ip -details link show
    ls ~/fwmini_ws/project/vendor/FW-mini-ros2/yhs_can_control/launch
    cat ~/fwmini_ws/project/vendor/FW-mini-ros2/yhs_can_control/params/cfg.yaml

厂家驱动成功启动后，检查：

    ros2 topic list | grep -E 'ctrl|chassis|wheel|ultrasonic'
    ros2 topic info -v /chassis_info_fb
    ros2 topic hz /chassis_info_fb

只有持续、时间戳有效的 yhs_can_interfaces/msg/ChassisInfoFb 才能作为轮速适配器输入。先确认消息和方向，不要自动发送运动命令。

## 5. 里程计与底盘低速试车

终端 A（加载上述 setup）：

    ros2 launch fwmini_odom_adapter adapter.launch.py

终端 B 检查：

    ros2 topic hz /wheel/odom
    ros2 topic echo --once /wheel/odom
    ros2 run tf2_ros tf2_echo odom base_link
    ros2 topic echo --once /diagnostics

前提：明确控制档位 gear=0 是否允许低速运动，确认厂家软件急停、物理急停和遥控接管有效，确保只有一个 CAN 控制发布源，使用架空车轮或隔离场地。

终端 A 另开：

    ros2 launch fwmini_chassis_adapter adapter.launch.py

确认 /ctrl_cmd 默认零速后，手动启用：

    ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: true}"

终端 B 以 10 Hz 连续发送前进 0.05 m/s 测试命令（准备好立即 Ctrl+C）：

    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist       "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

停止终端 B 后立即禁用：

    ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: false}"

Adapter 采用默认禁用、0.2 秒指令超时及 50 Hz 输出零速门控；它不是经过认证的安全控制器。任何方向错误、反馈中断、遥控接管异常，立即使用物理急停。

## 6. Femto Bolt 单独验证

安装奥比中光官方 OrbbecSDK_ROS2 的 Humble/Femto Bolt 驱动后：

    ros2 pkg prefix orbbec_camera
    ros2 launch fwmini_perception femto_bolt.launch.py       enable_color:=true enable_depth:=true enable_point_cloud:=true depth_registration:=true
    ros2 topic list | grep -E 'color|depth|points|gyro|accel'

本工程相机封装设置了 fwmini 命名空间；话题可能位于 /fwmini/camera/...，以运行时实际 topic 列表为准。未经实测的相机外参不能用来启动自动导航。

## 7. 验收顺序

G0 — ROS2 环境和厂家代码到位；
G1 — colcon 构建及 Adapter 测试通过；
G2 — CAN 和 /chassis_info_fb 稳定；
G3 — /wheel/odom 正常、急停生效、底盘可控低速移动；
G4 — Femto Bolt 彩色/深度/点云可用；
G5 — 后续单独开发 RTAB-Map 建图/定位、Nav2 与 AprilTag 自动初始化。

**明天先做 G0–G4，不要在 CAN 尚未稳定时同时调 Nav2。**
