# FW-mini 上车实验和记录工作台 v0.1

目标：**不被某一项硬件问题卡住**。每次上车按清单工作，遇到缺设备就记录 `BLOCKED` 或 `SKIPPED`，接着测试另一条算法路线。代码和数据可复现。

## 一行启动（Ubuntu 22.04 / ROS 2 Humble）

```bash
cd ~/fwmini_ws/worktrees/fwmini-can-protocol-humble
bash tools/field_workbench/start.sh
```

直接在 **Ubuntu 22.04** 的浏览器打开 **http://127.0.0.1:8765**。

通过别的电脑访问时，先在那个电脑上开启 SSH 端口转发：

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 yangxuan@192.168.2.116
```

然后在**该电脑**打开相同的 `http://127.0.0.1:8765`。网页服务默认仅绑定远端 `127.0.0.1`，不会暴露到局域网；SSH 连接中断只影响访问，不会修改已保存记录。

如果 8765 被占用，可以在启动前设置 `FWMINI_WORKBENCH_PORT=8777`，并相应修改端口转发。

## 每次上车推荐的 7 步

1. 点 **＋新建上车记录**，填写场地；当天若只是复盘可选历史会话。
2. 「00 环境」点 **只读检查**；检查得到的是诊断证据，**不会自动 PASS**。
3. 「01 CAN」检查接口。如果 can0 没接好，标 `BLOCKED`，记原因（不阻断相机相关工作）。
4. 对独立的「06 相机」检查 USB、图像、CameraInfo；接通后先录 `RGB-D`，再建图。
5. 「05 里程计与 TF」检查；若底盘没接可先使用手持数据 / 离线 rosbag 跑 RTAB-Map，但明确标注数据来源，**不可直接宣称全车闭环通过**。
6. 逐一选择 **RTAB-Map / Cartographer / SLAM Toolbox** 运行，**一次只启动一个 SLAM 后端**；每次把地图数据库、pbstream、posegraph 的完整路径写入证据栏。
7. 点 **导出 Markdown / CSV**，汇总 PASS、FAIL、BLOCKED、SKIPPED 和原因；下次继续，不必清空旧记录。

导航功能 (`Nav2`) 要求测量 footprint、全向里程计、急停/看门狗、完整障碍观测、唯一的 TF 发布者。**页面状态 PASS 绝不会自行改动 `hardware_contract.json` 或启动车辆。**

## 页面能做什么

- 17 个按真实顺序排列的测试点，分 10 个独立方向，支持 `NOT_RUN/PASS/FAIL/BLOCKED/SKIPPED`。
- 支持记录备注、结果、证据路径、时间、每次修改的 `events.jsonl`；可以切换历史会话。
- 只读检查预定义命令：系统/ROS 环境、CAN 链路、相机设备和话题、TF 话题、控制接口、SLAM 软件依赖、磁盘。
- 点击**复制命令**可以把当前阶段命令拷到终端执行；它不会执行建图启动、更不会下发运动控制。
- ROS bag 录制提供 `core`（底盘与 TF）、`rgbd`（彩色/深度）、`slam`（地图与激光）三套白名单，根据真实 ROS 图选话题。如果关键话题不存在，会阻止空录包。点击「停止并保存」触发 SIGINT，等待 rosbag 完整落盘。
- 导出 `.md` 和 `.csv`；保存于每次会话的 `evidence/field_workbench/<时间_随机ID>/`。

## 安全范围

- 服务只监听 loopback，通过 SSH 端口转发访问，写操作有临时随机 token；仅运行固定的只读检查命令，无任意终端命令接口。
- 只读检查 **不能代替真实设备验收**，自动检测到软件包也不意味着算法可用。
- 录包只是订阅话题，但 RGB-D 高码率可能很快占满磁盘；手动检查余量、时间与话题。异常断电后需要检查 rosbag 元数据/索引是否完整。
- 由于实时 CAN 档位码尚未验收，工作台**不提供运动解锁或 `/cmd_vel` 发送按钮**。轮子离地、急停测试需由现场操作员另行执行。
- 单摄像头视场存在侧后盲区，FAST-LIO2 需另加 MID360/IMU；本工作台不改变多导航启动合同中对这些条件的阻断。
- 使用 `Ctrl+C` 正常停止服务，确保 rosbag 已停止。如果进程被强制 `kill -9`，可能造成录包未正常索引；`bag_runs` 会保留启动记录供复盘。

## 离线测试与部署诊断

```bash
cd ~/fwmini_ws/worktrees/fwmini-can-protocol-humble
python3 -m unittest discover -s tools/field_workbench -p 'test_*.py' -v
python3 -m unittest discover -s tools -p test_multi_nav_contracts.py -v
python3 tools/test_fwmini_can_protocol.py
```

无 CAN / 相机时也能打开工作台，做系统诊断和记录，等待现场插入设备即可继续。
