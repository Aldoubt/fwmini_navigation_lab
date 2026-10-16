#!/usr/bin/env python3
"""FW-mini field-workbench: offline-first progress log and passive diagnostics.

Bound to loopback, no shell evaluation of browser input, never sends motor commands.
Python stdlib only; run after sourcing ROS2 Humble for live read-only probes.
"""

import argparse
import csv
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
import re
from pathlib import Path
import secrets
import shutil
import signal
import socket
import subprocess
import threading
import time
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
STATUSES = {"NOT_RUN", "PASS", "FAIL", "BLOCKED", "SKIPPED"}
LANES = ["环境", "CAN", "底盘", "传感器", "TF", "RTAB-Map", "Cartographer", "SLAM Toolbox", "Nav2", "FAST-LIO2"]
TASKS = [
    dict(id="p00_environment", lane="环境", title="00 · 确认系统与工作区", target="确认 Humble、当前分支、软件依赖，建立本次记录。", criteria="ROS 2 Humble 可用；Git 分支与测试日期明确。", probe="system", command="source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E 'nav2|rtabmap|cartographer'"),
    dict(id="p01_can_interface", lane="CAN", title="01 · CAN 接口和被动反馈", target="只读查看 can0、比特率、设备反馈；不发送控制命令。", criteria="can0 UP 且接收帧正常；确认 0x18C4D1EF 的 CRC、计数器与频率。", probe="can", command="ip -details link show can0\ncandump -L can0  # Ctrl+C 停止"),
    dict(id="p02_can_units", lane="CAN", title="02 · 通讯倍率及方向", target="对照原始 CAN、/chassis_info_fb 的 vx/vy/角速度。", criteria="vx、vy 均为 m/s，角速度由 °/s 转 rad/s 仅一次；不能出现 10 倍误差。", command="python3 tools/test_fwmini_can_protocol.py\nros2 topic echo /chassis_info_fb --once"),
    dict(id="p03_cmd_watchdog", lane="底盘", title="03 · 指令门控、急停及掉线", target="轮子离地、有人守急停；确认 50Hz 刷新及 0.2s 超时停车。", criteria="运动使能默认关闭；断开 cmd_vel 后归零、急停有效；档位码已核实。", probe="chassis", command="ros2 topic info -v /ctrl_cmd\nros2 service list | grep fwmini"),
    dict(id="p04_motion", lane="底盘", title="04 · 前进 / 横移 / 原地旋转", target="先轮子离地，再空旷地面；逐项记录命令与真实运动。", criteria="vx、vy、wz 符号和方向正确，停车距离与响应时间有记录。", command="# 人工控制；不要由网页直接下发运动命令。\nros2 bag record /cmd_vel /ctrl_cmd /chassis_info_fb /wheel/odom /tf"),
    dict(id="p05_odom_tf", lane="TF", title="05 · 全向里程计与 TF", target="确保唯一的 odom→base_link，位置随横移正确更新。", criteria="/wheel/odom 速率稳定；横移 y 正确；无第二个 odom TF 发布者。", probe="tf", command="ros2 topic hz /wheel/odom\nros2 run tf2_ros tf2_echo odom base_link"),
    dict(id="p06_femto_usb", lane="传感器", title="06 · Femto Bolt 连接和出图", target="检查 USB、RGB/Depth、CameraInfo，先不用底盘移动。", criteria="图像持续刷新且无周期性 fps=0 / 掉帧。", probe="camera", command="lsusb | grep -iE 'orbbec|2bc5'\nros2 topic list -t | grep -E 'image|camera_info'"),
    dict(id="p07_rgbd_sync", lane="传感器", title="07 · RGB-D 配准与时间同步", target="核对深度和彩色内参、深度对齐、时间戳及 TF。", criteria="RGB-D 画面对应、时间接近、光学坐标系正确；录包证据保留。", command="ros2 topic hz /fwmini/camera/color/image_raw\nros2 topic hz /fwmini/camera/depth/image_raw"),
    dict(id="p08_urdf_footprint", lane="TF", title="08 · URDF 外参 / 机器人轮廓", target="测量旋转中心、相机姿态、机器人最大外包络。", criteria="TF 树唯一且连续；footprint 包含传感器突出部分，数据人工测量。", probe="tf", command="ros2 run tf2_ros tf2_echo base_link camera_link\nros2 topic echo /tf_static --once"),
    dict(id="p09_rtabmap_map", lane="RTAB-Map", title="09 · RTAB-Map RGB-D 建图", target="先不启动 Nav2；只用轮速里程计 + RGB-D 构建地图。", criteria="建图无持续跳变，保存 .db 和 rosbag；记录有效帧数与轨迹。", probe="packages", command="ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=rtabmap navigation:=false"),
    dict(id="p10_rtabmap_reloc", lane="RTAB-Map", title="10 · RTAB-Map 冷启动定位", target="重启节点加载同一原生 .db；在已知位置验证重复定位。", criteria="地图加载成功并能重定位；偏差、耗时和失败案例可复现。", command="ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=rtabmap mode:=localization database_path:=/ABS/map.db navigation:=false"),
    dict(id="p11_scan", lane="传感器", title="11 · Depth / PointCloud 转 LaserScan", target="固定地面、墙壁、近障碍物高度与视场边界。", criteria="/scan 的角度、距离、坐标与实体匹配；记录盲区，不当 360° 激光。", probe="camera", command="ros2 topic hz /scan\nros2 topic echo /scan --once"),
    dict(id="p12_cartographer", lane="Cartographer", title="12 · Cartographer 2D 建图", target="同一底盘和 /scan、/wheel/odom 做单独建图。", criteria="地图、map→odom 合法；保存 .pbstream；不能与其他 SLAM 同开。", probe="packages", command="ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=cartographer_2d navigation:=false"),
    dict(id="p13_slam_toolbox", lane="SLAM Toolbox", title="13 · SLAM Toolbox 建图和重载", target="同一 LaserScan 进行独立的建图、序列化和加载。", criteria="原生 posegraph 可保存/重载；连续跟踪正常。", probe="packages", command="ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=slam_toolbox navigation:=false"),
    dict(id="p14_nav2_preflight", lane="Nav2", title="14 · Nav2 启动前门禁", target="真实 footprint、costmap、单控制源、急停；未验收绝不放行运动。", criteria="硬件 contract 全部经过人工验证；局部障碍、盲区策略满足现场要求。", probe="packages", command="cat src/fwmini_multi_nav_bringup/config/hardware_contract.json\nros2 pkg prefix nav2_bringup"),
    dict(id="p15_nav2_slow", lane="Nav2", title="15 · 低速闭环 / DWB / MPPI", target="先验证低速前进与转弯，最后开放横移和复杂轨迹。", criteria="轨迹跟踪、停车及绕障有实测误差；有录包可对比。", command="# 仅通过带实测参数和人工运动许可的 Nav2 启动链。\n# 本网页不提供一键驾驶。"),
    dict(id="p16_fastlio_optional", lane="FAST-LIO2", title="16 · MID360 + FAST-LIO2（可跳过）", target="后续加装 MID360/IMU 时接入算法，不阻塞视觉导航主线。", criteria="外参、时间同步、/map 与 /odom 责任明确；3D 重定位单独验收。", probe="packages", command="# 需要 MID360 + IMU，不能仅靠 Femto Bolt 启动 FAST-LIO2。"),
]
TASK_BY_ID = {task["id"]: task for task in TASKS}
BAG_PROFILES = {
    "core": ["/tf", "/tf_static", "/wheel/odom", "/chassis_info_fb", "/diagnostics", "/cmd_vel", "/cmd_vel_nav", "/smoothed_cmd_vel", "/ctrl_cmd"],
    "rgbd": ["/tf", "/tf_static", "/wheel/odom", "/fwmini/camera/color/image_raw", "/fwmini/camera/depth/image_raw", "/fwmini/camera/color/camera_info", "/fwmini/camera/depth/camera_info", "/camera/color/image_raw", "/camera/depth/image_raw", "/camera/color/camera_info", "/camera/depth/camera_info"],
    "slam": ["/tf", "/tf_static", "/wheel/odom", "/scan", "/map", "/rtabmap/info", "/rtabmap/mapData", "/chassis_info_fb", "/diagnostics"],
}

def now():
    return datetime.now().astimezone().isoformat(timespec="seconds")

def limited(value, size=1200):
    return str(value or "").replace("\x00", "").strip()[:size]

def run_readonly(argv, timeout=6):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                                stdin=subprocess.DEVNULL, check=False)
        output = (result.stdout + ("\n[stderr] " + result.stderr if result.stderr else ""))[:11000]
        return f"$ {' '.join(argv)}\n[exit={result.returncode}]\n{output}"
    except subprocess.TimeoutExpired:
        return f"$ {' '.join(argv)}\n[TIMEOUT {timeout}s: ROS 图响应慢或节点不可达]"
    except OSError as exc:
        return f"$ {' '.join(argv)}\n[UNAVAILABLE] {exc}"

def probe(probe_id):
    if probe_id not in {"system", "can", "camera", "tf", "chassis", "packages", "storage"}:
        raise ValueError("unknown read-only diagnostic")
    report = [f"READ ONLY · {probe_id} · {now()}", "不会发送 CAN 帧或 cmd_vel。"]
    if probe_id == "system":
        report += [Path("/etc/os-release").read_text()[:650], f"ROS_DISTRO={os.getenv('ROS_DISTRO', '<not sourced>')}",
                   run_readonly(["git", "-C", str(PROJECT), "status", "-sb"]),
                   run_readonly(["ros2", "pkg", "prefix", "nav2_bringup"])]
    if probe_id == "can":
        report += [run_readonly(["ip", "-details", "link", "show", "can0"]),
                   "提示：接口在线不代表车体控制已验收；请另录 candump 校验实际报文。"]
    if probe_id == "camera":
        report += [run_readonly(["lsusb"]), run_readonly(["ros2", "topic", "list", "-t"], 7)]
    if probe_id == "tf":
        report += [run_readonly(["ros2", "topic", "info", "-v", "/tf"], 7),
                   run_readonly(["ros2", "topic", "info", "-v", "/wheel/odom"], 7),
                   "注意：Topic 有发布者不等于 TF 方向、外参或时间戳正确。"]
    if probe_id == "chassis":
        report += [run_readonly(["ros2", "topic", "info", "-v", "/ctrl_cmd"], 7),
                   run_readonly(["ros2", "service", "list"], 7),
                   "只检查接口存在；未调用 enable_motion / 未发速度。"]
    if probe_id == "packages":
        for package in ("nav2_bringup", "rtabmap_slam", "cartographer_ros", "slam_toolbox", "depthimage_to_laserscan", "pointcloud_to_laserscan"):
            report.append(run_readonly(["ros2", "pkg", "prefix", package], 4))
    if probe_id == "storage":
        usage = shutil.disk_usage(PROJECT)
        report += [f"磁盘可用 {usage.free / (1024**3):.1f} GiB", run_readonly(["df", "-h", str(PROJECT)])]
    return "\n\n".join(report)

class Records:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.recording = None
        self.recording_log = None
        marker = self.root / "active_session.txt"
        if marker.is_file() and (self.root / marker.read_text().strip() / "session.json").is_file():
            self.active = marker.read_text().strip()
        else:
            self.new_session("首轮 FW-mini 上车验收", "未填写")

    def folder(self):
        return self.root / self.active

    def state(self):
        with self.lock:
            return json.loads((self.folder() / "session.json").read_text(encoding="utf-8"))

    def _save(self, record):
        dest = self.folder() / "session.json"
        tmp = self.folder() / ".session.new"
        tmp.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.replace(tmp, dest)

    def _event(self, kind, **details):
        with (self.folder() / "events.jsonl").open("a", encoding="utf-8") as fd:
            fd.write(json.dumps({"time": now(), "event": kind, **details}, ensure_ascii=False) + "\n")

    def new_session(self, name, place):
        with self.lock:
            if self.recording is not None and self.recording.poll() is None:
                raise ValueError("请先停止本次 ROS bag 录制，再创建新的会话")
            identifier = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + secrets.token_hex(3)
            folder = self.root / identifier
            folder.mkdir(parents=True, exist_ok=False)
            self.active = identifier
            record = dict(id=identifier, title=limited(name, 80) or "FW-mini 上车验收",
                          place=limited(place, 80), started=now(), steps={}, last_probe={}, bag_runs=[])
            self._save(record)
            (self.root / "active_session.txt").write_text(identifier, encoding="ascii")
            self._event("session_started", title=record["title"], place=record["place"])
            return record

    def sessions(self):
        result = []
        for folder in sorted(self.root.iterdir(), reverse=True):
            metadata = folder / "session.json"
            if folder.is_dir() and metadata.is_file():
                try:
                    record = json.loads(metadata.read_text(encoding="utf-8"))
                    result.append(dict(id=folder.name, title=record["title"],
                                       place=record["place"], started=record["started"],
                                       passed=sum(x.get("status") == "PASS" for x in record["steps"].values())))
                except (ValueError, KeyError, OSError):
                    continue
        return result

    def open_session(self, identifier):
        with self.lock:
            if self.recording is not None and self.recording.poll() is None:
                raise ValueError("录包进行中，请先停止录包")
            if not isinstance(identifier, str) or not re.fullmatch(r"[0-9]{8}_[0-9]{6}_[0-9a-f]{6}", identifier):
                raise ValueError("invalid session ID")
            target = self.root / identifier / "session.json"
            if not target.is_file():
                raise ValueError("session not found")
            self.active = identifier
            (self.root / "active_session.txt").write_text(identifier, encoding="ascii")
            self._event("session_opened")
            return self.state()

    def set_status(self, step, status, note="", evidence=""):
        if step not in TASK_BY_ID or status not in STATUSES:
            raise ValueError("unknown test step or status")
        with self.lock:
            record = self.state()
            record["steps"][step] = dict(status=status, note=limited(note),
                                         evidence=limited(evidence, 400), time=now())
            self._save(record)
            self._event("status", step=step, **record["steps"][step])
            return record["steps"][step]

    def store_probe(self, probe_id, report):
        if probe_id not in {t.get("probe") for t in TASKS} | {"storage"}:
            raise ValueError("unknown probe")
        with self.lock:
            filename = f"probe_{datetime.now().strftime('%H%M%S')}_{probe_id}_{secrets.token_hex(2)}.txt"
            (self.folder() / filename).write_text(report, encoding="utf-8")
            record = self.state()
            record["last_probe"][probe_id] = dict(time=now(), filename=filename)
            self._save(record)
            self._event("probe", probe=probe_id, file=filename)
        return filename

    def bag_state(self):
        if self.recording is None:
            return None
        return {"active": self.recording.poll() is None,
                "pid": self.recording.pid, "path": str(self.recording_path),
                "profile": self.recording_profile}

    def start_bag(self, profile):
        if profile not in BAG_PROFILES:
            raise ValueError("未知录包档位")
        with self.lock:
            if self.recording is not None and self.recording.poll() is None:
                raise ValueError("已有正在运行的录包任务，请先停止")
            if shutil.which("ros2") is None:
                raise ValueError("没有找到 ros2，请先 source /opt/ros/humble/setup.bash")
            result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True,
                                    stdin=subprocess.DEVNULL, timeout=8)
            if result.returncode != 0:
                raise ValueError("ROS 图不可用：" + limited(result.stderr, 180))
            available = set(result.stdout.splitlines())
            topics = [topic for topic in BAG_PROFILES[profile] if topic in available]
            required = {
                "core": {"/wheel/odom", "/chassis_info_fb"},
                "rgbd": {"/fwmini/camera/color/image_raw", "/fwmini/camera/depth/image_raw",
                         "/camera/color/image_raw", "/camera/depth/image_raw"},
                "slam": {"/scan", "/map"},
            }[profile]
            if not (required & available):
                raise ValueError(f"{profile} 缺少核心话题（{', '.join(sorted(required))}）；先启动对应节点")
            if not topics:
                raise ValueError("当前 ROS 图没有本档位的话题，先启动传感器/底盘（不必运动）")
            disk_free = shutil.disk_usage(self.root).free
            minimum = 3 * 1024**3 if profile == "rgbd" else 1024**3
            if disk_free < minimum:
                raise ValueError("磁盘余量不足，请先清理/移动旧录包")
            folder = self.folder() / "bags"
            folder.mkdir(exist_ok=True)
            name = f"{profile}_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{secrets.token_hex(2)}"
            destination = folder / name
            log = folder / (name + ".log")
            cmd = ["ros2", "bag", "record", "-o", str(destination), *topics]
            handle = log.open("w", encoding="utf-8")
            try:
                process = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT,
                                           stdin=subprocess.DEVNULL, start_new_session=True)
            finally:
                handle.close()
            time.sleep(0.45)
            if process.poll() is not None:
                raise ValueError(f"rosbag 录制启动失败，见日志：{log}")
            self.recording = process
            self.recording_profile = profile
            self.recording_path = destination
            self.recording_log = log
            record = self.state()
            entry = {"profile": profile, "path": str(destination), "log": str(log),
                     "topics": topics, "started": now(), "pid": process.pid}
            record["bag_runs"].append(entry)
            self._save(record)
            self._event("bag_start", **entry)
            return entry

    def stop_bag(self):
        with self.lock:
            if self.recording is None:
                raise ValueError("当前没有由工作台启动的录包进程")
            proc = self.recording
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
            record = self.state()
            for entry in reversed(record["bag_runs"]):
                if entry["pid"] == proc.pid:
                    entry["stopped"] = now()
                    entry["returncode"] = proc.returncode
                    break
            self._save(record)
            self._event("bag_stop", pid=proc.pid, returncode=proc.returncode)
            self.recording = None
            return dict(path=str(self.recording_path), returncode=proc.returncode)

    def report(self, fmt):
        with self.lock:
            record = self.state()
        rows = []
        for task in TASKS:
            entry = record["steps"].get(task["id"], {})
            rows.append([task["id"], task["lane"], task["title"],
                         entry.get("status", "NOT_RUN"), entry.get("time", ""),
                         entry.get("note", ""), entry.get("evidence", "")])
        if fmt == "csv":
            output = io.StringIO()
            writer = csv.writer(output)
            writer.writerow(["ID", "方向", "测试步骤", "状态", "时间", "备注", "证据路径"])
            writer.writerows(rows)
            return output.getvalue().encode("utf-8-sig")
        head = [f"# FW-mini 现场验收：{record['title']}", "", f"- 会话：`{record['id']}`",
                f"- 场地：{record['place']}", f"- 开始：{record['started']}",
                f"- 证据目录：`{self.folder()}`", "", "| 方向 | 检查项 | 结果 | 备注/证据 |",
                "|---|---|---|---|"]
        for _, lane, title, status, _, note, evidence in rows:
            clean = lambda x: x.replace("|", "\\|").replace("\n", " ")
            head.append(f"| {clean(lane)} | {clean(title)} | {status} | {clean(note)} {clean(evidence)} |")
        head += ["", "## ROS bag", ""]
        for bag in record["bag_runs"]:
            head.append(f"- `{bag['path']}` · {bag['profile']} · {bag.get('started', '')} → {bag.get('stopped', '正在录制/非正常中断')} · 话题 {', '.join(bag['topics'])}")
        head += ["", "## 只读诊断记录", ""]
        for name, info in record["last_probe"].items():
            head.append(f"- {name}：`{info['filename']}`（{info['time']}）")
        return ("\n".join(head) + "\n").encode("utf-8")

class Handler(BaseHTTPRequestHandler):
    server_version = "FWminiFieldWorkbench/0.1"

    def log_message(self, fmt, *args):
        return  # Access logs contain no benefit during field use; errors appear to stderr.

    def _send(self, payload, mime="application/json", code=200, attachment=None):
        raw = payload if isinstance(payload, bytes) else json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", mime + ("; charset=utf-8" if mime.startswith("text/") or mime == "application/json" else ""))
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        if attachment:
            self.send_header("Content-Disposition", f'attachment; filename="{attachment}"')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        path = urlparse(self.path).path
        db = self.server.db
        if path == "/":
            html = (HERE / "index.html").read_text(encoding="utf-8")
            self._send(html.replace("__CSRF_TOKEN__", self.server.token).encode("utf-8"), "text/html")
        elif path == "/api/state":
            sessions = db.sessions()
            self._send(dict(session=db.state(), tasks=TASKS, lanes=LANES, bags=list(BAG_PROFILES),
                            recording=db.bag_state(), directory=str(db.folder()), sessions=sessions))
        elif path == "/api/report.md":
            self._send(db.report("md"), "text/markdown", attachment="fwmini_field_report.md")
        elif path == "/api/report.csv":
            self._send(db.report("csv"), "text/csv", attachment="fwmini_field_report.csv")
        else:
            self._send({"error": "not found"}, code=404)

    def do_POST(self):
        if self.headers.get("X-Workbench-Token", "") != self.server.token:
            self._send({"error": "missing/invalid local workbench token"}, code=403)
            return
        try:
            length = int(self.headers.get("Content-Length", "-1"))
            if not 0 <= length <= 10000:
                raise ValueError("JSON request exceeds size limit")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("expect JSON object")
            path = urlparse(self.path).path
            db = self.server.db
            if path == "/api/new":
                result = db.new_session(body.get("title"), body.get("place"))
            elif path == "/api/open":
                result = db.open_session(body.get("id"))
            elif path == "/api/status":
                result = db.set_status(body.get("step"), body.get("status"), body.get("note"), body.get("evidence"))
            elif path == "/api/probe":
                name = body.get("probe")
                result = {"output": probe(name), "probe": name}
                result["file"] = db.store_probe(name, result["output"])
            elif path == "/api/bag/start":
                result = db.start_bag(body.get("profile"))
            elif path == "/api/bag/stop":
                result = db.stop_bag()
            else:
                self._send({"error": "not found"}, code=404)
                return
            self._send({"ok": True, "result": result})
        except (ValueError, TimeoutError, OSError, subprocess.TimeoutExpired) as exc:
            self._send({"ok": False, "error": limited(exc, 500)}, code=400)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="bind loopback only (SSH forward for remote access)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--data", type=Path, default=PROJECT / "evidence" / "field_workbench")
    opts = parser.parse_args()
    if opts.host not in ("127.0.0.1", "::1"):
        parser.error("only loopback binding is supported; use SSH port forwarding")
    server = ThreadingHTTPServer((opts.host, opts.port), Handler)
    server.db = Records(opts.data)
    server.token = secrets.token_hex(16)
    print(f"FW-mini 现场工作台：http://{opts.host}:{server.server_port}", flush=True)
    print(f"记录目录：{server.db.root}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if server.db.recording is not None:
            server.db.stop_bag()
        server.server_close()

if __name__ == "__main__":
    main()
