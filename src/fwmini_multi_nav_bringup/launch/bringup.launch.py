1: """Backend-selectable SLAM + optional gated Nav2 for ROS 2 Humble FW-mini.
2: 
3: Only map->odom is published by the selected backend; wheel odom->base_link,
4: camera/MID360 drivers and all sensor TFs remain owned by the existing robot.
5: """
6: 
7: import os
8: 
9: from ament_index_python.packages import get_package_share_directory
10: from launch import LaunchDescription
11: from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
12: from launch.launch_description_sources import PythonLaunchDescriptionSource
13: from launch.substitutions import LaunchConfiguration
14: from launch_ros.actions import Node
15: 
16: from fwmini_multi_nav_bringup.contracts import (
17:     ContractError,
18:     validate_hardware_contract,
19:     validate_selection,
20: )
21: 
22: 
23: def _cfg(context, name):
24:     return LaunchConfiguration(name).perform(context)
25: 
26: 
27: def _bool(context, name):
28:     value = _cfg(context, name).lower()
29:     if value not in ("true", "false"):
30:         raise ContractError(name + " must be true or false")
31:     return value == "true"
32: 
33: 
34: def _spawn(context):
35:     backend = _cfg(context, "backend")
36:     mode = _cfg(context, "mode")
37:     database = os.path.expanduser(_cfg(context, "database_path"))
38:     posegraph = os.path.expanduser(_cfg(context, "posegraph_file"))
39:     source = validate_selection(
40:         backend, mode, _cfg(context, "scan_source"), database, posegraph
41:     )
42:     use_sim = _bool(context, "use_sim_time")
43:     actions = []
44:     share = get_package_share_directory("fwmini_multi_nav_bringup")
45: 
46:     # No SLAM backend in this launch creates odom->base_link. The original
47:     # fwmini_odom_adapter owns it. External bringup is started independently.
48:     if source == "depth":
49:         actions.append(Node(
50:             package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node",
51:             name="fwmini_depth_to_scan", output="screen",
52:             parameters=[{
53:                 "use_sim_time": use_sim, "scan_height": 12,
54:                 "scan_time": 0.1, "range_min": 0.25, "range_max": 8.0,
55:             }],
56:             remappings=[
57:                 ("depth", _cfg(context, "depth_topic")),
58:                 ("depth_camera_info", _cfg(context, "depth_camera_info_topic")),
59:                 ("scan", "/scan"),
60:             ],
61:         ))
62:     elif source == "cloud":
63:         actions.append(Node(
64:             package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
65:             name="fwmini_cloud_to_scan", output="screen",
66:             parameters=[{
67:                 "use_sim_time": use_sim,
68:                 "target_frame": "base_link", "transform_tolerance": 0.15,
69:                 "min_height": -0.10, "max_height": 0.40,
70:                 "angle_min": -3.14159265, "angle_max": 3.14159265,
71:                 "angle_increment": 0.0087, "scan_time": 0.1,
72:                 "range_min": 0.25, "range_max": 8.0, "use_inf": True,
73:                 "queue_size": 10,
74:             }],
75:             remappings=[("cloud_in", _cfg(context, "cloud_topic")), ("scan", "/scan")],
76:         ))
77: 
78:     if backend == "rtabmap":
79:         # Deliberately do NOT start rtabmap_odom/rgbd_odometry.
80:         # The robot's own /odom supplies continuous tracking and owns odom TF.
81:         params = {
82:             "use_sim_time": use_sim,
83:             "frame_id": "base_link",
84:             "map_frame_id": "map",
85:             "odom_frame_id": "",
86:             "publish_tf": True,
87:             "subscribe_depth": True,
88:             "subscribe_rgb": True,
89:             "subscribe_rgbd": False,
90:             "subscribe_scan": False,
91:             "subscribe_odom_info": False,
92:             "approx_sync": True,
93:             "qos_image": 2,
94:             "qos_camera_info": 2,
95:             "qos_odom": 2,
96:             "database_path": database,
97:             "Mem/IncrementalMemory": "true" if mode == "mapping" else "false",
98:             "Mem/InitWMWithAllNodes": "false" if mode == "mapping" else "true",
99:             "Grid/FromDepth": "true",
100:             "Grid/3D": "false",
101:             "Grid/RangeMax": "8.0",
102:             "Grid/CellSize": "0.05",
103:         }
104:         actions.append(Node(
105:             package="rtabmap_slam", executable="rtabmap",
106:             name="rtabmap", namespace="rtabmap", output="screen",
107:             parameters=[params],
108:             remappings=[
109:                 ("rgb/image", _cfg(context, "rgb_topic")),
110:                 ("depth/image", _cfg(context, "depth_topic")),
111:                 ("rgb/camera_info", _cfg(context, "camera_info_topic")),
112:                 ("odom", _cfg(context, "odom_topic")),
113:                 ("grid_map", "/map"),
114:             ],
115:         ))
116:     elif backend == "slam_toolbox":
117:         # The official Humble launch owns lifecycle auto-activation.
118:         from launch.actions import EmitEvent, RegisterEventHandler
119:         from launch.events import matches_action
120:         from launch_ros.actions import LifecycleNode
121:         from launch_ros.event_handlers import OnStateTransition
122:         from launch_ros.events.lifecycle import ChangeState
123:         from lifecycle_msgs.msg import Transition
124: 
125:         config = os.path.join(
126:             share, "config", "slam_toolbox", mode + ".yaml"
127:         )
128:         overrides = {"use_sim_time": use_sim, "use_lifecycle_manager": False}
129:         if mode == "localization":
130:             overrides["map_file_name"] = posegraph
131:         node = LifecycleNode(
132:             package="slam_toolbox",
133:             executable=(
134:                 "async_slam_toolbox_node" if mode == "mapping"
135:                 else "localization_slam_toolbox_node"
136:             ),
137:             name="slam_toolbox", output="screen",
138:             parameters=[config, overrides],
139:             remappings=[("scan", "/scan")],
140:         )
141:         actions.extend([
142:             node,
143:             RegisterEventHandler(OnStateTransition(
144:                 target_lifecycle_node=node, start_state="configuring",
145:                 goal_state="inactive",
146:                 entities=[EmitEvent(event=ChangeState(
147:                     lifecycle_node_matcher=matches_action(node),
148:                     transition_id=Transition.TRANSITION_ACTIVATE
149:                 ))],
150:             )),
151:             EmitEvent(event=ChangeState(
152:                 lifecycle_node_matcher=matches_action(node),
153:                 transition_id=Transition.TRANSITION_CONFIGURE,
154:             )),
155:         ])
156:     elif backend == "cartographer_2d":
157:         lua_dir = os.path.join(share, "config", "cartographer")
158:         actions.extend([
159:             Node(
160:                 package="cartographer_ros", executable="cartographer_node",
161:                 name="cartographer_node", output="screen",
162:                 parameters=[{"use_sim_time": use_sim}],
163:                 arguments=[
164:                     "-configuration_directory", lua_dir,
165:                     "-configuration_basename", "fwmini_2d.lua",
166:                 ],
167:                 remappings=[("scan", "/scan"), ("odom", _cfg(context, "odom_topic"))],
168:             ),
169:             Node(
170:                 package="cartographer_ros",
171:                 executable="cartographer_occupancy_grid_node",
172:                 name="cartographer_occupancy_grid_node", output="screen",
173:                 parameters=[{"use_sim_time": use_sim}],
174:                 arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
175:                 remappings=[("map", "/map")],
176:             ),
177:         ])
178: 
179:     if _bool(context, "navigation"):
180:         contract = _cfg(context, "hardware_contract")
181:         validate_hardware_contract(contract, backend, source)
182:         nav2_file = os.path.expanduser(_cfg(context, "nav2_params_file"))
183:         if not nav2_file or not os.path.isfile(nav2_file):
184:             raise ContractError("Nav2 requires a full verified nav2_params_file")
185:         actions.append(IncludeLaunchDescription(
186:             PythonLaunchDescriptionSource(os.path.join(
187:                 get_package_share_directory("nav2_bringup"),
188:                 "launch", "navigation_launch.py",
189:             )),
190:             launch_arguments={
191:                 "use_sim_time": "true" if use_sim else "false",
192:                 "params_file": nav2_file,
193:                 "use_composition": "False",
194:             }.items(),
195:         ))
196:     return actions
197: 
198: 
199: def generate_launch_description():
200:     share = get_package_share_directory("fwmini_multi_nav_bringup")
201:     defaults = [
202:         ("backend", "rtabmap", "rtabmap | slam_toolbox | cartographer_2d"),
203:         ("mode", "mapping", "mapping or native-map localization"),
204:         ("scan_source", "auto", "auto | none | depth | cloud"),
205:         ("use_sim_time", "false", "true for rosbag2 --clock replay"),
206:         ("navigation", "false", "Nav2 is gated until physical calibration"),
207:         ("database_path", "~/.ros/fwmini_rtabmap.db", "RTAB-Map native .db"),
208:         ("posegraph_file", "", "SLAM Toolbox serialized pose graph"),
209:         ("odom_topic", "/wheel/odom", "Existing fwmini_odom_adapter output"),
210:         ("rgb_topic", "/fwmini/camera/color/image_raw", "Rectified RGB Image"),
211:         ("depth_topic", "/fwmini/camera/depth/image_raw", "Registered depth Image"),
212:         ("camera_info_topic", "/fwmini/camera/color/camera_info", "RGB CameraInfo"),
213:         ("depth_camera_info_topic", "/fwmini/camera/depth/camera_info", "Depth CameraInfo"),
214:         ("cloud_topic", "/livox/lidar", "MID360 PointCloud2 topic"),
215:         ("hardware_contract", os.path.join(share, "config", "hardware_contract.json"),
216:          "Confirmed real extrinsics/chassis status JSON"),
217:         ("nav2_params_file", "", "Complete verified Nav2 omni parameters"),
218:     ]
219:     return LaunchDescription([
220:         *[DeclareLaunchArgument(name, default_value=default, description=desc)
221:           for name, default, desc in defaults],
222:         OpaqueFunction(function=_spawn),
223:     ])