"""
server.py — 机械臂调试控制台后端
===================================
Flask (REST) + 原生 WebSocket (/ws) + numpy FK 仿真

Modes:
  SIMULATION（默认）: 无 ROS，纯内存状态，用于前端开发和调试
  ROS_MODE（设置 ROS_MODE=True）: 连接真实机械臂，发布/订阅 ROS 话题

Usage:
  pip install -r requirements.txt
  python server.py
  浏览器打开 http://localhost:5000
"""

import asyncio
import json
import os
import threading
import time
from datetime import datetime

from flask import Flask, jsonify, request, render_template, send_from_directory
from flask_cors import CORS
from asgiref.wsgi import WsgiToAsgi

# ── ROS 模式开关 ────────────────────────────────────────────────
ROS_MODE = True

try:
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARN] ROS 不可用，将以纯仿真模式运行")

_ros_node = None  # ArmROSBridge 实例

if ROS_MODE and ROS_AVAILABLE:
    class ArmROSBridge:
        """Web 前端 ↔ ROS 机械臂节点之间的桥接器。

        发布：
          /arm_control/joint_commands   (sensor_msgs/JointState)
          /arm_config                   (std_msgs/String, JSON)
        订阅：
          /arm_status/joint_states      (sensor_msgs/JointState) → 前端显示"实际角度"
        """
        def __init__(self):
            rospy.init_node('arm_web_bridge', anonymous=True, disable_signals=True)
            self.pub = rospy.Publisher('/arm_control/joint_commands', JointState, queue_size=10)
            self._config_pub = rospy.Publisher('/arm_config', String, queue_size=10)
            self.sub = rospy.Subscriber('/arm_status/joint_states', JointState,
                                        self._on_joint_state, queue_size=10)
            self._latest = {}
            self._lock = threading.Lock()
            print('[ROS] Bridge 初始化完成')

        def _on_joint_state(self, msg):
            with self._lock:
                self._latest = dict(zip(msg.name, msg.position))

        def publish(self, values: dict):
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = list(values.keys())
            js.position = [float(v) for v in values.values()]
            self.pub.publish(js)

        def publish_arm_config(self, config: dict):
            msg = String()
            msg.data = json.dumps(config)
            self._config_pub.publish(msg)

    def _ros_spinner():
        global _ros_node
        try:
            _ros_node = ArmROSBridge()
            rospy.spin()
        except Exception as e:
            print(f"[ROS] Spinner 异常，切换为仿真: {e}")

    _t = threading.Thread(target=_ros_spinner, daemon=True)
    _t.start()
    print("[INFO] ROS 模式已启用 — Bridge 活跃")

# ── Flask 应用 ─────────────────────────────────────────────────
app = Flask(__name__, template_folder='.')
CORS(app)

# ── 关节定义 ──────────────────────────────────────────────────
JOINT_DEFINITIONS = {
    "joint_1": {"name": "底座旋转",       "min": -180.0, "max": 180.0, "unit": "deg", "default": 0.0},
    "joint_2": {"name": "肩关节",         "min": -90.0,  "max": 90.0,  "unit": "deg", "default": 0.0},
    "joint_3": {"name": "肘关节",         "min": -135.0, "max": 135.0, "unit": "deg", "default": 0.0},
    "joint_4": {"name": "腕俯仰",         "min": -90.0,  "max": 90.0,  "unit": "deg", "default": 0.0},
    "joint_5": {"name": "腕翻滚",         "min": -180.0, "max": 180.0, "unit": "deg", "default": 0.0},
    "joint_6": {"name": "夹爪开合",       "min": 0.0,    "max": 100.0, "unit": "%",   "default": 0.0},
    "joint_7": {"name": "辅助轴 1",       "min": -90.0,  "max": 90.0,  "unit": "deg", "default": 0.0},
    "joint_8": {"name": "辅助轴 2",       "min": -90.0,  "max": 90.0,  "unit": "deg", "default": 0.0},
}

# ── 共享状态 ───────────────────────────────────────────────────
class ArmState:
    """线程安全的内存状态（仿真模式用）。"""
    def __init__(self):
        self._lock = threading.RLock()
        self._values = {k: d["default"] for k, d in JOINT_DEFINITIONS.items()}
        self._last_update = None

    def set(self, key, value):
        with self._lock:
            self._values[key] = float(value)
            self._last_update = datetime.now().isoformat()

    def get_all(self):
        with self._lock:
            return {
                k: {"value": round(v, 3), "min": d["min"],
                    "max": d["max"], "unit": d["unit"], "name": d["name"]}
                for k, (v, d) in zip(self._values, JOINT_DEFINITIONS.items())
            }

    def get_values_only(self):
        with self._lock:
            return {k: round(v, 3) for k, v in self._values.items()}


arm_state = ArmState()
_arm_config = None  # 最近一次 POST 的 DH 配置 JSON

# ── Flask 路由 ─────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/params', methods=['GET'])
def get_params():
    return jsonify(arm_state.get_all())


@app.route('/api/params', methods=['POST'])
def update_params():
    data = request.get_json() or {}
    updated = []
    for key, defn in JOINT_DEFINITIONS.items():
        if key in data:
            raw = float(data[key])
            clamped = max(defn["min"], min(defn["max"], raw))
            arm_state.set(key, clamped)
            updated.append(key)
    if ROS_MODE and _ros_node:
        _ros_node.publish(arm_state.get_values_only())
    return jsonify({"ok": True, "updated": updated, "all": arm_state.get_values_only()})


@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify({
        "ros_mode": ROS_MODE,
        "ros_connected": ROS_MODE and ROS_AVAILABLE,
        "last_update": arm_state._last_update,
    })


@app.route('/api/arm_config', methods=['POST'])
def post_arm_config():
    """接收 DH 配置 JSON → 生成 URDF → 推送给前端拓扑信息。"""
    global _arm_config
    data = request.get_json()
    if not data:
        return jsonify({"error": "No JSON body"}), 400

    _arm_config = data
    print(f"[Arm Config] arm_name={data.get('arm_name')} dof={data.get('dof')}")

    if ROS_MODE and _ros_node:
        _ros_node.publish_arm_config(data)

    try:
        from urdf_generator import generate_urdf
        urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'generated_urdfs')
        os.makedirs(urdf_dir, exist_ok=True)
        urdf_path = os.path.join(urdf_dir, 'current.urdf')
        generate_urdf(data, urdf_path)

        # 解析 URDF → 拓扑推给前端（Three.js 渲染用）
        topo = _build_topology_from_urdf(urdf_path)
        if topo:
            _ws_hub.push(topo)
    except Exception as e:
        print(f"[Arm Config] URDF 生成/推送失败: {e}")

    return jsonify({"ok": True, "arm_name": data.get("arm_name"), "dof": data.get("dof")})


@app.route('/api/arm_config', methods=['GET'])
def get_arm_config():
    if _arm_config is None:
        return jsonify({"error": "No arm configuration set yet"}), 404
    return jsonify(_arm_config)


@app.route('/api/load_urdf', methods=['POST'])
def load_urdf_file():
    """上传 URDF → 推拓扑给前端。"""
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'generated_urdfs')
    os.makedirs(urdf_dir, exist_ok=True)
    target = os.path.join(urdf_dir, 'uploaded.urdf')

    if 'file' in request.files:
        request.files['file'].save(target)
    else:
        body = request.get_data(as_text=True)
        if not body:
            return jsonify({"error": "No file or body"}), 400
        with open(target, 'w', encoding='utf-8') as fp:
            fp.write(body)

    topo = _build_topology_from_urdf(target)
    if topo:
        _ws_hub.push(topo)

    # 统计 dof
    dof = topo["links"].count("revolute") if topo else 0
    return jsonify({"ok": True, "urdf": target, "dof": dof})


@app.route('/api/sim/reload_urdf', methods=['POST'])
def sim_reload_urdf():
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'generated_urdfs')
    for name in ['current.urdf', 'uploaded.urdf']:
        path = os.path.join(urdf_dir, name)
        if os.path.exists(path):
            topo = _build_topology_from_urdf(path)
            if topo:
                _ws_hub.push(topo)
            return jsonify({"ok": True, "urdf": path})
    return jsonify({"error": "No URDF found"}), 404


@app.route('/api/reset', methods=['POST'])
def reset_params():
    for key, defn in JOINT_DEFINITIONS.items():
        arm_state.set(key, defn["default"])
    return jsonify({"ok": True})


# ── 静态资源 ───────────────────────────────────────────────────
_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets')
_VENDOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vendor')


@app.route('/assets/<path:filename>')
def serve_asset(filename):
    return send_from_directory(_ASSETS_DIR, filename)


@app.route('/assets/vendor/<path:filename>')
def serve_vendor(filename):
    return send_from_directory(_VENDOR_DIR, filename)


# ── URDF → Three.js 拓扑转换（纯 numpy，不依赖 GPU）───────────
# 从 URDF XML 解析 link/joint 几何，供前端 Three.js 构建 3D 场景

import numpy as np


def _build_topology_from_urdf(urdf_path: str):
    """解析 URDF，返回前端 Three.js 可直接消费的拓扑 JSON。

    返回结构：
      {
        "type": "mani_topology",
        "links": [
          {"name": "base_link", "shapes": [...], "joint_type": null},
          {"name": "link_1",    "shapes": [...], "joint_type": "revolute", "joint_origin": [...]},
          ...
        ]
      }
    """
    import re

    with open(urdf_path, 'r', encoding='utf-8') as f:
        xml = f.read()

    # 去掉 XML 注释
    xml_clean = re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)

    links_out = []

    # ── 解析所有 link ──────────────────────────────────────────
    for lm in re.finditer(r'<link\s+name="([^"]+)"[^>]*>(.*?)</link>', xml_clean, re.DOTALL):
        link_name = lm.group(1)
        link_body = lm.group(2)
        shapes = []

        for vm in re.finditer(r'<visual>(.*?)</visual>', link_body, re.DOTALL):
            vbody = vm.group(1)

            # origin
            om = re.search(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"', vbody)
            xyz = [float(v) for v in om.group(1).split()] if om else [0, 0, 0]
            rpy = [float(v) for v in om.group(2).split()] if om else [0, 0, 0]

            # material
            mm = re.search(r'<material\s+name="([^"]+)"', vbody)
            mat_name = mm.group(1) if mm else 'default'
            color = {'arm_white': [0.95, 0.95, 0.95],
                     'arm_motor': [0.05, 0.05, 0.05],
                     'tip_red':   [1.0, 0.2, 0.2],
                     'white':     [0.95, 0.95, 0.95],
                     'motor':     [0.05, 0.05, 0.05]}.get(mat_name, [0.85, 0.85, 0.9])

            # geometry
            rpy_rad = np.array(rpy, dtype=np.float32)
            R = _rpy_to_R(rpy_rad)
            quat = _R_to_quat_wxyz(R)
            pose = {"p": xyz, "q": quat.tolist()}

            bm = re.search(r'<box\s+size="([^"]+)"', vbody)
            if bm:
                sx, sy, sz = [float(v) for v in bm.group(1).split()]
                shapes.append({"type": "box", "size": [sx, sy, sz], "local_pose": pose, "color": color})
                continue

            cm = re.search(r'<cylinder\s+radius="([^"]+)"\s+length="([^"]+)"', vbody)
            if cm:
                r, L = float(cm.group(1)), float(cm.group(2))
                shapes.append({"type": "cylinder", "radius": r, "length": L, "local_pose": pose, "color": color})
                continue

            sm = re.search(r'<sphere\s+radius="([^"]+)"', vbody)
            if sm:
                shapes.append({"type": "sphere", "radius": float(sm.group(1)), "local_pose": pose, "color": color})
                continue

            mesh_m = re.search(r'<mesh\s+filename="([^"]+)"', vbody)
            if mesh_m:
                shapes.append({"type": "mesh", "filename": mesh_m.group(1), "local_pose": pose, "color": color})

        links_out.append({"name": link_name, "shapes": shapes})

    # ── 解析所有 joint，标记对应 child link 的关节类型和轴 ───
    for jm in re.finditer(
            r'<joint\s+name="([^"]+)"\s+type="([^"]+)"[^>]*>(.*?)</joint>',
            xml_clean, re.DOTALL):
        child = re.search(r'<child\s+link="([^"]+)"', jm.group(3))
        origin_m = re.search(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"', jm.group(3))
        axis_m = re.search(r'<axis\s+xyz="([^"]+)"', jm.group(3))

        if not child:
            continue

        child_name = child.group(1)
        joint_type = jm.group(2)
        origin_xyz = [float(v) for v in origin_m.group(1).split()] if origin_m else [0, 0, 0]
        axis_xyz = [float(v) for v in axis_m.group(1).split()] if axis_m else [0, 0, 1]

        # 找到对应的 link，补充关节信息
        for link in links_out:
            if link["name"] == child_name:
                link["joint_type"] = joint_type
                link["joint_origin"] = origin_xyz
                link["joint_axis"] = axis_xyz
                break

    # ── 推导 base URL（供前端加载 mesh） ─────────────────────
    base_url = ''
    if 'assets' in urdf_path:
        idx = urdf_path.find('assets')
        base_url = '/' + urdf_path[idx:].replace(os.sep, '/').rsplit('/', 1)[0] + '/'

    return {"type": "mani_topology", "links": links_out, "base_url": base_url}


def _rpy_to_R(rpy):
    """RPY (roll, pitch, yaw) → 3×3 旋转矩阵。"""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,    cp*sr,            cp*cr],
    ], dtype=np.float32)


def _R_to_quat_wxyz(R):
    """3×3 旋转矩阵 → 四元数 (w, x, y, z)。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = 2.0 * np.sqrt(tr + 1.0)
        return np.array([0.25 * S,
                         (R[2, 1] - R[1, 2]) / S,
                         (R[0, 2] - R[2, 0]) / S,
                         (R[1, 0] - R[0, 1]) / S], dtype=np.float32)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / S,
                          0.25 * S,
                          (R[0, 1] + R[1, 0]) / S,
                          (R[0, 2] + R[2, 0]) / S], dtype=np.float32)
    if R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / S,
                          (R[0, 1] + R[1, 0]) / S,
                          0.25 * S,
                          (R[1, 2] + R[2, 1]) / S], dtype=np.float32)
    S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return np.array([(R[1, 0] - R[0, 1]) / S,
                      (R[0, 2] + R[2, 0]) / S,
                      (R[1, 2] + R[2, 1]) / S,
                      0.25 * S], dtype=np.float32)


# ── WebSocket Hub ──────────────────────────────────────────────
class _WSHub:
    """线程安全的 WS 客户端注册表 + 广播队列。"""
    def __init__(self):
        self.clients = set()

    def add(self, q):
        self.clients.add(q)

    def remove(self, q):
        self.clients.discard(q)

    def push(self, msg: dict):
        """非阻塞广播，队列满时丢弃旧消息。"""
        for q in list(self.clients):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                try:
                    q.get_nowait()
                    q.put_nowait(msg)
                except Exception:
                    pass


_ws_hub = _WSHub()


def _apply_incoming_params(payload: dict):
    """把前端 slider 数据写入共享状态，并推送给 ROS。"""
    if not isinstance(payload, dict):
        return
    for key, defn in JOINT_DEFINITIONS.items():
        if key in payload:
            raw = float(payload[key])
            clamped = max(defn["min"], min(defn["max"], raw))
            arm_state.set(key, clamped)
    if ROS_MODE and _ros_node:
        _ros_node.publish(arm_state.get_values_only())


async def _ws_send_loop(q: asyncio.Queue, send):
    while True:
        msg = await q.get()
        try:
            await send({"type": "websocket.send", "text": json.dumps(msg)})
        except Exception:
            return


async def _broadcaster():
    """定期广播 arm_state 到所有 WS 客户端（20 Hz）。"""
    last_snapshot = None
    while True:
        await asyncio.sleep(0.05)
        snapshot = arm_state.get_values_only()
        if snapshot != last_snapshot:
            last_snapshot = snapshot
            _ws_hub.push({
                "type": "arm_state",
                "ts": datetime.now().isoformat(timespec='milliseconds'),
                "data": {"target": snapshot, "actual": snapshot},
            })


async def _ws_handler(scope, receive, send):
    """ASGI WebSocket handler for /ws。"""
    if scope["type"] != "websocket":
        return

    await send({"type": "websocket.accept"})
    q: asyncio.Queue = asyncio.Queue(maxsize=8)
    _ws_hub.add(q)
    sender_task = asyncio.create_task(_ws_send_loop(q, send))
    print(f"[WS] 客户端连接 (总数={len(_ws_hub.clients)})")

    # 连接时立即推送当前状态
    try:
        q.put_nowait({
            "type": "arm_state",
            "ts": datetime.now().isoformat(timespec='milliseconds'),
            "data": {"target": arm_state.get_values_only(), "actual": arm_state.get_values_only()},
        })
    except asyncio.QueueFull:
        pass

    try:
        while True:
            event = await receive()
            if event["type"] == "websocket.disconnect":
                break
            if event["type"] != "websocket.receive":
                continue

            text = event.get("text") or (event.get("bytes") or b"").decode("utf-8", "ignore")
            if not text:
                continue

            try:
                msg = json.loads(text)
            except Exception:
                continue

            if msg.get("type") == "params":
                _apply_incoming_params(msg.get("data") or {})
            elif msg.get("type") == "subscribe":
                try:
                    q.put_nowait({"type": "subscribed", "channels": msg.get("channels", [])})
                except asyncio.QueueFull:
                    pass
    finally:
        sender_task.cancel()
        _ws_hub.remove(q)
        print(f"[WS] 客户端断开 (剩余={len(_ws_hub.clients)})")


# ── ASGI 路由分发 ──────────────────────────────────────────────
class WSDashApp:
    def __init__(self, flask_asgi, ws_handler):
        self.flask_asgi = flask_asgi
        self.ws_handler = ws_handler

    async def __call__(self, scope, receive, send):
        if scope["type"] == "websocket" and scope.get("path") == "/ws":
            await self.ws_handler(scope, receive, send)
        else:
            await self.flask_asgi(scope, receive, send)


flask_asgi = WsgiToAsgi(app)
asgi_app = WSDashApp(flask_asgi, _ws_handler)

# ── 入口 ───────────────────────────────────────────────────────
if __name__ == '__main__':
    port = 5000
    mode_label = "ROS" if ROS_MODE else "Simulation"
    print(f"""
  ╔═══════════════════════════════════════════╗
  ║     Robotic Arm Control — Backend         ║
  ╠═══════════════════════════════════════════╣
  ║  Mode   : {mode_label:<29}║
  ║  URL    : http://localhost:{port}           ║
  ╚═══════════════════════════════════════════╝
    """)
    import uvicorn
    config = uvicorn.Config(
        asgi_app,
        host='0.0.0.0',
        port=port,
        log_level='info',
        loop="asyncio",
        lifespan="off",
    )

    async def _main():
        broadcaster = asyncio.create_task(_broadcaster())
        try:
            await uvicorn.Server(config).serve()
        finally:
            broadcaster.cancel()

    asyncio.run(_main())
