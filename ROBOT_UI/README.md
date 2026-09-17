# ROBOT_UI — 机械臂调试控制台

> 一套面向"机械臂重力补偿调参"场景的网页调试前端 + Python 后端。
> 功能暂时只有在网页修改发送的json，仿真未做好
> 下面是AI写的参考一下

---

## 一、项目目标 & 解决的问题

1. **离线调试**：无 ROS、无机械臂时，前端仍可运行"演示模式"，纯前端可拖动滑块并可视化，方便 UI 设计与几何验证。
2. **在线联调**：接上 ROS 后，前端通过 `/api/*` + WebSocket 与 ROS 节点双向同步：
   - 下行：**网页滑块 → ROS 关节命令 → 实体机械臂**
   - 上行：**实体机械臂 → ROS 关节状态 → 网页"实际角度"显示**
3. **参数配置导出**：网页上配置的 DH 参数（质量、连杆长、质心、扭转角等）一键打包成 JSON，自动生成 URDF 并装载进仿真器。
4. **3D 预览**：内置 Three.js 渲染器，支持 DH 编辑视图。

---

## 二、目录结构

```
ROBOT_UI/
├── README.md                   ← 本文件（交接说明）
├── index.html                  ← 前端单页应用（HTML+CSS+JS，含 Three.js 3D 渲染）
├── server.py                   ← Python 后端（Flask + 原生 WebSocket，583行）
├── urdf_generator.py           ← JSON → URDF 的转换器（被 server.py 调用）
├── example.json                ← DH 配置示例（旧字段 l/d/alpha/theta）
├── requirements.txt            ← Python 依赖清单
├── download_vendor.sh          ← 下载 CDN 静态资源的脚本
├── assets/                     ← 静态资源（前端通过 /assets/... 路由访问）
│   ├── threejs/              ← Three.js 库（three.min.js, OrbitControls.js）
│   ├── vendor/               ← STLLoader.js 等第三方
│   ├── so100/                ← SO-100 6-DOF 机械臂 URDF + mesh（可选）
│   └── two_axis_arm/          ← 两轴机械臂 URDF（可选）
└── generated_urdfs/            ← 自动生成的 URDF 缓存目录
    ├── current.urdf           ← 通过 /api/arm_config 生成的最新 URDF
    └── uploaded.urdf          ← 通过 /api/load_urdf 上传的 URDF
```


## 三、技术栈一览

### 前端
- **HTML + 原生 CSS + 原生 JavaScript**（零构建工具，浏览器直接跑）
- **Three.js**（本地化在 `assets/threejs/`，**禁用 CDN** 避免 MIME 问题）
- **OrbitControls.js / STLLoader.js**：3D 视角控制 + STL 模型加载
- **浏览器原生 WebSocket**：`ws://host:5000/ws`

### 后端
- **Flask 3.0**（HTTP + 静态资源）
- **asgiref + uvicorn**（把 Flask WSGI 包装成 ASGI，在顶层用一个 `WSDashApp` 路由分发 HTTP / WebSocket）
- **Flask-CORS**（跨域）
- **numpy**（URDF XML 解析 + RPY→四元数数学工具）

### ROS 通信（仅 ROS_MODE=True 时）
- Python `rospy` + `sensor_msgs/JointState` + `std_msgs/String`

---

## 四、环境要求

| 项目 | 要求 |
|------|------|
| Python | **3.9+**（推荐 3.10） |
| OS | Ubuntu 20.04+ / macOS / Windows 都可（ROS 部分仅 Ubuntu） |
| GPU | **不需要**（纯 numpy FK，无 GPU 依赖） |
| ROS（可选） | ROS1 Noetic（仅当 `ROS_MODE=True` 且要连实体机械臂时） |
| 网络 | 默认监听 `0.0.0.0:5000`，局域网内可通过 IP 访问 |

---

## 五、快速启动

### 方式 1：纯前端演示（最简）
无需后端，调试 UI 用：
```bash
# 直接用浏览器打开
open ROBOT_UI/index.html        # macOS
# 或者起一个静态服务（避免 file:// 跨域问题）
cd ROBOT_UI && python -m http.server 8080
# 浏览器访问 http://localhost:8080
```
此时所有滑块工作于**演示模式（Demo Mode）**，前端的"已连接"按钮呈黄色。

### 方式 2：本地后端 + 仿真模式（推荐用于开发）

```bash
cd ROBOT_UI
pip install -r requirements.txt
python server.py                  # 默认 ROS_MODE=True，会优雅降级到仿真
```

启动后终端会打印：
```
╔═══════════════════════════════════════════╗
║     Robotic Arm Control — Backend       ║
╠═══════════════════════════════════════════╣
║  Mode   : Simulation          ║
║  URL    : http://localhost:5000           ║
╚═══════════════════════════════════════════╝
```

打开 http://localhost:5000 即可。

### 方式 3：连实体机械臂（ROS 模式）

1. 确保 Ubuntu 上 ROS 已启动：
   ```bash
   roscore
   ```
2. **修改 `server.py` 顶部开关**（默认就是 `True`）：
   ```python
   ROS_MODE = True  # ← 改成 True 才会启用 ROS Bridge
   ```
3. 启动后端：
   ```bash
   pip install -r requirements.txt
   python server.py
   ```
4. 确保你的机械臂 ROS 节点已经发布了标准 `sensor_msgs/JointState` 到：
   - 发布（机械臂 → 系统）：`/arm_status/joint_states`
   - 订阅（系统 → 机械臂）：`/arm_control/joint_commands`
   - 订阅（系统 → 机械臂）：`/arm_config`（JSON 字符串，机械臂物理参数）

### 方式 4：局域网跨机访问

让 Ubuntu 跑后端，Windows 浏览器访问：
```bash
# Ubuntu 端
python server.py    # 默认监听 0.0.0.0:5000
```
Windows 浏览器打开 `http://Ubuntu的IP:5000`（如 `http://192.168.1.100:5000`）。
若连不上，检查 Ubuntu 防火墙：
```bash
sudo ufw allow 5000
```

---

## 六、HTTP API 参考

所有接口前缀 `/api/*`，基础 URL `http://localhost:5000`。

### 6.1 关节参数

| Method | Endpoint | 说明 |
|--------|----------|------|
| GET | `/api/params` | 获取 8 个关节的当前值 + 量程范围 + 单位 |
| POST | `/api/params` | 修改关节值（ROS 模式下同时推到 `/arm_control/joint_commands`） |
| POST | `/api/reset` | 全部关节恢复到默认值（0/0/0/...） |

**POST `/api/params` 请求体**：
```json
{
  "joint_1": 45.0,
  "joint_2": -30.0,
  "joint_3": 0.0
}
```
后端会自动 clamp 到每个关节的 `[min, max]` 范围，返回：
```json
{
  "ok": true,
  "updated": ["joint_1", "joint_2", "joint_3"],
  "all": {"joint_1": 45.0, "joint_2": -30.0, ...}
}
```

**关节量程定义**（在 `server.py` 的 `JOINT_DEFINITIONS`）：

| Key | 中文名 | 量程 | 单位 | 默认 |
|-----|--------|------|------|------|
| `joint_1` | 底座旋转（Base Rotation） | [-180, 180] | deg | 0.0 |
| `joint_2` | 肩关节（Shoulder） | [-90, 90] | deg | 0.0 |
| `joint_3` | 肘关节（Elbow） | [-135, 135] | deg | 0.0 |
| `joint_4` | 腕俯仰（Wrist Pitch） | [-90, 90] | deg | 0.0 |
| `joint_5` | 腕翻滚（Wrist Roll） | [-180, 180] | deg | 0.0 |
| `joint_6` | 夹爪开合（Gripper） | [0, 100] | % | 0.0 |
| `joint_7` | 辅助轴 1（Aux 1） | [-90, 90] | deg | 0.0 |
| `joint_8` | 辅助轴 2（Aux 2） | [-90, 90] | deg | 0.0 |

> 💡 改量程或增减关节：编辑 `server.py` 的 `JOINT_DEFINITIONS`，同时需要同步 `index.html` 的 HTML 滑块数量。

### 6.2 机械臂配置（DH 参数）

| Method | Endpoint | 说明 |
|--------|----------|------|
| POST | `/api/arm_config` | 接收机械臂的 DH 配置 JSON，自动生成 URDF 并推送拓扑给前端 |
| GET | `/api/arm_config` | 获取上一次 POST 进来的配置 |

**POST `/api/arm_config` 请求体**（与 `example(1).json` 同结构）：
```json
{
  "arm_name": "my_arm",
  "units": {"length": "m", "mass": "kg"},
  "gravity": [0.0, 0.0, -9.80665],
  "dof": 2,
  "joints": {
    "joint1": {
      "type": "axial",
      "link": {
        "length": 0.18,
        "mass": 0.16,
        "center_of_mass": [0.09, 0.0, 0.0],
        "twist": 1.5708,
        "offset": 0.0
      },
      "actuator_mass": 0.055
    },
    "joint2": {
      "type": "tangential",
      "link": {
        "length": 0.15,
        "mass": 0.12,
        "center_of_mass": [0.075, 0.0, 0.0],
        "twist": 0.0,
        "offset": 0.0
      },
      "actuator_mass": 0.045
    }
  }
}
```

**字段含义**：

| 字段 | 含义 |
|------|------|
| `arm_name` | 机械臂名，会写进生成的 URDF 文件名 |
| `dof` | 自由度（关节数）。`joints` 中的关节数应等于 `dof` |
| `joints[].type` | `axial`（绕 z 旋转） / `tangential`（绕 x 旋转）——目前在 URDF 里都映射成 `revolute` |
| `joints[].link.length` | DH 参数 a（沿 x 偏移） |
| `joints[].link.twist` | DH 参数 α（绕 x 扭转角，单位 rad） |
| `joints[].link.offset` | DH 参数 θ 零位偏角（单位 rad） |
| `joints[].link.mass` | 连杆质量（kg） |
| `joints[].link.center_of_mass` | 质心坐标 [x, y, z] |
| `joints[].actuator_mass` | 舵机质量（合并到 link mass） |
| `gravity` | 重力加速度向量（默认 `[0, 0, -9.80665]`） |

后端收到后会：
1. 调用 `urdf_generator.generate_urdf(...)` 写入 `generated_urdfs/current.urdf`
2. 用纯 numpy 解析该 URDF，生成 Three.js 拓扑 JSON，通过 WebSocket 推送给前端
3. ROS 模式下还会发布到 `/arm_config`（`std_msgs/String`，JSON 字符串）

### 6.3 URDF 上传

| Method | Endpoint | 说明 |
|--------|----------|------|
| POST | `/api/load_urdf` | 上传 URDF 文件（multipart/form-data 的 `file=` 字段，或 raw XML body） |
| POST | `/api/sim/reload_urdf` | 重新加载 `generated_urdfs/` 下的 URDF（调试用） |

上例：
```bash
curl -F file=@my_arm.urdf http://localhost:5000/api/load_urdf
# 或：
curl -X POST --data-binary @my_arm.urdf http://localhost:5000/api/load_urdf
```

### 6.4 状态 & 系统

| Method | Endpoint | 说明 |
|--------|----------|------|
| GET | `/api/status` | 整体连接状态（ROS / 仿真 / last_update） |
| GET | `/` | 返回 `index.html`（前端单页） |
| GET | `/assets/<path>` | 静态资源（URDF / mesh / Three.js） |
| GET | `/assets/vendor/<path>` | vendor 静态资源 |

---

## 七、WebSocket 协议

**连接地址**：`ws://host:5000/ws`

浏览器连接后，会依次收到：

| 消息类型 (`type`) | payload | 频率 | 说明 |
|-------------------|---------|------|------|
| `subscribed` | `{channels: [...]}` | 握手 1 次 | 应答客户端的 subscribe 消息 |
| `arm_state` | `{data: {target: {...}, actual: {...}}}` | **20 Hz**（限流） | 8 个关节的目标值 + 实际值 |
| `mani_topology` | `{links: [...], base_url: string}` | URDF 上传/生成时 | 机器人 link/joint 几何拓扑，Three.js 渲染用 |

**前端 → 后端消息**（可选）：
```json
{"type": "params", "data": {"joint_1": 30, "joint_2": -10}}
{"type": "subscribe", "channels": ["arm_state", "mani_topology"]}
```

---

## 八、ROS 话题规范

如果你要写自己的 ROS 控制节点对接本系统，按以下规范：

### 8.1 话题清单

| 方向 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **系统 → 机械臂** | `/arm_control/joint_commands` | `sensor_msgs/JointState` | 关节目标角度（弧度，不是角度！） |
| **系统 → 机械臂** | `/arm_config` | `std_msgs/String` | DH 配置 JSON（`msg.data` 是完整 JSON 字符串） |
| **机械臂 → 系统** | `/arm_status/joint_states` | `sensor_msgs/JointState` | 关节实际角度（弧度） |

### 8.2 字段顺序

注意 `JointState.name` 的顺序要和系统一致：
```
['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8']
```
position 数组的元素数 == name 数组的元素数。

### 8.3 单位

> ⚠️ ROS 标准用**弧度**。本系统网页侧是**角度**，转换在后端自动完成：
> `弧度 = 角度 × π / 180`

### 8.4 示例：最简的 ROS 节点骨架（Python rospy）

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

pub = rospy.Publisher('/arm_status/joint_states', JointState, queue_size=10)

def on_cmd(msg):
    # msg.position 是弧度，发送到下位机驱动舵机
    # ... 驱动舵机 ...

    # 读取舵机实际位置（角度），转弧度后回发
    actual_pos_deg = [...]  # 读舵机当前位置
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = msg.name
    js.position = [d * 3.1415926 / 180.0 for d in actual_pos_deg]
    pub.publish(js)

rospy.Subscriber('/arm_control/joint_commands', JointState, on_cmd)
rospy.init_node('my_arm_driver')
rospy.spin()
```

---

## 九、前端界面区域说明（`index.html`）

打开页面后大致分为以下区域，从上到下：

1. **顶部状态栏**：连接状态徽章（绿色=已连ROS / 黄色=演示模式 / 灰色=连接中）、模式切换按钮
2. **左面板 — 关节滑块**（带颜色区分）：
   - 🔵 **蓝色** = 角度控制滑块（每个关节一行，含 target + actual 数值显示）
   - 🟠 **橙色** = 重力补偿（Kp + 前馈力矩 FF，仅在 ROS 模式显示）
3. **中部 — 配置面板**：机械臂物理参数卡片（质量、长度、质心…），可一键导出 JSON
4. **右面板 — 3D 视图**：
   - 鼠标拖动：旋转视角
   - 滚轮：缩放
5. **底部 — JSON 预览 / 历史日志**

> 💡 UI 改动只需要在 `index.html` 里改 CSS 变量（顶部 `:root { --accent: #00e5ff; ... }`）和对应 DOM 节点，无需任何构建步骤。

---

## 十、扩展开发指引

### 10.1 想加一个新的 REST 接口
直接编辑 `server.py`，在 `# ── Flask 路由 ──` 段加：
```python
@app.route('/api/my_new_endpoint', methods=['GET'])
def my_new_endpoint():
    return jsonify({"ok": True})
```
保存后**重启** `server.py`（Flask 不支持热重载）。前端通过 `fetch('/api/my_new_endpoint')` 调用。

### 10.2 想加一个新的 WebSocket 消息
在 `server.py` 里调用 `_ws_hub.push({"type": "my_msg", "data": ...})` 即可广播到所有客户端。

### 10.3 想改关节数量或量程
1. 改 `server.py` 的 `JOINT_DEFINITIONS` dict
2. 改 `index.html` 里对应滑块的 `min` / `max` / step
3. **重启后端**

### 10.4 想换 3D 模型（STL）
1. 把 STL 文件放到 `assets/<model_name>/meshes/`（与 URDF 同级，URDF 里 mesh 路径是相对路径）
2. URDF 放到 `assets/<model_name>/`
3. 前端按钮改 `/assets/<model_name>/<model>.urdf`

### 10.5 想自定义 URDF 生成规则
全部在 `urdf_generator.py` 一个文件里：
- `URDF_TEMPLATE`：URDF 整体框架
- `generate_link_xml(idx, def)`：单个 link XML 片段
- `generate_joint_xml(idx, prev_def, this_def)`：单个 joint XML 片段（用相邻两个 link 的 DH 参数拼 origin）

改完保存即可，下次 `POST /api/arm_config` 自动用新规则生成。

---

## 十一、故障排查表

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 浏览器点滑块无反应 | 处于"演示模式"且后端未启动 | 启动 `python server.py`，或接受演示模式 |
| 启动报错 `ModuleNotFoundError: No module named 'flask'` | 没装依赖 | `pip install -r requirements.txt` |
| 启动 ROS 模式后报 `ROS not available: No module named 'rospy'` | 没装 rospy（Ubuntu 默认有） | 见上一节方式 3，或保持 `ROS_MODE=True` 也能自动降级 |
| 前端"已连接"一直灰色 | WS 连不上 | 看浏览器 DevTools Console，确认后端 5000 端口监听、URL 用 `ws://` |
| 滑块拖动后机械臂不动 | ROS_MODE 没开 / 话题名不一致 | 检查 `server.py` 顶部 `ROS_MODE = True`，参考第八节确认话题名 |
| 滑块拖动后 green "actual" 数字不动 | 你的节点没发布 `/arm_status/joint_states` | 让 ROS 节点定期发布该话题 |
| `POST /api/arm_config` 后 3D 视图没变化 | 看后端 stdout 的 `[Arm Config]` 日志 | 可能是 urdf_generator.py 报错 |
| 局域网 Windows 连不上 Ubuntu | 防火墙 | `sudo ufw allow 5000` |

---

## 十二、`server.py` 内部模块速查（接手后端必读）

```
server.py
├── 1~10      头部注释（模块说明 + 使用方法）
├── 25~30     导入（flask, asgiref, uvicorn）
├── 33~34     ROS 模式开关（ROS_MODE = True）
├── 37~42     ROS 导入 + 检测（缺 rospy 时自动降级）
├── 45~85     ArmROSBridge 类（ROS 桥接）
├── 91~92     启动 ROS spinner 线程
├── 97~127    ArmState 类（线程安全内存状态）
├── 130~137   JOINT_DEFINITIONS（8个关节量程）
├── 142~220   Flask 路由（/api/params, /api/status, /api/arm_config,
│              /api/load_urdf, /api/sim/reload_urdf, /api/reset）
├── 226~250   静态资源（/assets/*, /assets/vendor/*）
├── 256~370   _build_topology_from_urdf()（纯 numpy URDF→拓扑 JSON）
├── 375~420   _rpy_to_R() / _R_to_quat_wxyz()（数学工具）
├── 425~445   _WSHub 类（WebSocket 客户端注册 + 广播）
├── 450~470   _apply_incoming_params() / _ws_send_loop() / _broadcaster()
├── 475~530   _ws_handler（ASGI WebSocket handler）
├── 535~550   WSDashApp（HTTP/WS 路由分发）
├── 555~584   入口（uvicorn 配置 + asyncio main）
```
