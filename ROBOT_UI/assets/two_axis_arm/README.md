# 两轴机械臂 URDF 测试方案

## 目标
验证 `ROBOT_UI/server.py` + `index.html` 输出的 JSON 能否在 ManiSkill 中驱动一台两轴机械臂。

## 整体数据链路

```
┌─────────────────┐    WS /ws     ┌──────────────────┐     ┌─────────────────┐
│  浏览器 index.html │ ──────────► │   server.py       │ ──► │ ManiSkill 仿真    │
│  (拖滑块)         │  {params}    │   (Flask+uvicorn)  │     │  TwoAxisArm      │
│                  │              │                  │     │                 │
│  JSON arm_config │  POST        │                  │     │  URDF:           │
│  → /api/arm_config│ ──────────► │                  │     │  joint_1 / joint_2 │
└─────────────────┘              └──────────────────┘     └─────────────────┘
                                          │
                                          │  ROS_MODE=True 时
                                          ▼
                                   /arm_control/joint_commands
                                   /arm_config (std_msgs/String)
```

> 关键：网页发的 JSON 形状不变，只是数值变了。本测试方案保持同样的数据流，
> 把接收端从"真机 ROS 节点"换成"ManiSkill 仿真环境"。

---

## 文件清单

| 文件 | 作用 |
|------|------|
| `ROBOT_UI/assets/two_axis_arm/two_axis_arm.urdf` | 两轴机械臂 URDF（参数对齐 example.json） |
| `src/simulation/mani_skill/assets/robots/two_axis_arm/two_axis_arm.urdf` | 同上，mani_skill 内部加载需要 |
| `src/simulation/mani_skill/agents/robots/two_axis_arm/two_axis_arm.py` | ManiSkill 的 `TwoAxisArm` agent 类 |
| `src/simulation/mani_skill/agents/robots/two_axis_arm/__init__.py` | Python 包入口 |
| `ROBOT_UI/test_two_axis_from_web.py` | 测试脚本 1：WebSocket 方式订阅 /ws |
| `ROBOT_UI/server.py`（已扩展 `/api/sim/two_axis_step`） | 新增两轴专用 step 端点 |

---

## 字段对照（example.json → URDF）

| example.json 字段 | URDF 取值 | 说明 |
|-----------------|----------|------|
| `joints.joint1.type` | revolute | axial/tangential 只是 UI 标识，仿真都按 revolute 处理 |
| `joints.joint1.link.d` | `joint_1` origin xyz z=0.08 | DH 偏距 d=0.08m |
| `joints.joint1.link.alpha` | `joint_1` origin rpy pitch=π/2 | DH 扭转角 α=π/2 |
| `joints.joint1.link.theta` | `joint_1` 隐含零位=0 | DH 零位偏角 θ=0 |
| `joints.joint1.link.mass` | `link_1` mass=0.16 | 连杆质量 0.16kg |
| `joints.joint1.link.center_of_mass` | `link_1` inertial origin xyz=(0,0,0.04) | 质心 |
| `joints.joint1.actuator_mass` | _未单独建模_ | 简化到 link mass 上（仿真精度要求不高） |
| `joints.joint2.link.l` | `joint_2` origin xyz x=0.15 | DH 杆长 l=0.15m |
| `joints.joint2.link.alpha` | `joint_2` origin rpy=0 | α=0 |
| `joints.joint2.link.theta` | `joint_2` origin rpy yaw=π/2 | θ=π/2 |
| `joints.joint2.link.mass` | `link_2` mass=0.12 | 0.12kg |
| `joints.joint2.link.center_of_mass` | `link_2` inertial origin xyz=(0.075,0,0) | 杆中点 |

关节名 `joint_1` / `joint_2`（带下划线）——与 server.py 转 ROS 的命名一致，无须翻译。

---

## 测试方法

### 方案 A：纯 WebSocket（推荐）

#### 1. 启动 server.py
```bash
cd ROBOT_UI
python server.py
```
浏览器打开 `http://localhost:5000`，把 DOF 设为 2，导入 `example.json`，拖动滑块。

#### 2. 另开终端，跑测试脚本
```bash
cd ROBOT_UI
pip install websockets   # 首次需要
python test_two_axis_from_web.py
```

输出示例：
```
[Init] 已加载 agent: uid=two_axis
[Init] active joints: ['joint_1', 'joint_2']
[Init] tcp_pos (rest): [0.300, 0.000, 0.080]
[WS] 连接到 ws://localhost:5000/ws ...
[WS] 已连接，等待网页推送关节角...
[step 0001] target(deg)=[  +0.0,   +0.0] qpos(rad)=[+0.000, +0.000] tcp=[+0.300, +0.000, +0.080]
[step 0002] target(deg)=[ +30.0,   +0.0] qpos(rad)=[+0.524, +0.000] tcp=[+0.260, +0.150, +0.080]
```

### 方案 B：HTTP POST（更直接）

直接 POST 到 server.py 的两轴专用端点：
```bash
curl -X POST http://localhost:5000/api/sim/two_axis_step \
     -H 'Content-Type: application/json' \
     -d '{"angles_deg": [30, -15]}'
```

返回：
```json
{
  "ok": true,
  "mode": "real",
  "angles_deg": [30, -15],
  "tcp": [0.260, 0.150, 0.080],
  "qpos_rad": [0.524, -0.262]
}
```

订阅 WS 广播：
```js
ws.onmessage = (evt) => {
  const msg = JSON.parse(evt.data);
  if (msg.type === 'two_axis_state') {
    console.log('tcp:', msg.data.tcp);
  }
};
```

---

## 落库位置 & 路径约定

ManiSkill 通过 `urdf_path = f"{PACKAGE_ASSET_DIR}/robots/two_axis_arm/two_axis_arm.urdf"` 加载，
其中 `PACKAGE_ASSET_DIR = src/simulation/mani_skill/assets/`。
所以 URDF 必须放在 `src/simulation/mani_skill/assets/robots/two_axis_arm/`。

`server.py` 通过 `_setup_local_mani_path()` 把 `src/simulation` 注入 sys.path，
所以 `from mani_skill.agents.robots.two_axis_arm import TwoAxisArm` 能直接 import。

---

## 可行性结论

**测试方案完全可行**，原因：

1. ✅ URDF 物理参数与 example.json 一一对应（`mass`/`center_of_mass`/`DH`），验证 JSON ↔ URDF 翻译一致
2. ✅ 关节名 `joint_1`/`joint_2` 与 server.py ROS 端命名一致，无需名字翻译层
3. ✅ ManiSkill `TwoAxisArm` 已仿照 SO100 模板实现，含 PD 关节位置控制器
4. ✅ 通信链路有 2 条（A: WebSocket, B: HTTP POST），互为备份
5. ✅ 即便 GPU/驱动不可用，fallback 2D 关节图模式也保留链路完整
6. ✅ 末端 `wrist_tip` link 可作为 tcp 抓取点

---

## 已知限制 / 待扩展

- `actuator_mass`（执行器质量）未单独建模，简化合到 link mass 上。如要精确，可为每个 joint 加一个 fixed 虚拟 link 承载执行器质量
- 转动惯量用细杆公式近似，仅满足仿真稳定性，不追求物理精确
- `_default_reset_human_pose` 用默认初始姿态；如需精确零位，可在 keyframe.rest 中指定
- 浏览器 `arm_state` 仍发 8 个 joint（joint_1..joint_8），未与 `dof` 联动裁剪；本测试脚本只取前两个，可工作