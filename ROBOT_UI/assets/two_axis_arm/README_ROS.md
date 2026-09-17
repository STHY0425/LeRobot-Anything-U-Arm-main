# ROS 链路 + ManiSkill 仿真 测试方案

## 完整数据流

```
网页 (index.html)        server.py               ROS 话题           本脚本
拖滑块                    (ROS_MODE=True)        (master)            (本节创建的)
                                                ┌─────────────────┐
WS /ws  ─── params ───►  apply_incoming_params   │ /arm_config       │ ◄──── POST /api/arm_config
                       │                        │   std_msgs/String │
                       ▼                        │   (JSON 字符串)   │
                  _ros_node.publish_arm_config()│                   │
                       │                        │ /arm_control       │
                       ▼                        │   /joint_commands │
                  /arm_config ─────────────────►│   sensor_msgs      │
                       │                        │   /JointState     │
                       │ _ros_node.publish()    │                   │
                       ▼                        │                   │
                  /arm_control/joint_commands ─►│                   │
                                                └─────────────────┘
                                                         │
                                                         ▼
                                            two_axis_ros_listener.py
                                            (订阅两边 → ManiSkill step)
```

## 文件清单（本轮新增）

| 文件 | 作用 |
|------|------|
| `ROBOT_UI/urdf_generator.py` | 根据 `/arm_config` JSON 动态生成 URDF |
| `ROBOT_UI/two_axis_ros_listener.py` | ROS1 节点：订阅 /arm_config + /arm_control/joint_commands，驱动 ManiSkill |
| `ROBOT_UI/ros_topic_smoke_test.py` | 仅订阅话题打日志，验证链路（不依赖 ManiSkill） |
| `ROBOT_UI/generated_urdfs/current.urdf` | 运行时生成的 URDF（每次 arm_config 变化时刷新） |

> **URDF 不再写死为 2 轴**。`urdf_generator.py` 接受任意 DOF（1..7）、任意 DH/质量/质心，
> 当 `/arm_config` 触发时**实时重建 URDF 并 reload ManiSkill 环境**。

---

## 操作步骤

### 0. 前置

每开一个终端都要 source ROS：
```bash
source /opt/ros/<你的版本>/setup.bash
```

roscore 也要跑起来：
```bash
roscore &
```

### 1. 启动 server.py（开 ROS 模式）

**终端 A**：
```bash
source /opt/ros/noetic/setup.bash
cd ROBOT_UI
python3 server.py
```

看到：
```
[INFO] ROS mode enabled — bridge active.
  ║  Mode   : ROS             ║
  ║  URL    : http://localhost:5000
```
说明 OK。

### 2. （可选）开一个 smoke test 验证话题

**终端 B**：
```bash
source /opt/ros/noetic/setup.bash
cd ROBOT_UI
python3 ros_topic_smoke_test.py
```

预期看到：
```
[SmokeTest] 订阅 /arm_config 和 /arm_control/joint_commands ...
```

### 3. 浏览器操作

`http://localhost:5000`：
1. DOF 设为 **2**（或 3 看心情）
2. 加载 `ROBOT_UI/example.json`
3. 点 "📤 发送配置到后端" → 触发 `/arm_config` 话题
4. 拖 `joint_1` 滑块 → 触发 `/arm_control/joint_commands`

**终端 B** 应当实时打印：
```
[CFG #1] arm_name=example, dof=2
[CMD #20] name=['joint_1', 'joint_2', 'joint_3', ..., 'joint_8'], position(原始单位)=[...]
[CMD #21] ...
[CFG #2] arm_name=example, dof=3   ← 如果改了 DOF
```

> ⚠️ 注意 `position` 字段其实是 **度**（不是对接文档说的弧度）——
> 这是 server.py 当前的代码行为，`two_axis_ros_listener.py` 已经做了自动识别（>6.3 视为度）。

### 4. 启动 ROS → ManiSkill 仿真节点

**终端 C**：
```bash
source /opt/ros/noetic/setup.bash
cd ROBOT_UI
python3 two_axis_ros_listener.py
```

预期看到：
```
[Node] two_axis_maniskill_listener 已启动
[URDF 输出目录: .../generated_urdfs]
[Mani] 默认环境已就绪，dof=?, joints=[...]
```

然后浏览器发 `/arm_config` 后会看到：
```
[CFG] 收到 arm_config: arm_name=example, dof=2, joints=['joint1','joint2']
[Mani] URDF 写入 .../generated_urdfs/current.urdf, dof=2
[Mani] 重载完成, dof=2
```

拖滑块后：
```
[STEP unit_in=deg] qpos(deg)=[30.0, 0.0] tcp=[0.235, 0.150, 0.080]
[STEP unit_in=deg] qpos(deg)=[45.0, 15.0] tcp=[...some change...]
```

✅ 测试通过：看到 `qpos` 与网页 slider 一致、`tcp` 在变。

---

## 字段映射（与 example.json 对照）

| example.json 字段 | 进入 ROS 话题 | ManiSkill 仿真位置 |
|-----------------|--------------|------------------|
| `arm_name` | `/arm_config` JSON | URDF robot name |
| `dof` | `/arm_config` JSON | URDF link/joint 数量 |
| `joints[i].type` (axial/tangential) | `/arm_config` JSON | **丢弃**（UI 分类，仿真都用 revolute） |
| `joints[i].link.l` | `/arm_config` JSON | URDF joint origin xyz.x 或 visual x 偏 |
| `joints[i].link.d` | `/arm_config` JSON | URDF joint origin xyz.z 或 visual z 偏 |
| `joints[i].link.alpha` | `/arm_config` JSON | URDF rpy roll |
| `joints[i].link.theta` | `/arm_config` JSON | URDF rpy yaw |
| `joints[i].link.mass` | `/arm_config` JSON | URDF inertial mass（加上 actuator_mass） |
| `joints[i].link.center_of_mass` | `/arm_config` JSON | URDF inertial origin |
| `joints[i].actuator_mass` | `/arm_config` JSON | 合并到 link mass |
| 滑块角度（度） | `/arm_control/joint_commands.position` | listener 自动识别度/弧度，转弧度喂给 ManiSkill |

---

## 已知限制 / 待优化

1. **server.py 当前把 slider 值当度直接发（与对接文档写的"弧度"不一致）**。listener 已做启发式补偿，但如果将来 server.py 改成发弧度，记得把 listener 里 `>6.3` 的判断去掉。

2. **ManiSkill 必须装好**（`pip install mani_skill torch`）。如未装或 GPU 不可用，listener 不会报错，但 `[Mani] 默认环境初始化失败` 会打很多日志，CFG 与 STEP 都不会工作。

3. **URDF 重建时 env.close() + 重 create** 有 ~1s 卡顿；如果快速连发 arm_config 可能错过中间状态。可考虑加节流（已加 `_last_urdf_signature`，只有内容真变了才 reload）。

4. **关节轴方向 hardcode**：
   - joint_1  → `axis 0 0 1`（axial 语义：基座绕 z 旋转）
   - 其余       `axis 1 0 0`（tangential：水平面内俯仰）
   如要 `joints[i].type` 真实影响 axis，需要更复杂的 SH 解析（在生成器里加分支）。

5. **actuator_mass 简化合并到 link mass**。如要严格区分，每个 joint 加一个 fixed `actuator_link`。

6. **网页目前发 8 个关节（joint_1..joint_8）但 DOF 是 2**。listener 取名字出现在 arm_config 里的关节。slider 多余的会被忽略。

---

## 退出顺序

- 浏览器关闭或停止拖动 → 终端 B 和 C 即可慢慢退出。
- Ctrl+C 终端 C（ManiSkill）：会跳过 `env.close()` 因为 ROS spin 可能不响应 signal；建议直接 `kill -INT pid` 或 close 终端。
- Ctrl+C server.py → 清理 ROS 节点。
- Ctrl+C roscore。

---

## 如果跑不通怎么办

| 现象 | 排查 |
|------|------|
| server.py 报 `No module named 'rospy'` | 终端未 source ROS；或需要 `pip install rospkg` |
| smoke test 收不到 `/arm_config` | 检查浏览器是否点了"发送配置"；rostopic list 看看话题在不在 |
| smoke test 收不到 `/arm_control/joint_commands` | 是否拖了滑块；server.py 默认 5Hz 推送一次（arm_state 广播），但只有你 set 时才会 publish |
| ManiSkill 节点启动报 `[Mani] 默认环境初始化失败: No module named 'torch'` | `pip install torch` |
| ManiSkill 节点报 `[Mani] step 失败: ...` | 看末尾栈；最常见是关节 active joints 数量与 action 长度不匹配（listener 已按名字对齐，应不会出） |
| tcp 一直 0 | ManiSkill fallback 没真仿真；或 env 不支持物理步进（如 cpu/cpu_backend 配置问题），可改成 `sim_backend="physx_cuda"`（如机器有 GPU） |