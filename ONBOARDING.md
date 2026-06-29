# 项目快速上手提示词

你正在参与一个 ROS1 机械臂阻尼控制工程项目。请先执行以下步骤，快速掌握工程全貌后再接受任务。

## 第一步：阅读核心文件

按顺序阅读：

1. `hxj_duoji_node.py` — 唯一核心源文件，全部逻辑在这一个文件里
2. `config/example.json` — 机械臂参数配置，是所有逻辑的数据源头
3. `tests/test_hxj_duoji_node.py` — 单元测试，反映代码实际行为
4. `fashionstar_servo_python_sdk_api.md` — 舵机官方 SDK API 文档

## 第二步：把握模块化核心思想

这个工程的灵魂是 **JSON 驱动的模块化可塑性 + 3D 动态阻尼**。以下是设计层面的核心原则：

### 1. 数据驱动，不硬编码
机械臂有多少轴、每个关节是什么类型、杆长多少——全部由 `config/example.json` 决定。`servo_ids` 从 JSON 的 `dof` 动态派生。所有阻尼参数默认值集中在文件顶部的 `DEFAULTS` 字典，改值只改一处。

### 2. 轴向/切向解耦
关节按 `type` 字段分两类，处理方式完全不同：
- **轴向（axial）**：α=π/2，参与 3D 运动学但阻尼固定 `base_damping_power`，不参与末端阻尼分发。
- **切向（tangential）**：α=0，参与 3D 运动学和阻尼分发。

### 3. 3D 运动学包含所有关节
所有关节（含轴向）都在 3D DH 表中，确保底座/中间 axial 旋转后末端位置和速度准确。顺逆重力判断用末端速度的 z 分量 vz 正负。

### 4. 阻尼是功率，不是力矩
`set_damping(id, power)` 只能延缓角度变化。三档末端合阻尼预算（固定值）按雅可比列范数权重分发到切向关节。

### 5. 三档固定预算 + 雅可比权重分发
- `end_damping_low=600` — 逆重力（上抬），固定低预算
- `end_damping_mid=1000` — 水平/静止，固定中预算
- `end_damping_high=1500` — 顺重力（下坠），固定高预算
- 切向关节按 3D 雅可比列范数权重分发所选预算，杠杆高的关节分到更多
- 轴向关节固定 `base_damping_power=500`

### 6. DH 参数包含中间轴向杆长
两个切向关节之间夹着的轴向关节的连杆长度累加进 DH 参数 `a`。

### 7. 硬件限制就是限制
单舵机阻尼上限 1000mW，到达后截断。零点安装时校准，代码层面不操心。

### 8. 单文件三职责
`ServoConfig`（配置）+ `RosPublisher`（ROS 发布）+ `Controller`（控制+阻尼解算）。线程间用共享字典 + 互斥锁通信。

### 9. 状态机
`IDLE` → `HOLD`（阻尼控制，核心）→ `LOCKED`（锁死，flag 触发，后续接传感器）→ `ERROR`。`LOCKED` 用 `stop_on_control_mode(method=0x11)` 保持锁力。

## 第三步：运行环境

- WSL2 Ubuntu 26.04，Docker 容器 `ros1-noetic`（ROS Noetic + Python 3.8 + numpy）
- 工作空间挂载：`~/ros-workspaces/ros1_ws` → 容器内 `/root/ros1_ws`
- git 分支：WH
- 中文注释，中文回复

运行测试：
```bash
docker exec ros1-noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/ros1_ws/LRrobot/src/LeRobot-Anything-U-Arm-main && PYTHONPATH=.:$PYTHONPATH python3 -m unittest tests.test_hxj_duoji_node -v'
```

## 第四步：确认理解

读完文件后，用你自己的话简述：
1. 为什么轴向关节在运动学里但不在阻尼分发里？
2. 三档固定预算是怎么切换的？切向关节怎么分到这个预算？
3. 这个工程怎么实现 3-7 轴自适应的？

然后说"我已理解，请给任务。"
