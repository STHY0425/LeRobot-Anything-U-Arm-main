
# `teleop_sim.py` 细框架（按模块拆开）

## 模块A：启动参数（入口）
- **代码位置**：文件底部 `if __name__ == "__main__":` 里的 `argparse`
- **它做什么**：选择 `--robot --scene --rate --serial-port`
- **当前问题**：没有 `--input mock/replay`，导致没硬件就很难推进
- **耿负责（运行负责人）**
  - 加 `--input mock/replay/serial` 这种输入模式开关（重点）
  - 确保 `choices=[...]` 和代码真实支持的机器人一致
- **宏配合**
  - 提供“支持机器人列表”和每个机器人的映射卡片（表B），让入口能列出来

## 模块B：仿真环境创建（让不同机器人模型能加载、能动）
- **代码位置**：`ServoTeleoperatorSim.__init__()` 里 `gym.make(...)` + `env.reset()`
- **它做什么**：创建 ManiSkill 环境，决定 action_space 是什么形状
- **耿负责**
  - 确保每个 `robot_uids` 都能正常 `gym.make`，失败要有友好报错
  - 输出关键打印：action_space 是多少维
- **宏负责**
  - 每个机器人 action 的维度/含义整理进映射卡片（表B）

## 模块C：相机视角（看得清楚，方便演示）
- **代码位置**：`_setup_camera_pose()` + `sapien_utils.look_at(...)`
- **它做什么**：设置 viewer 相机位置，让展示时看得清
- **耿负责**
  - 统一一个“好看的默认视角”，不同机器人尽量都能看清

## 模块D：输入源（Serial / Mock / Replay）——这是“没实体也能推进”的关键
### D1 串口输入（现在已有）
- **代码位置**：
  - 串口初始化：`__init__` 里 `serial.Serial(...)`
  - 零点标定：`_init_servos()`
  - 读角线程：`angle_stream_loop()`（写死 `num_joints=7`）
- **它做什么**：从串口读舵机 PWM → 转角度 → 转弧度 → 得到 `pose`
- **耿负责（运行负责人）**
  - 把 `num_joints=7` 改为 `N` 可配置（跟你们的 `pose[N]` 目标一致）
  - 串口读不到不要直接崩溃：改成“提示 + 自动切 mock 或继续等待”

### D2 Mock 输入（必须做，第一周就要）
- **代码位置**：需要在 `angle_stream_loop()` 里根据 `--input` 分支（目前没有）
- **它做什么**：不用串口，直接生成 `pose[N]`（比如正弦波/分关节轮流动）
- **耿负责**
  - 这是保证“任何人任何电脑都能跑”的核心

### D3 Replay 回放输入（第二周开始做）
- **它做什么**：从文件读取 `pose` 序列，保证每次运行结果一致（可复现）
- **耿负责**
  - 实现 `--input replay --replay-file xxx.npy`

## 模块E：线程与数据通道（保证不卡、不延迟）
- **代码位置**：
  - `arm_pos_queue = Queue(maxsize=1)`
  - `publish_arm_pos()` / `get_latest_arm_pos()`
  - `produce_thread`（读角线程）
  - `consume_thread`（控制线程）
- **它做什么**：始终只保留最新 pose，避免延迟堆积
- **耿负责**
  - 保证线程不会死、停机能正确清理（`stop_event`）
  - 出现异常不要整个程序直接挂掉（改成提示并继续/切 mock）

## 模块F：核心映射（pose → action）——平台化的核心资产
- **代码位置**：`convert_pose_to_action(self, pose)`
- **它做什么**：不同机器人 action 维度不同、关节顺序不同、方向不同、夹爪范围不同
- **宏负责（映射负责人）**
  - 把 if-else 整理成“映射配置表”（dict 也行）
  - 每新增一个机器人，先写映射卡片（表B），再写进配置表
- **耿配合**
  - 帮忙跑起来验证（看动作是否明显反向/错位）

## 模块G：夹爪映射（经常出错，要单独管）
- **代码位置**：`angle_to_gripper(angle_rad, pos_min, pos_max, angle_range=...)`
- **它做什么**：把“一个角度”变成“夹爪开合范围”
- **宏负责**
  - 每个机器人明确夹爪范围（min/max），写进映射卡片

## 模块H：仿真执行（env.step/render）
- **代码位置**：`teleop_sim_handler(action)` + `env.step(action); env.render()`
- **它做什么**：真正驱动仿真动作
- **耿负责**
  - 保证频率稳定（`rate`）和不卡（必要时降低渲染频率）

---

# 3. ManiSkill “只看关键”的建议清单（够用，不会被大库淹没）

做平台适配，只需要搞清 3 件事：
1) `robot_uids` 是怎么注册的  
2) 每个 robot 的 action_space 多少维、含义是什么  
3) `pd_joint_pos` 这种控制模式到底期望什么 action

因此建议只看这些（按重要程度）：

- **(1) 环境入口/注册**
  - `src/simulation/mani_skill/envs/__init__.py`
  - `src/simulation/mani_skill/agents/registration.py`
- **(2) 控制器（用的就是 pd_joint_pos）**
  - `src/simulation/mani_skill/agents/controllers/pd_joint_pos.py`
  - `src/simulation/mani_skill/agents/controllers/base_controller.py`
- **(3) 要支持的机器人（按目录找）**
  - `src/simulation/mani_skill/agents/robots/panda/`
  - `.../xarm6/`
  - `.../so100/`
  - `.../arx_x5/`
  - `.../piper/`
  - `.../widowx/`（如果 widowx250s 走这里）
  - `.../fetch_xlerobot/`（对应 x_fetch）

---

# 4. 两个人的任务对应关系

## 耿（负责“能跑、能切换、没硬件也能跑”）
对应模块：A、B、C、D、E、H
- **模块A（启动参数）**：加 `--input mock/replay/serial`、加 `--master-dof`（或 `--servo-ids`）
- **模块B（初始化）**：环境创建失败要提示清楚；robot 清单一致
- **模块C（输入源）**：把串口读不到改成“提示+切 mock”，不要崩
- **模块D（线程/队列）**：保证跑久不崩、
- **模块H（执行）**：保证频率和渲染不卡

**每周要交付**：
- 能跑的命令（至少 3 条）
- 当前支持机器人清单（至少 6 个）
- 运行截图/录屏（证明能切换机器人）

## 宏（负责“映射正确、配置化、可验证”）
对应模块：F、G + 关键 ManiSkill
- **模块F（convert_pose_to_action）**：把 if-else 整理成“映射配置表”
- **模块G（夹爪映射）**：每个机器人夹爪范围写清楚
- **关键 ManiSkill（第3节）**：查 action_space 含义，避免映射维度错

**每周要交付**：
- 映射卡片（表B）（每周至少新增/完善 2 个机器人）
- 映射配置表（dict）+ 简单检查输出（action_dim 是否匹配）
- “标准测试 pose 序列”（后期用于回放/验收）
