# 🦾 LeRobot Anything U-Arm — Follower Robots User Guide

This guide introduces how to use **U-Arm** as a teleoperation controller for various robots in the **LeRobot Series**, including **SO-100**, **Lekiwi**, and **XLeRobot**.

---

## 📁 File Structure

```
LeRobot/
├── lekiwi/              # Lekiwi robot related code
├── so100/               # SO-100 follower arm related code
├── xlerobot/            # XLeRobot related code
├── uarm.py              # U-Arm teleoperator module
└── README.md            # This user guide
```

---

## ⚙️ Quick Start

### 1. Environment Setup

You can directly use the **LeRobot environment** — **no need for ROS**!

🔗 Installation guide: [LeRobot Official Repository](https://github.com/huggingface/lerobot/blob/main/README.md)

> **Note:** The `[feetech]` feature is required.

---

### 2. File Placement

Move the `uarm.py` file into your LeRobot teleoperator directory:

```bash
mv uarm.py ${PATH_TO_YOUR_LEROBOT}/lerobot/src/lerobot/teleoperators/
```

---

## 🚀 Script Usage

### 🦾 SO-100

Use `so100_teleop.py` to control the **SO-100** follower arm via **U-Arm**.

**Usage:**

```bash
python so100_teleop.py
```

---

### 🤖 Lekiwi

Before using Lekiwi teleoperation, replace the original Lekiwi robot file with our modified version:

```bash
${PATH_TO_YOUR_LEROBOT}/lerobot/src/lerobot/robots/lekiwi/lekiwi.py
```

This modified file **adds motion control support** for Lekiwi.

Then, run:

```bash
python lekiwi_teleop.py
```

#### 🕹️ Default Keyboard Mapping

| Key   | Action        |
| ----- | ------------- |
| **w** | Move forward  |
| **s** | Move backward |
| **a** | Move left     |
| **d** | Move right    |
| **z** | Rotate left   |
| **x** | Rotate right  |
| **r** | Speed up      |
| **f** | Slow down     |
| **q** | Quit teleop   |

You can customize these keys in:

```
${PATH_TO_YOUR_LEROBOT}/lerobot/src/lerobot/robots/lekiwi/config_lekiwi.py
```

---

### 🧠 XLeRobot

Use `xlerobot_teleop.py` to control **XLeRobot** by **U-Arm**.

**Usage:**

```bash
python xlerobot_teleop.py
```

#### 🕹️ Default Keyboard Mapping

| Key          | Action               |
| ------------ | -------------------- |
| **i**        | Move forward         |
| **k**        | Move backward        |
| **j**        | Move left            |
| **l**        | Move right           |
| **u**        | Rotate left          |
| **o**        | Rotate right         |
| **n**        | Speed up             |
| **m**        | Slow down            |
| **b**        | Quit teleop          |
| **<**, **>** | Head motor 1 control |
| **,**, **.** | Head motor 2 control |

You can customize:
* Base control mappings in
  `${PATH_TO_YOUR_LEROBOT}/lerobot/src/lerobot/robots/xlerobot/config_xlerobot.py`
* Head control mappings in
  `HEAD_KEYMAP` in `xlerobot_teleop.py`

> ⚠️ **Note:** This teleoperation module is still under testing. Unexpected behavior or bugs may occur — please use cautiously.

---

## 💡 Additional Tips

* Always set the **U-Arm port** and **follower robot port** correctly in each `*_teleop.py` script.
* If you are operating **alone** and want to use **XLeRobot**’s mobility function, we recommend connecting an **SO-100 arm and base** together, then treating it as a **Lekiwi** robot for control.
