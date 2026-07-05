#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按键锁定/解锁线程模块。

本模块将原 hxj_servo_key_hold.py 的按键逻辑改写成独立线程类，
可直接嵌入 hxj_duoji_node.py 的三线程架构中，作为第四个线程运行。

设计原则：
- 不访问串口，不操作舵机 SDK — 这些由 Controller 线程独占。
- 不初始化 ROS 节点 — 由 main() 统一初始化。
- 通过 controller.lock_requested / controller.unlock_requested 标志
  与 Controller 通信，触发 HOLD ↔ LOCKED 状态切换。
- 只做键盘监听和边沿检测，职责单一。

按键检测双模式：
1. evdev 模式（优先）：直接读 /dev/input/event* 的 EV_KEY 事件，
   获得真实的物理按下/松开，零 timeout、零抖动误判。
   需要 /dev/input/event* 读权限（用户加入 input 组）。
2. stdin 回退模式：终端字符流 + 双相自适应 timeout。
   首键阶段用长 timeout 熬过 OS 首键延迟，持续阶段用短 timeout 快速检测松手。
   不需要额外权限，但本质上仍是推断，高负载下可能误判。

使用方法（在 hxj_duoji_node.py 的 main() 中）：

    from key_hold_thread import KeyHoldThread

    key_thread = KeyHoldThread(
        controller=controller,
        shared_state=shared_state,
        state_lock=state_lock,
        stop_event=stop_event,
    )
    t = start_thread("key_hold_thread", key_thread.run, ())
"""

import fcntl
import glob
import os
import select
import struct
import sys
import termios
import time
import tty


# ===== stdin 回退模式：双相 timeout =====

# 首键阶段 timeout（秒）。
# OS auto-repeat 的首键延迟通常 250ms（xset r rate 250 40），
# 此值必须 > 首键延迟，否则第一个字符到重复之间的间隙会被误判为松开。
INITIAL_TIMEOUT = 0.5

# 持续阶段 timeout（秒）。
# 收到 2+ 个字符后切换到此值。auto-repeat 间隔约 25ms，
# 此值远大于 25ms 足以容忍抖动，同时比首键阶段短得多，松手后能快速响应。
SUSTAINED_TIMEOUT = 0.08

# 松开抖动冷却期（秒）。
# 下降沿（松开→锁定）触发后，在此时间内忽略所有 stdin 输入，
# 让 OS auto-repeat 残留的 'k' 字符被静默 drain 掉，不触发误 unlock。
# auto-repeat 在松手后最多还会发几个字符（<50ms 内排空），0.15s 足够覆盖。
RELEASE_DEBOUNCE = 0.15

# 旧字段名兼容（外部可能引用 DEFAULT_HOLD_TIMEOUT）。
DEFAULT_HOLD_TIMEOUT = INITIAL_TIMEOUT


# ===== 通用配置 =====

# 默认监听的按键字符（stdin 模式）。
DEFAULT_HOLD_KEY = "k"

# evdev KEY_K 的 keycode（linux/input-event-codes.h）。
KEY_K_CODE = 45

# evdev 事件类型。
EV_KEY = 0x01

# evdev input_event 结构体格式（64-bit Linux）。
# struct input_event { struct timeval time; __u16 type; __u16 code; __s32 value; }
# timeval = two longs = 16 bytes on 64-bit.
EVDEV_EVENT_FORMAT = "llHHI"
EVDEV_EVENT_SIZE = struct.calcsize(EVDEV_EVENT_FORMAT)

# 键盘监听轮询频率（Hz）。
DEFAULT_POLL_RATE = 50


class KeyHoldThread:
    """按键锁定/解锁线程。

    监听键盘输入，检测 'k' 键的按住/松开边沿：
    - 按住 'k'：设置 controller.unlock_requested = True
      → Controller 状态机在下一周期 LOCKED → HOLD
    - 松开 'k'：设置 controller.lock_requested = True
      → Controller 状态机在下一周期 HOLD → LOCKED

    本线程不直接修改 shared_state["control_state"]，
    而是通过 Controller 已有的 flag 让状态机自己切换，保持状态机的唯一入口不变。
    """

    def __init__(
        self,
        controller,
        shared_state,
        state_lock,
        stop_event,
        hold_timeout=DEFAULT_HOLD_TIMEOUT,
        hold_key=DEFAULT_HOLD_KEY,
        poll_rate=DEFAULT_POLL_RATE,
        rospy=None,
    ):
        """初始化按键线程。

        参数:
            controller:   Controller 实例，用于设置 lock_requested / unlock_requested。
            shared_state: 线程共享状态字典（只读，用于判断当前状态）。
            state_lock:   threading.Lock，读 shared_state 时加锁。
            stop_event:   threading.Event，外部停止信号。
            hold_timeout: 首键阶段松开判定阈值（秒），默认 0.5s。
                          持续阶段自动切换到 SUSTAINED_TIMEOUT。
            hold_key:     监听的按键字符，默认 'k'。
            poll_rate:    键盘轮询频率（Hz），默认 50。
            rospy:        rospy 模块引用，用于日志输出（可选）。
        """
        self.controller = controller
        self.shared_state = shared_state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.hold_key = hold_key
        self.poll_rate = int(poll_rate)
        self.rospy = rospy

        # stdin 模式：双相 timeout。
        self.initial_timeout = float(hold_timeout)
        self.sustained_timeout = SUSTAINED_TIMEOUT
        self.current_timeout = self.initial_timeout

        # 键盘状态。
        self._orig_termios = None
        self.last_key_time = 0.0
        self.key_was_held = False
        self._key_char_count = 0
        # 下降沿冷却期截止时间戳，>0 时忽略 stdin 输入。
        self._release_cooldown_until = 0.0

        # evdev 模式。
        self.use_evdev = False
        self._evdev_fds = []

    # ===== evdev 模式 =====

    def _evdev_ioctl_supported(self, fd):
        """用 EVIOCGBIT 检查设备是否支持 KEY_K。"""
        # EVIOCGBIT(ev_type, size) = _IOR('E', 0x20 + ev_type, size)
        IOC_NRBITS = 8
        IOC_TYPEBITS = 8
        IOC_SIZEBITS = 14
        IOC_DIRBITS = 2
        IOC_NRSHIFT = 0
        IOC_TYPESHIFT = IOC_NRSHIFT + IOC_NRBITS
        IOC_SIZESHIFT = IOC_TYPESHIFT + IOC_TYPEBITS
        IOC_DIRSHIFT = IOC_SIZESHIFT + IOC_SIZEBITS
        IOC_READ = 2
        nr = 0x20 + EV_KEY
        buf_size = 96
        ioc = (
            (IOC_READ << IOC_DIRSHIFT)
            | (0x45 << IOC_TYPESHIFT)
            | (nr << IOC_NRSHIFT)
            | (buf_size << IOC_SIZESHIFT)
        )
        buf = bytearray(buf_size)
        try:
            fcntl.ioctl(fd, ioc, buf)
        except OSError:
            return False
        byte_idx = KEY_K_CODE // 8
        bit_idx = KEY_K_CODE % 8
        return bool(buf[byte_idx] & (1 << bit_idx))

    def try_setup_evdev(self):
        """尝试打开 evdev 键盘设备。成功返回 True。

        遍历 /dev/input/event*，用 EVIOCGBIT 检查哪些设备支持 KEY_K，
        把所有可读的键盘设备都打开（笔记本内置键盘 + 外接键盘都监听）。
        """
        paths = sorted(glob.glob("/dev/input/event*"))
        for path in paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                continue
            if self._evdev_ioctl_supported(fd):
                self._evdev_fds.append(fd)
            else:
                os.close(fd)
        if self._evdev_fds:
            self.use_evdev = True
            return True
        return False

    def read_evdev(self):
        """读取 evdev 事件，返回 'press' / 'release' / None。

        只关注 EV_KEY + KEY_K 的事件：
        - value=1 → 物理按下
        - value=0 → 物理松开
        - value=2 → auto-repeat（忽略）
        """
        if not self._evdev_fds:
            return None
        try:
            readable, _, _ = select.select(self._evdev_fds, [], [], 0)
        except (OSError, ValueError):
            return None
        for fd in readable:
            while True:
                try:
                    data = os.read(fd, EVDEV_EVENT_SIZE)
                except (OSError, BlockingIOError):
                    break
                if len(data) < EVDEV_EVENT_SIZE:
                    break
                _sec, _usec, ev_type, code, value = struct.unpack(
                    EVDEV_EVENT_FORMAT, data
                )
                if ev_type == EV_KEY and code == KEY_K_CODE:
                    if value == 1:
                        return "press"
                    if value == 0:
                        return "release"
        return None

    def close_evdev(self):
        """关闭所有 evdev 文件描述符。"""
        for fd in self._evdev_fds:
            try:
                os.close(fd)
            except OSError:
                pass
        self._evdev_fds = []

    # ===== stdin 模式 =====

    def setup_keyboard(self):
        """将终端设置为 cbreak 模式，实现单字符无回车读取。"""
        try:
            if sys.stdin.isatty():
                self._orig_termios = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self._log_info(
                    "键盘 cbreak 模式已设置（按住 '%s' 解锁，松开锁定）",
                    self.hold_key,
                )
        except Exception as exc:
            self._log_warn("键盘设置失败（可能在非交互环境）: %s", exc)

    def restore_keyboard(self):
        """恢复终端到原始模式。"""
        if self._orig_termios is not None:
            try:
                termios.tcsetattr(
                    sys.stdin, termios.TCSADRAIN, self._orig_termios
                )
                self._orig_termios = None
            except Exception:
                pass

    def check_keyboard(self):
        """非阻塞读取 stdin 缓冲区，更新按键时间戳和字符计数。

        下降沿冷却期内（_release_cooldown_until > now）静默 drain
        缓冲区残留字符，不更新 last_key_time，避免 auto-repeat 残留
        触发误 unlock。
        """
        try:
            if not sys.stdin.isatty():
                return
            now = time.time()
            in_cooldown = now < self._release_cooldown_until
            has_key = False
            while True:
                dr, _, _ = select.select([sys.stdin], [], [], 0)
                if dr:
                    ch = sys.stdin.read(1)
                    if ch == self.hold_key:
                        has_key = True
                else:
                    break
            if in_cooldown:
                # 冷却期：静默吞掉残留字符，不更新时间戳。
                return
            if has_key:
                self.last_key_time = time.time()
                self._key_char_count += 1
                # 收到 2+ 个字符 → 切换到持续阶段（短 timeout，快速检测松手）。
                if self._key_char_count >= 2:
                    self.current_timeout = self.sustained_timeout
        except Exception:
            pass

    def is_key_held(self):
        """判断当前是否处于"按键按住"状态（双相 timeout）。"""
        held = (time.time() - self.last_key_time) < self.current_timeout
        if not held:
            # 超时 → 按键已松开，重置到首键阶段。
            self._key_char_count = 0
            self.current_timeout = self.initial_timeout
        return held

    # ===== 状态切换接口 =====

    def request_lock(self):
        """请求锁定：设置 Controller 的 lock_requested 标志。"""
        self.controller.lock_requested = True
        self._log_info(">>> 松开按键：请求锁定（HOLD → LOCKED）")

    def request_unlock(self):
        """请求解锁：设置 Controller 的 unlock_requested 标志。"""
        self.controller.unlock_requested = True
        self._log_info("<<< 按住按键：请求解锁（LOCKED → HOLD）")

    def get_current_state(self):
        """安全读取当前控制状态。"""
        self.state_lock.acquire()
        try:
            return self.shared_state["control_state"]
        finally:
            self.state_lock.release()

    # ===== 主循环 =====

    def run(self):
        """按键线程主循环。

        优先尝试 evdev 模式（真实按键事件），失败则回退到 stdin 双相 timeout。
        两种模式都做边沿检测：只在按下/松开状态变化的那一刻触发请求。
        """
        poll_delay = 1.0 / float(self.poll_rate)
        keyboard_ready = False

        try:
            # 尝试 evdev。
            if self.try_setup_evdev():
                self._log_info(
                    "evdev 模式已启用（%d 个键盘设备，真实按键事件，零抖动）",
                    len(self._evdev_fds),
                )
            else:
                self._log_info(
                    "evdev 不可用（权限不足或无键盘设备），回退到 stdin 双相 timeout"
                )
                self._log_info(
                    "  首键 timeout=%.2fs, 持续 timeout=%.2fs",
                    self.initial_timeout,
                    self.sustained_timeout,
                )

            while not self.stop_event.is_set():
                # START 状态会用 input() 读取菜单，按键线程不能抢 stdin。
                # evdev 模式不受影响（不读 stdin），但仍然不处理按键事件。
                if self.get_current_state() == "START":
                    self.key_was_held = False
                    time.sleep(poll_delay)
                    continue

                if self.use_evdev:
                    # ===== evdev 模式：真实按键事件 =====
                    event = self.read_evdev()
                    if event == "press" and not self.key_was_held:
                        self.request_unlock()
                        self.key_was_held = True
                    elif event == "release" and self.key_was_held:
                        self.request_lock()
                        self.key_was_held = False
                else:
                    # ===== stdin 模式：双相 timeout =====
                    if not keyboard_ready:
                        self.setup_keyboard()
                        keyboard_ready = True

                    self.check_keyboard()
                    currently_held = self.is_key_held()

                    if currently_held and not self.key_was_held:
                        self.request_unlock()
                    elif not currently_held and self.key_was_held:
                        self.request_lock()
                        # 下降沿冷却：启动 debounce 期，静默 drain
                        # auto-repeat 残留字符，防止松开瞬间反复 lock/unlock。
                        self._release_cooldown_until = (
                            time.time() + RELEASE_DEBOUNCE
                        )

                    self.key_was_held = currently_held

                time.sleep(poll_delay)
        finally:
            self.close_evdev()
            self.restore_keyboard()
            self._log_info("按键线程已退出")

    # ===== 日志工具 =====

    def _log_info(self, msg, *args):
        """安全输出 info 日志，兼容无 rospy 场景。"""
        if self.rospy is not None:
            self.rospy.loginfo("[KeyHold] " + msg, *args)
        else:
            print("[KeyHold] " + (msg % args))

    def _log_warn(self, msg, *args):
        """安全输出 warn 日志，兼容无 rospy 场景。"""
        if self.rospy is not None:
            self.rospy.logwarn("[KeyHold] " + msg, *args)
        else:
            print("[KeyHold][WARN] " + (msg % args))
