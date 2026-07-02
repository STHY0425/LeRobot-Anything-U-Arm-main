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

import select
import sys
import termios
import time
import tty


# 按键松开判定的默认时间阈值（秒）。
# 如果超过这个时间没有收到 'k' 键，则认为用户已松开按键。
# 配合 xset r rate 250 40，首键延迟为 250ms，之后每 25ms 重复一次。
# 阈值必须大于首键延迟，否则第一个字符到重复之间的间隙会被误判为松开。
# 终端按键重复在 ROS/串口负载下可能出现 0.3s 以上抖动，0.5s 折中响应和稳定性。
DEFAULT_HOLD_TIMEOUT = 0.5

# 默认监听的按键字符。
DEFAULT_HOLD_KEY = "k"

# 键盘监听轮询频率（Hz）。
# 不需要太高，50Hz 足够检测按键边沿。
DEFAULT_POLL_RATE = 50


class KeyHoldThread:
    """按键锁定/解锁线程。

    监听终端键盘输入，检测 'k' 键的按住/松开边沿：
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
            hold_timeout: 按键松开判定阈值（秒），默认 0.5s。
            hold_key:     监听的按键字符，默认 'k'。
            poll_rate:    键盘轮询频率（Hz），默认 50。
            rospy:        rospy 模块引用，用于日志输出（可选）。
        """
        self.controller = controller
        self.shared_state = shared_state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.hold_timeout = float(hold_timeout)
        self.hold_key = hold_key
        self.poll_rate = int(poll_rate)
        self.rospy = rospy

        # 键盘状态。
        self._orig_termios = None
        self.last_key_time = 0.0
        self.key_was_held = False

    # ===== 键盘设置/恢复 =====

    def setup_keyboard(self):
        """将终端设置为 cbreak 模式，实现单字符无回车读取。

        保存原始终端属性到 self._orig_termios，
        以便在退出时恢复。
        """
        try:
            if sys.stdin.isatty():
                self._orig_termios = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self._log_info("键盘 cbreak 模式已设置（按住 '%s' 解锁，松开锁定）", self.hold_key)
        except Exception as exc:
            self._log_warn("键盘设置失败（可能在非交互环境）: %s", exc)

    def restore_keyboard(self):
        """恢复终端到原始模式。"""
        if self._orig_termios is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._orig_termios)
                self._orig_termios = None
            except Exception:
                pass

    # ===== 键盘检测 =====

    def check_keyboard(self):
        """非阻塞读取键盘缓冲区，更新按键时间戳。

        清空整个 stdin 缓冲区，如果其中有 hold_key，
        则更新 last_key_time 为当前时间。
        """
        try:
            if not sys.stdin.isatty():
                return
            has_key = False
            while True:
                dr, _, _ = select.select([sys.stdin], [], [], 0)
                if dr:
                    ch = sys.stdin.read(1)
                    if ch == self.hold_key:
                        has_key = True
                else:
                    break
            if has_key:
                self.last_key_time = time.time()
        except Exception:
            pass

    def is_key_held(self):
        """判断当前是否处于"按键按住"状态。

        如果距离上次收到 hold_key 的时间小于 hold_timeout，
        则认为按键仍然被按住。
        """
        return (time.time() - self.last_key_time) < self.hold_timeout

    # ===== 状态切换接口 =====

    def request_lock(self):
        """请求锁定：设置 Controller 的 lock_requested 标志。

        Controller 状态机在下一个周期检测到此标志后，
        会自动把 HOLD → LOCKED。
        """
        self.controller.lock_requested = True
        self._log_info(">>> 松开按键：请求锁定（HOLD → LOCKED）")

    def request_unlock(self):
        """请求解锁：设置 Controller 的 unlock_requested 标志。

        Controller 状态机在下一个周期检测到此标志后，
        会自动把 LOCKED → HOLD。
        """
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

        每周期：
        1. 非阻塞检查键盘输入。
        2. 判断按键是否被按住。
        3. 边沿检测：按住→松开 触发 lock，松开→按住 触发 unlock。
        4. 按频率休眠。

        边沿触发确保每次按住/松开只请求一次，不会重复下发。
        """
        poll_delay = 1.0 / float(self.poll_rate)
        keyboard_ready = False

        try:
            self._log_info("按键线程已启动（poll_rate=%dHz, timeout=%.2fs）", self.poll_rate, self.hold_timeout)

            while not self.stop_event.is_set():
                # START 状态会用 input() 读取菜单，按键线程不能抢 stdin。
                if self.get_current_state() == "START":
                    self.key_was_held = False
                    time.sleep(poll_delay)
                    continue

                if not keyboard_ready:
                    self.setup_keyboard()
                    keyboard_ready = True

                # 1. 读取键盘缓冲区。
                self.check_keyboard()

                # 2. 判断当前是否按住。
                currently_held = self.is_key_held()

                # 3. 边沿检测：只在状态变化的那一刻触发。
                if currently_held and not self.key_was_held:
                    # 上升沿：刚按下 → 请求解锁。
                    self.request_unlock()
                elif not currently_held and self.key_was_held:
                    # 下降沿：刚松开 → 请求锁定。
                    self.request_lock()

                # 4. 更新状态记录。
                self.key_was_held = currently_held

                # 5. 休眠。
                time.sleep(poll_delay)
        finally:
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
