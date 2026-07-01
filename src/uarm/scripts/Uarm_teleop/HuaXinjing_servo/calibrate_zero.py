#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""华馨京舵机零点校准工具

适用型号：带 -M 后缀（绝对值编码器）的华馨京总线舵机。
功能：
  1. 查看所有舵机当前角度
  2. 逐个释放扭矩 → 手动转到目标位置 → 设为零点
  3. 批量设零点（所有舵机当前位置归零）
  4. 设零后自动验证

使用前确认：
  - 舵机已上电、串口已连接 (/dev/ttyUSB0)
  - 舵机 ID 已分配 (0~6)
  - 舵机型号带 -M 后缀（绝对值编码器才能设零点）
"""
import sys
import time
import serial
import fashionstar_uart_sdk as uservo

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
SERVO_COUNT = 7  # 舵机数量


def open_manager():
    """打开串口并初始化舵机管理器"""
    uart = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        parity=serial.PARITY_NONE,
        stopbits=1,
        bytesize=8,
        timeout=0,
    )
    return uart, uservo.UartServoManager(uart)


def query_all_angles(manager, servo_ids):
    """查询所有舵机当前角度，返回 {id: angle} 字典"""
    angles = {}
    for sid in servo_ids:
        try:
            angle = manager.query_servo_angle(sid)
            if angle is not None:
                angles[sid] = float(angle)
        except Exception:
            angles[sid] = None
    return angles


def show_all_angles(manager, servo_ids):
    """打印所有舵机当前角度"""
    angles = query_all_angles(manager, servo_ids)
    print("\n" + "-" * 50)
    print("当前舵机角度：")
    print("-" * 50)
    for sid in servo_ids:
        if angles.get(sid) is not None:
            print("  舵机 %d: %8.2f°" % (sid, angles[sid]))
        else:
            print("  舵机 %d: ❌ 无响应" % sid)
    print("-" * 50)
    return angles


def calibrate_single(manager, servo_id):
    """校准单个舵机：释放扭矩 → 等用户调整 → 设零点 → 验证"""
    print("\n" + "=" * 50)
    print("校准舵机 %d" % servo_id)
    print("=" * 50)

    # 显示当前角度
    try:
        angle = manager.query_servo_angle(servo_id)
        print("当前角度: %.2f°" % angle if angle is not None else "无法读取角度")
    except Exception:
        print("无法读取角度")

    # Step 1: 释放扭矩
    print("\n步骤 1: 释放扭矩...")
    try:
        manager.stop_on_control_mode(servo_id, method=0x10, power=0)
        print("  舵机 %d 扭矩已释放 ✅" % servo_id)
    except Exception as e:
        print("  释放失败: %s" % e)
        return False

    # Step 2: 等用户手动调整
    print("\n步骤 2: 请手动将舵机 %d 转到目标零点位置" % servo_id)
    input("  调整好后按 Enter 继续...")

    # Step 3: 失锁状态下设零点
    print("\n步骤 3: 设置零点...")
    try:
        manager.disable_torque(servo_id)
        time.sleep(0.1)
        manager.set_origin_point(servo_id)
        print("  零点设置指令已发送 ✅")
    except Exception as e:
        print("  设置零点失败: %s" % e)
        return False

    # Step 4: 等待并验证
    time.sleep(0.5)
    print("\n步骤 4: 验证零点...")
    try:
        new_angle = manager.query_servo_angle(servo_id)
        if new_angle is not None:
            print("  设零后角度: %.2f°" % new_angle)
            if abs(new_angle) < 1.0:
                print("  ✅ 校准成功！")
                return True
            else:
                print("  ⚠️  角度偏离零点 %.2f°，可能需要微调" % new_angle)
                return True
        else:
            print("  ⚠️  无法读取角度，但零点指令已发送")
            return True
    except Exception as e:
        print("  ⚠️  验证失败: %s（零点指令可能已生效）" % e)
        return True


def calibrate_all(manager, servo_ids):
    """逐个校准所有舵机"""
    print("\n" + "=" * 50)
    print("逐个校准所有舵机（共 %d 个）" % len(servo_ids))
    print("=" * 50)
    print("流程：对每个舵机释放扭矩 → 手动调整 → 设零点")

    for i, sid in enumerate(servo_ids):
        print("\n>>> 舵机 %d (%d/%d)" % (sid, i + 1, len(servo_ids)))
        calibrate_single(manager, sid)
        if i < len(servo_ids) - 1:
            input("\n按 Enter 继续下一个舵机...")

    print("\n" + "=" * 50)
    print("🎉 所有舵机零点校准完成！")
    print("=" * 50)
    show_all_angles(manager, servo_ids)


def batch_set_zero(manager, servo_ids):
    """批量设零点：所有舵机当前位置直接设为零点"""
    print("\n" + "=" * 50)
    print("批量设零点（所有舵机当前位置 → 零点）")
    print("=" * 50)
    print("⚠️  这会把所有舵机当前角度位置设为零点！")
    print("⚠️  请确认所有舵机已在目标零点位置！")

    show_all_angles(manager, servo_ids)

    confirm = input("\n确认设零？输入 yes 继续: ").strip().lower()
    if confirm != "yes":
        print("已取消")
        return

    for sid in servo_ids:
        try:
            manager.disable_torque(sid)
            time.sleep(0.05)
            manager.set_origin_point(sid)
            print("  舵机 %d: 零点已设置 ✅" % sid)
            time.sleep(0.1)
        except Exception as e:
            print("  舵机 %d: ❌ 失败: %s" % (sid, e))

    time.sleep(0.5)
    print("\n设零后角度：")
    show_all_angles(manager, servo_ids)


def main():
    print("\n" + "=" * 50)
    print("  华馨京舵机零点校准工具 (-M 绝对值编码器)")
    print("=" * 50)

    # 检查串口
    try:
        test_ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        test_ser.close()
        print("✅ 串口 %s 可用" % SERIAL_PORT)
    except Exception as e:
        print("❌ 无法打开串口 %s: %s" % (SERIAL_PORT, e))
        sys.exit(1)

    uart, manager = open_manager()
    servo_ids = list(range(SERVO_COUNT))

    while True:
        print("\n" + "-" * 50)
        print("请选择操作：")
        print("  1. 查看所有舵机当前角度")
        print("  2. 逐个校准零点（释放→手动调整→设零）")
        print("  3. 批量设零点（所有舵机当前位置→零点）")
        print("  4. 退出")
        print("-" * 50)

        choice = input("请输入选项 (1-4): ").strip()

        if choice == "1":
            show_all_angles(manager, servo_ids)
        elif choice == "2":
            calibrate_all(manager, servo_ids)
        elif choice == "3":
            batch_set_zero(manager, servo_ids)
        elif choice == "4":
            print("再见！")
            break
        else:
            print("无效选项")

    uart.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print("\n❌ 发生错误: %s" % e)
        import traceback
        traceback.print_exc()
