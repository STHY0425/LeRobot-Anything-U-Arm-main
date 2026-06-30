#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
华馨京舵机完整配置脚本
功能: ID 配置、测试、验证、释放/锁定扭矩、复位到中间位置
"""
import time
import serial
import sys
import argparse
import fashionstar_uart_sdk as uservo

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
SERVO_COUNT = 7  # 舵机数量


def open_manager(port, baud):
    """打开串口并初始化舵机管理器"""
    uart = serial.Serial(
        port=port,
        baudrate=baud,
        parity=serial.PARITY_NONE,
        stopbits=1,
        bytesize=8,
        timeout=0,
    )
    return uservo.UartServoManager(uart)


def test_servo(control, servo_id):
    """测试单个舵机响应"""
    try:
        angle = control.query_servo_angle(servo_id)
        if angle is not None:
            return True, angle
    except:
        pass
    return False, None


def set_servo_angle_by_id(control, servo_id, angle, velocity=50):
    """设置舵机角度"""
    try:
        control.set_servo_angle(servo_id, angle, velocity=velocity)
        return True
    except Exception as e:
        print(f"      ❌ 设置角度失败: {e}")
        return False


def detect_servo_id(ser, start_id=0, end_id=16):
    """检测当前连接的舵机 ID（只连接一个舵机时有效）"""
    try:
        mgr = uservo.UartServoManager(ser)
        for cand in range(start_id, end_id + 1):
            if mgr.ping(cand):
                return cand
    except Exception:
        pass
    return None


def set_servo_id(ser, new_id, current_id):
    """通过串口对象写入新 ID（地址 0x22）"""
    try:
        mgr = uservo.UartServoManager(ser)
        addr = 0x22
        mgr.write_data(current_id, addr, bytes([new_id]))
        return True
    except Exception:
        return False


def test_servo_query(ser, servo_id):
    """使用串口对象测试指定 ID 的舵机并返回 (success, angle)"""
    try:
        mgr = uservo.UartServoManager(ser)
        angle = mgr.query_servo_angle(servo_id)
        if angle is not None:
            return True, angle
    except Exception:
        pass
    return False, None


def configure_ids():
    """配置舵机 ID"""
    print("\n" + "=" * 70)
    print("【配置舵机 ID】")
    print("=" * 70)
    print("\n⚠️  重要说明:")
    print("  1. 每次只连接一个舵机到控制板")
    print("  2. 配置完成后断开该舵机")
    print("  3. 再连接下一个舵机")
    print("  4. 按顺序配置 ID: 0, 1, 2, 3, 4, 5, 6")
    # print("\n为什么要这样做？")
    # print("  - 出厂舵机 ID 均为 1")
    # print("  - 如果同时连接多个，它们会同时响应命令")
    # print("  - 必须逐个配置不同的 ID")
    
    input("\n按 Enter 开始配置...")
    
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
            for i in range(7):
                print(f"\n{'=' * 70}")
                print(f"配置舵机 {i+1}/7 (ID = {i})")
                print("=" * 70)
                
                input(f"\n 请只连接第 {i+1} 个舵机")
                input(" 确认连接后按 Enter 开始配置...")
                
                print("\n正在检测当前 ID...")
                current_id = detect_servo_id(ser)
                if current_id is None:
                    print("⚠️  未检测到舵机 ID，请检查连接/供电")
                    if i < 6:
                        input(f"\n 请断开第 {i+1} 个舵机，按 Enter 继续...")
                    continue

                print(f"检测到当前 ID = {current_id}，开始写入 ID {i}...")
                if set_servo_id(ser, i, current_id):
                    print(f"✅ 舵机 ID {i} 配置完成")
                    
                    # 验证
                    time.sleep(0.5)
                    success, angle = test_servo_query(ser, i)
                    if success:
                        print(f"✅ 验证成功: 当前角度 = {angle:.1f}°")
                    else:
                        print(f"⚠️  无法验证，但配置命令已发送")
                else:
                    print(f"⚠️  配置命令已发送（舵机可能不返回响应）")
                
                if i < 6:
                    input(f"\n3. 请断开第 {i+1} 个舵机，按 Enter 继续...")
    except Exception as e:
        print(f"\n❌ 配置过程中出错: {e}")
    
    print("\n" + "=" * 70)
    print("🎉 所有舵机 ID 配置完成！")
    print("=" * 70)
    print("\n下一步:")
    print("  1. 连接所有 7 个舵机")
    print("  2. 选择菜单选项 2 测试所有舵机")


def test_all_servos():
    """测试所有舵机"""
    print("\n" + "=" * 70)
    print("【测试所有舵机】")
    print("=" * 70)
    
    input("\n请确保所有舵机都已连接，按 Enter 开始测试...\n")
    
    try:
        control = open_manager(SERIAL_PORT, BAUD_RATE)
        
        print("测试结果:")
        print("-" * 70)
        
        success_count = 0
        angles = []
        
        for i in range(SERVO_COUNT):
            success, angle = test_servo(control, i)
            if success:
                print(f"  舵机 {i}: ✅ 响应正常 | 角度 = {angle:6.1f}°")
                success_count += 1
                angles.append(angle)
            else:
                print(f"  舵机 {i}: ❌ 无响应")
                angles.append(None)
        
        print("-" * 70)
        print(f"\n结果: {success_count}/{SERVO_COUNT} 舵机响应正常")
        
        if success_count == SERVO_COUNT:
            print(f"\n🎉 所有 {SERVO_COUNT} 个舵机工作正常！")
            print(f"\n当前角度: {angles}")
            print("\n可以开始使用了")
        elif success_count == 0:
            print(f"\n❌ 所有舵机都无响应，请检查:")
            print("  1. 舵机是否正确连接")
            print("  2. 电源是否接通")
            print("  3. 串口设备是否正确")
        else:
            print(f"\n⚠️  只有 {success_count} 个舵机响应，请检查:")
            print("  1. 未响应的舵机是否正确连接")
            print("  2. ID 是否配置正确 (应为 0-6)")
            print("  3. 电源是否充足")
    
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")


def release_all_torque():
    """释放所有舵机扭矩"""
    print("\n" + "=" * 70)
    print("【释放舵机扭矩】")
    print("=" * 70)
    print("\n释放扭矩后，舵机可以自由转动，便于手动调整位置")
    
    input("按 Enter 继续...")
    
    try:
        control = open_manager(SERIAL_PORT, BAUD_RATE)
        
        print("\n正在释放扭矩...")
        print("-" * 70)
        for i in range(SERVO_COUNT):
            control.stop_on_control_mode(i, method=0x10, power=0)
            print(f"  舵机 {i}: 扭矩已释放 ✅")
            time.sleep(0.1)
        
        print("-" * 70)
        print("\n✅ 所有舵机扭矩已释放")
        print("   现在可以手动调整舵机位置")
    
    except Exception as e:
        print(f"\n❌ 释放失败: {e}")


def lock_all_torque():
    """锁定所有舵机扭矩"""
    print("\n" + "=" * 70)
    print("【锁定舵机扭矩】")
    print("=" * 70)
    print("\n锁定扭矩后，舵机会保持当前位置")
    
    input("按 Enter 继续...")
    
    try:
        control = open_manager(SERIAL_PORT, BAUD_RATE)
        
        print("\n正在锁定扭矩...")
        print("-" * 70)
        for i in range(SERVO_COUNT):
            control.stop_on_control_mode(i, method=0x11, power=500)
            print(f"  舵机 {i}: 扭矩已锁定 ✅")
            time.sleep(0.1)
        
        print("-" * 70)
        print("\n✅ 所有舵机扭矩已锁定")
    
    except Exception as e:
        print(f"\n❌ 锁定失败: {e}")


def reset_all_servos_to_center():
    """将所有舵机复位到中间位置（135°）"""
    print("\n" + "=" * 70)
    print("【舵机区域复位】")
    print("=" * 70)
    print("\n功能说明:")
    print("  - 将所有舵机移动到中间位置（135°）")
    print("  - 这是最安全的位置，有最大的活动空间")
    print("  - 建议在安装舵机到机械臂前执行此操作")
    print("\n角度范围:")
    print("  - 舵机物理范围: 0° - 270°")
    print("  - 中间位置: 135°")
    print("  - 从中间位置可以向两边各转动 ±135°")
    
    print("\n⚠️  注意:")
    print("  1. 确保所有舵机都已连接")
    print("  2. 舵机会缓慢移动到中间位置（2秒）")
    print("  3. 移动过程中请勿触碰舵机")
    
    input("\n按 Enter 开始复位...")
    
    try:
        control = open_manager(SERIAL_PORT, BAUD_RATE)
        
        center_angle = 135.0
        move_velocity = 50  # °/s，缓慢移动
        
        print("\n正在复位...")
        print("-" * 70)
        
        for i in range(SERVO_COUNT):
            print(f"  舵机 {i}: 移动到中间位置 (135°)...", end='', flush=True)
            if set_servo_angle_by_id(control, i, center_angle, velocity=move_velocity):
                print(" ✅")
            else:
                print(" ❌")
            time.sleep(0.1)
        
        print("-" * 70)
        print("\n等待舵机移动完成...")
        time.sleep(2.5)
        
        # 验证位置
        print("\n验证舵机位置:")
        print("-" * 70)
        all_ok = True
        for i in range(SERVO_COUNT):
            success, angle = test_servo(control, i)
            if success:
                # 允许 ±5° 的误差
                if abs(angle - center_angle) < 5:
                    print(f"  舵机 {i}: ✅ 位置正确 ({angle:.1f}°)")
                else:
                    print(f"  舵机 {i}: ⚠️  位置偏差较大 ({angle:.1f}° vs 135°)")
                    all_ok = False
            else:
                print(f"  舵机 {i}: ❌ 无法读取位置")
                all_ok = False
        
        print("-" * 70)
        
        if all_ok:
            print("\n🎉 所有舵机已复位到中间位置！")
            print("\n下一步:")
            print("  1. 现在可以安全地将舵机安装到机械臂上")
            print("  2. 安装时保持舵机在当前位置")
            print("  3. 这样可以确保机械臂有最大的活动范围")
        else:
            print("\n⚠️  部分舵机复位可能不完全，请检查:")
            print("  1. 舵机是否正确连接")
            print("  2. 电源是否充足")
            print("  3. 舵机是否被卡住")
    
    except Exception as e:
        print(f"\n❌ 复位失败: {e}")


def main():
    print("\n" + "=" * 70)
    print(" " * 15 + "华馨京舵机完整配置工具")
    print("=" * 70)
    
    # 检查串口
    try:
        test_ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        test_ser.close()
        print(f"\n✅ 串口 {SERIAL_PORT} 可用")
    except Exception as e:
        print(f"\n❌ 无法打开串口 {SERIAL_PORT}")
        print(f"   错误: {e}")
        print("\n请检查:")
        print("  1. 舵机控制板是否连接")
        print("  2. 串口设备名称是否正确")
        print("  3. 是否有权限访问串口: sudo chmod 666 /dev/ttyUSB0")
        sys.exit(1)
    
    # 主菜单
    while True:
        print("\n" + "-" * 70)
        print("请选择操作:")
        print("  1. 配置舵机 ID (首次使用必须)")
        print("  2. 测试所有舵机")
        print("  3. 释放所有舵机扭矩 (便于手动调整)")
        print("  4. 锁定所有舵机扭矩")
        print("  5. 区域复位 - 所有舵机回到中间位置 (推荐安装前使用)")
        print("  6. 退出")
        print("-" * 70)
        
        choice = input("\n请输入选项 (1-6): ").strip()
        
        if choice == '1':
            configure_ids()
        elif choice == '2':
            test_all_servos()
        elif choice == '3':
            release_all_torque()
        elif choice == '4':
            lock_all_torque()
        elif choice == '5':
            reset_all_servos_to_center()
        elif choice == '6':
            print("\n再见！")
            break
        else:
            print("\n❌ 无效选项，请重新选择")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
