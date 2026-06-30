#!/usr/bin/env python3
"""
中菱舵机完整配置脚本
功能: ID 配置、测试、验证
"""
import serial
import time
import re
import sys

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

def send_command(ser, cmd):
    """发送命令并接收响应"""
    ser.write(cmd.encode('ascii'))
    time.sleep(0.1)
    return ser.read_all().decode('ascii', errors='ignore')

def set_servo_id(ser, new_id):
    """配置舵机 ID"""
    cmd = f'#000PID{new_id:03d}!'
    response = send_command(ser, cmd)
    return True  # 中菱舵机可能不返回响应

def test_servo(ser, servo_id):
    """测试舵机响应"""
    cmd = f'#{servo_id:03d}PRAD!'
    response = send_command(ser, cmd)
    
    match = re.search(r'P(\d{4})', response)
    if match:
        pwm = int(match.group(1))
        angle = (pwm - 500) / 2000 * 270
        return True, angle
    return False, None

def release_torque(ser, servo_id):
    """释放舵机扭矩"""
    cmd = f'#{servo_id:03d}PULK!'
    send_command(ser, cmd)

def lock_torque(ser, servo_id):
    """锁定舵机扭矩"""
    cmd = f'#{servo_id:03d}PLOK!'
    send_command(ser, cmd)

def move_servo_to_position(ser, servo_id, pwm_value, move_time=1000):
    """移动舵机到指定位置
    
    Args:
        ser: 串口对象
        servo_id: 舵机ID
        pwm_value: 目标PWM值 (500-2500)
        move_time: 移动时间(毫秒)
    """
    cmd = f'#{servo_id:03d}P{pwm_value:04d}T{move_time:04d}!'
    send_command(ser, cmd)

def reset_all_servos_to_center():
    """将所有舵机复位到中间位置（135°）"""
    print("\n" + "=" * 70)
    print("【舵机区域复位】")
    print("=" * 70)
    print("\n功能说明:")
    print("  - 将所有舵机移动到中间位置（135°，PWM=1500）")
    print("  - 这是最安全的位置，有最大的活动空间")
    print("  - 建议在安装舵机到机械臂前执行此操作")
    print("\n角度范围:")
    print("  - 舵机物理范围: 0° - 270°")
    print("  - 中间位置: 135° (PWM=1500)")
    print("  - 从中间位置可以向两边各转动 ±135°")
    
    print("\n⚠️  注意:")
    print("  1. 确保所有舵机都已连接")
    print("  2. 舵机会缓慢移动到中间位置（2秒）")
    print("  3. 移动过程中请勿触碰舵机")
    
    input("\n按 Enter 开始复位...")
    
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
        print("\n正在复位...")
        print("-" * 70)
        
        # PWM=1500 对应 135°（中间位置）
        center_pwm = 1500
        move_time = 2000  # 2秒缓慢移动
        
        for i in range(7):
            print(f"  舵机 {i}: 移动到中间位置 (135°)...", end='', flush=True)
            move_servo_to_position(ser, i, center_pwm, move_time)
            time.sleep(0.1)
            print(" ✅")
        
        print("-" * 70)
        print("\n等待舵机移动完成...")
        time.sleep(2.5)  # 等待移动完成
        
        # 验证位置
        print("\n验证舵机位置:")
        print("-" * 70)
        all_ok = True
        for i in range(7):
            success, angle = test_servo(ser, i)
            if success:
                # 允许 ±5° 的误差
                if abs(angle - 135) < 5:
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

def main():
    print("\n" + "=" * 70)
    print(" " * 20 + "中菱舵机配置向导")
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
        print("  3. 是否有权限访问串口 (sudo chmod 666 /dev/ttyUSB0)")
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
    print("\n为什么要这样做？")
    print("  - 出厂舵机 ID 都是 000")
    print("  - 如果同时连接多个，它们会同时响应命令")
    print("  - 必须逐个配置不同的 ID")
    
    input("\n按 Enter 开始配置...")
    
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
        for i in range(7):
            print(f"\n{'=' * 70}")
            print(f"配置舵机 {i+1}/7 (ID = {i})")
            print("=" * 70)
            
            input(f"\n1. 请只连接第 {i+1} 个舵机")
            input("2. 确认连接后按 Enter 开始配置...")
            
            print(f"\n正在配置 ID {i}...")
            if set_servo_id(ser, i):
                print(f"✅ 舵机 ID {i} 配置完成")
                
                # 验证
                time.sleep(0.5)
                success, angle = test_servo(ser, i)
                if success:
                    print(f"✅ 验证成功: 当前角度 = {angle:.1f}°")
                else:
                    print(f"⚠️  无法验证，但配置命令已发送")
            else:
                print(f"⚠️  配置命令已发送（舵机可能不返回响应）")
            
            if i < 6:
                input(f"\n3. 请断开第 {i+1} 个舵机，按 Enter 继续...")
    
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
    
    input("\n请确保所有 7 个舵机都已连接，按 Enter 开始测试...")
    
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
        print("\n测试结果:")
        print("-" * 70)
        
        success_count = 0
        angles = []
        
        for i in range(7):
            success, angle = test_servo(ser, i)
            if success:
                print(f"  舵机 {i}: ✅ 响应正常 | 角度 = {angle:6.1f}°")
                success_count += 1
                angles.append(angle)
            else:
                print(f"  舵机 {i}: ❌ 无响应")
                angles.append(None)
        
        print("-" * 70)
        print(f"\n结果: {success_count}/7 舵机响应正常")
        
        if success_count == 7:
            print("\n🎉 所有舵机工作正常！")
            print("\n当前角度:")
            print(f"  {[f'{a:.1f}°' if a else 'N/A' for a in angles]}")
            print("\n可以开始使用了:")
            print("  rosrun uarm servo_reader.py")
        elif success_count == 0:
            print("\n❌ 所有舵机都无响应，请检查:")
            print("  1. 舵机是否正确连接")
            print("  2. 电源是否接通")
            print("  3. 串口设备是否正确")
        else:
            print(f"\n⚠️  只有 {success_count} 个舵机响应，请检查:")
            print("  1. 未响应的舵机是否正确连接")
            print("  2. ID 是否配置正确")
            print("  3. 电源是否充足")

def release_all_torque():
    """释放所有舵机扭矩"""
    print("\n" + "=" * 70)
    print("【释放舵机扭矩】")
    print("=" * 70)
    print("\n释放扭矩后，舵机可以自由转动，便于手动调整位置")
    
    input("按 Enter 继续...")
    
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
        for i in range(7):
            release_torque(ser, i)
            print(f"  舵机 {i}: 扭矩已释放")
    
    print("\n✅ 所有舵机扭矩已释放")
    print("   现在可以手动调整舵机位置")

def lock_all_torque():
    """锁定所有舵机扭矩"""
    print("\n" + "=" * 70)
    print("【锁定舵机扭矩】")
    print("=" * 70)
    print("\n锁定扭矩后，舵机会保持当前位置")
    
    input("按 Enter 继续...")
    
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5) as ser:
        for i in range(7):
            lock_torque(ser, i)
            print(f"  舵机 {i}: 扭矩已锁定")
    
    print("\n✅ 所有舵机扭矩已锁定")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
