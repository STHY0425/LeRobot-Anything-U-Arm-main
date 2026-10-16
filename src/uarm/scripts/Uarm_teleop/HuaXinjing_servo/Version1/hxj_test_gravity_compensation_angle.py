#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自适应重力补偿控制脚本 (根据角度变化动态调整阻尼功率)

工作原理:
1. 舵机进入阻尼模式，读取实时角度
2. 对比上一时刻角度变化量
3. 角度正向变化越大，阻尼功率越大
4. 使舵机能被轻松推动，同时不会因重力下坠

使用示例:
rosrun fashionstar_servo_py adaptive_gravity_compensation_angle.py
rosrun fashionstar_servo_py adaptive_gravity_compensation_angle.py --id 1 --interval 0.2 --angle-gain 20
rosrun fashionstar_servo_py adaptive_gravity_compensation_angle.py --angle-gain 30 --base-power 600
"""
import time
import argparse
import serial
import fashionstar_uart_sdk as uservo

# ==================== 硬件参数配置 ====================
SERVO_PORT_NAME = '/dev/ttyUSB0'  # USB 转 TTL 端口号
SERVO_BAUDRATE = 115200           # 舵机波特率
SERVO_ID = 0                      # 舵机 ID
BASE_DAMPING_POWER = 600          # 基础阻尼功率(mW) - 空载时的阻尼
ANGLE_GAIN = 20                   # 角度增益 (mW/deg) - 每度增加多少 mW 阻尼
MAX_DAMPING_POWER = 3000          # 最大阻尼功率(mW) - 安全限制
READ_INTERVAL = 0.2               # 读取间隔(秒) - 越短补偿越快
# ===================================================


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description='自适应重力补偿控制: 根据角度变化动态调整阻尼功率',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        """
    )
    parser.add_argument('--port', '-p', default=SERVO_PORT_NAME, help='串口端口 (默认: {})'.format(SERVO_PORT_NAME))
    parser.add_argument('--baud', '-b', type=int, default=SERVO_BAUDRATE, help='波特率 (默认: {})'.format(SERVO_BAUDRATE))
    parser.add_argument('--id', '-i', type=int, default=SERVO_ID, help='舵机 ID (默认: {})'.format(SERVO_ID))
    parser.add_argument('--base-power', type=int, default=BASE_DAMPING_POWER, help='基础阻尼功率(mW) (默认: {})'.format(BASE_DAMPING_POWER))
    parser.add_argument('--angle-gain', type=float, default=ANGLE_GAIN, help='角度增益 mW/deg (默认: {:.1f})'.format(ANGLE_GAIN))
    parser.add_argument('--max-power', type=int, default=MAX_DAMPING_POWER, help='最大阻尼功率(mW) (默认: {})'.format(MAX_DAMPING_POWER))
    parser.add_argument('--interval', type=float, default=READ_INTERVAL, help='读取/补偿间隔(秒) (默认: {})'.format(READ_INTERVAL))
    parser.add_argument('--once', action='store_true', help='只运行一次 (默认为持续补偿模式)')
    return parser.parse_args()


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


def read_servo_angle(control, servo_id):
    """读取舵机角度 (单位: 度)"""
    return control.query_servo_angle(servo_id)


def calculate_compensation_power(delta_angle, base_power, angle_gain, max_power):
    """根据角度变化计算补偿阻尼功率 (单位: mW)

    公式：补偿功率 = 基础功率 + 角度变化 × 角度增益
    角度变化为负时，仅保持基础阻尼功率
    """
    if delta_angle <= 0:
        return base_power
    power = int(base_power + delta_angle * angle_gain)
    return max(0, min(power, max_power))


def main():
    """主程序：进入阻尼模式，实时进行角度自适应补偿"""
    args = parse_args()

    try:
        print('正在打开串口 {} ...'.format(args.port))
        control = open_manager(args.port, args.baud)
        print('串口打开成功！')
    except Exception as e:
        print('串口打开失败！请确认: 1.串口号是否正确 2.是否给了权限 (sudo chmod 777 /dev/ttyUSB0)')
        print('错误信息: {}'.format(e))
        return

    # 初始化阻尼模式
    try:
        print('\n初始化阻尼模式...')
        control.set_damping(args.id, args.base_power)
        print('已切换到阻尼模式: ID={}, 初始功率={}mW'.format(args.id, args.base_power))
        time.sleep(0.5)
    except Exception as e:
        print('切换阻尼模式失败！请检查舵机ID、供电和通信。')
        print('错误信息: {}'.format(e))
        return

    # 记录状态
    current_damping_power = args.base_power
    last_angle = read_servo_angle(control, args.id)

    print('\n' + '=' * 80)
    print('重力补偿参数:')
    print('  角度增益: {:.1f} mW/deg'.format(args.angle_gain))
    print('  最大功率: {} mW'.format(args.max_power))
    print('  读取间隔: {:.2f} s'.format(args.interval))
    print('=' * 80)
    print('按 Ctrl+C 停止补偿\n')

    def compensate_once():
        """执行一次读取和补偿"""
        nonlocal last_angle, current_damping_power
        try:
            angle = read_servo_angle(control, args.id)
        except Exception as e:
            print('读取失败！请检查: 1.舵机是否通电 2.通信线接对没 3.ID是否正确')
            print('错误信息: {}'.format(e))
            return False

        if angle is None:
            print('读取到无效数据 (angle={})'.format(angle))
            return False

        delta_angle = angle - last_angle
        
        # 补偿逻辑：
        # 1. 角度正向变化（往下落）时，计算阻尼并维持在该高位
        # 2. 角度大幅度负向变化（往上抬 >= 3度）时，复位到基础功率
        # 3. 其他微调情况，维持当前功率不变
        if delta_angle > 0:
            target_power = int(args.base_power + delta_angle * args.angle_gain*2)
            target_power = min(target_power, args.max_power)
            # 只有计算出的新功率更大时才更新（锁定高位）
            if target_power > current_damping_power:
                current_damping_power = target_power
        elif delta_angle <= -2.0:
            # 手部向上抬升超过 3 度，重置阻尼
            current_damping_power = args.base_power
        
        comp_power = current_damping_power

        try:
            control.set_damping(args.id, comp_power)
        except Exception as e:
            print('更新阻尼功率失败: {}'.format(e))
            return False

        print('舵机[ID:{}] 角度: {:7.2f}° | Δ角度: {:7.2f}° | 补偿功率: {} mW'.format(
            args.id, angle, delta_angle, comp_power
        ))

        last_angle = angle
        return True

    if not args.once:
        print('进入持续补偿模式...')
        try:
            while True:
                if not compensate_once():
                    break
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print('\n\n已停止补偿，恢复基础阻尼功率...')
            try:
                control.set_damping(args.id, args.base_power)
                print('恢复完成')
            except Exception:
                pass
    else:
        print('单次补偿模式：')
        compensate_once()


if __name__ == '__main__':
    main()
