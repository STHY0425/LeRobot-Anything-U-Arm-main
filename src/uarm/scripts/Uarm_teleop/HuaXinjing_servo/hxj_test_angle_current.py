#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import serial
import fashionstar_uart_sdk as uservo

def test_angle_mode_current():
    uart = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
    control = uservo.UartServoManager(uart)
    
    # 切换到角度模式并移动到当前位置
    curr_angle = control.query_servo_angle(0)
    print(f"当前角度: {curr_angle}")
    print("切换到角度模式...")
    control.set_servo_angle(0, 0)
    time.sleep(1)
    
    print("在角度模式下读取电流（请尝试手动推动舵机）...")
    for i in range(20):
        angle = control.query_servo_angle(0)
        curr = control.query_current(0)
        print(f"角度: {angle:7.2f} | 电流: {curr:.3f}A")
        time.sleep(0.5)

if __name__ == "__main__":
    test_angle_mode_current()
