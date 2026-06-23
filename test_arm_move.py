#!/usr/bin/env python3
"""
机械臂移动控制脚本
可以直接运行，通过串口控制机械臂移动到指定位置

使用方法:
    python3 arm_move.py x y z
    例如: python3 arm_move.py 0 280 180
    
参数:
    x, y, z: 机械臂目标位置（单位：毫米）
"""

import serial
import time
import math
import re
import sys


class Kinematics:
    """运动学求解类"""
    
    def __init__(self):
        self.L0 = 0.0
        self.L1 = 0.0
        self.L2 = 0.0
        self.L3 = 0.0
        self.servo_angle = [0.0, 0.0, 0.0, 0.0]
        self.servo_pwm = [0, 0, 0, 0]
    
    def setup_kinematics(self, L0=100, L1=105, L2=88, L3=155):
        """设置机械臂连杆长度（单位：毫米）"""
        self.L0 = L0 * 10
        self.L1 = L1 * 10
        self.L2 = L2 * 10
        self.L3 = L3 * 10
    
    def kinematics_analysis(self, x, y, z, Alpha):
        """运动学逆解分析"""
        pi = math.pi
        x_scaled = x * 10
        y_scaled = y * 10
        z_scaled = z * 10
        
        l0 = self.L0
        l1 = self.L1
        l2 = self.L2
        l3 = self.L3
        
        if x_scaled == 0:
            theta6 = 0.0
        else:
            theta6 = math.atan2(x_scaled, y_scaled) * 270.0 / pi
        
        y_scaled = math.sqrt(x_scaled * x_scaled + y_scaled * y_scaled)
        y_scaled = y_scaled - l3 * math.cos(Alpha * pi / 180.0)
        z_scaled = z_scaled - l0 - l3 * math.sin(Alpha * pi / 180.0)
        
        if z_scaled < -l0:
            return 1
        if math.sqrt(y_scaled * y_scaled + z_scaled * z_scaled) > (l1 + l2):
            return 2
        
        ccc = math.acos(y_scaled / math.sqrt(y_scaled * y_scaled + z_scaled * z_scaled))
        bbb = (y_scaled * y_scaled + z_scaled * z_scaled + l1 * l1 - l2 * l2) / (2 * l1 * math.sqrt(y_scaled * y_scaled + z_scaled * z_scaled))
        
        if bbb > 1 or bbb < -1:
            return 5
        
        zf_flag = -1 if z_scaled < 0 else 1
        theta5 = ccc * zf_flag + math.acos(bbb)
        theta5 = theta5 * 180.0 / pi
        
        if theta5 > 180.0 or theta5 < 0.0:
            return 6
        
        aaa = -(y_scaled * y_scaled + z_scaled * z_scaled - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        
        if aaa > 1 or aaa < -1:
            return 3
        
        theta4 = math.acos(aaa)
        theta4 = 180.0 - theta4 * 180.0 / pi
        
        if theta4 > 135.0 or theta4 < -135.0:
            return 4
        
        theta3 = Alpha - theta5 + theta4
        
        if theta3 > 90.0 or theta3 < -90.0:
            return 7
        
        self.servo_angle[0] = theta6
        self.servo_angle[1] = theta5 - 90
        self.servo_angle[2] = theta4
        self.servo_angle[3] = theta3
        
        self.servo_pwm[0] = int(1500 - 2000.0 * self.servo_angle[0] / 270.0)
        self.servo_pwm[1] = int(1500 + 2000.0 * self.servo_angle[1] / 270.0)
        self.servo_pwm[2] = int(1500 + 2000.0 * self.servo_angle[2] / 270.0)
        self.servo_pwm[3] = int(1500 - 2000.0 * self.servo_angle[3] / 270.0)
        
        return 0
    
    def kinematics_move(self, x, y, z, time_ms):
        """计算移动到目标位置的串口命令
        
        Args:
            x, y, z: 目标位置（单位：毫米）
            time_ms: 运动时间（毫秒）
        
        Returns:
            串口命令字符串，如果无解返回 None
        """
        if y < 0:
            return None
        
        min_alpha = 0
        flag = 0
        
        # 搜索最佳角度
        for i in range(0, -136, -1):
            if self.kinematics_analysis(x, y, z, i) == 0:
                if i < min_alpha:
                    min_alpha = i
                flag = 1
        
        if flag:
            self.kinematics_analysis(x, y, z, min_alpha)
            cmd_return = "{{#000P{:04d}T{:04d}!#001P{:04d}T{:04d}!#002P{:04d}T{:04d}!#003P{:04d}T{:04d}!}}".format(
                self.servo_pwm[0], time_ms,
                self.servo_pwm[1], time_ms,
                self.servo_pwm[2], time_ms,
                3000 - self.servo_pwm[3], time_ms
            )
            return cmd_return
        
        return None


class SerialController:
    """串口通信类"""
    
    def __init__(self, port_name="/dev/ttyUSB0", baudrate=115200):
        self.port_name = port_name
        self.baudrate = baudrate
        self.ser = None
    
    def open(self):
        """打开串口"""
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            
            if self.ser.is_open:
                self.ser.setRTS(False)
                time.sleep(2.0)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                return True
                
        except serial.SerialException as e:
            print(f"串口打开失败: {e}")
        except Exception as e:
            print(f"发生错误: {e}")
        
        return False
    
    def send_command(self, command):
        """发送命令"""
        if self.ser is None or not self.ser.is_open:
            print("错误：串口未打开！")
            return False
        
        try:
            self.ser.write(command.encode('utf-8'))
            self.ser.flush()
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def close(self):
        """关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")


def calculate_execution_time(command):
    """计算命令执行时间"""
    pattern = r'T(\d{4})!'
    matches = re.findall(pattern, command)
    
    if not matches:
        return 1.0
    
    total_ms = sum(int(t) for t in matches)
    num_servos = len(matches)
    total_ms += (num_servos - 1) * 20
    
    return total_ms / 1000.0


def set_servo(serial_controller, index, pwm, time_ms=1500):
    """设置单个舵机位置
    
    Args:
        serial_controller: 串口控制器实例
        index: 舵机编号 (0-5)
        pwm: PWM值 (0-3000)
        time_ms: 运动时间（毫秒）
    
    Returns:
        (执行时间, 是否成功)
    """
    if serial_controller is None or not serial_controller.ser.is_open:
        print("串口未初始化！")
        return (0.0, False)
    
    if index < 0 or index > 5:
        print(f"舵机编号必须在0-5之间，当前为 {index}")
        return (0.0, False)
    
    if pwm < 0 or pwm > 3000:
        print(f"PWM值必须在0-3000之间，当前为 {pwm}")
        return (0.0, False)
    
    if index == 3:
        pwm_value = 3000 - pwm
    else:
        pwm_value = pwm
    
    command = "#{:03d}P{:04d}T{:04d}!".format(index, pwm_value, time_ms)
    
    print(f"设置舵机 {index}: PWM={pwm_value}, 时间={time_ms}ms")
    
    success = serial_controller.send_command(command)
    
    exec_time = time_ms / 1000.0 + 0.02
    return (exec_time, success)


def move_to(serial_controller, kinematics, x, y, z, time_ms=2000):
    """移动机械臂到指定位置
    
    Args:
        serial_controller: 串口控制器实例
        kinematics: 运动学求解实例
        x, y, z: 目标位置（毫米）
        time_ms: 运动时间（毫秒）
    
    Returns:
        (执行时间, 是否成功)
    """
    if serial_controller is None or not serial_controller.ser.is_open:
        print("串口未初始化！")
        return (0.0, False)
    
    command = kinematics.kinematics_move(x, y, z, time_ms)
    
    if command is None:
        print("错误：无法找到有效的逆运动学解！")
        return (0.0, False)
    
    print(f"x={x}, y={y}, z={z},L0={kinematics.L0}, L1={kinematics.L1}, L2={kinematics.L2}, L3={kinematics.L3}发送命令: {command}")
    
    exec_time = calculate_execution_time(command)
    
    success = serial_controller.send_command(command)
    
    return (exec_time, success)


def main():
    # 解析命令行参数
    if len(sys.argv) != 4:
        print("使用方法: python3 arm_move.py x y z")
        print("例如: python3 arm_move.py 0 280 180")
        sys.exit(1)
    
    try:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        z = int(sys.argv[3])
    except ValueError:
        print("错误：位置参数必须是整数！")
        sys.exit(1)
    
    # 初始化组件
    kinematics = Kinematics()
    kinematics.setup_kinematics(L0=100, L1=105, L2=88, L3=155)
    
    serial_controller = SerialController(port_name="/dev/ttyUSB0", baudrate=115200)
    
    # 打开串口
    print(f"正在打开串口 {serial_controller.port_name}...")
    if not serial_controller.open():
        print("串口打开失败！")
        sys.exit(1)
    print("串口已打开")
    
    try:
        ################################张开夹抓###########################################
        print(f"\n张开夹抓")
        exec_time, success = set_servo(serial_controller, 5, 1600, time_ms=2000)
        
        if success:
            print(f"命令已发送，预计执行时间: {exec_time:.2f} 秒")
            print(f"等待夹抓打开...")
            time.sleep(exec_time + 1.0)
            print("夹抓打开完成！")
        else:
            print("夹抓打开失败！") 


        ######################################移动到指定位置上方#######################################
        # 移动到目标位置
        print(f"\n移动机械臂到位置: ({x}, {y}, {z})")
        exec_time, success = move_to(serial_controller, kinematics, x, y, z, time_ms=2000)
        
        if success:
            print(f"命令已发送，预计执行时间: {exec_time:.2f} 秒")
            print(f"等待机械臂到达目标位置...")
            time.sleep(exec_time / 2 + 1.0)
            print("移动完成！")
        else:
            print("移动失败！")

        ################################关闭夹抓###########################################
        print(f"\n关闭夹抓")
        exec_time, success = set_servo(serial_controller, 5, 1700, time_ms=2000)
        
        if success:
            print(f"命令已发送，预计执行时间: {exec_time:.2f} 秒")
            print(f"等待夹抓关闭...")
            time.sleep(exec_time + 1.0)
            print("夹抓关闭完成！")
        else:
            print("夹抓关闭失败！")
        
        #####################################提起########################################
        z = 180
        print(f"\n移动机械臂到位置: ({x}, {y}, {z})")
        exec_time, success = move_to(serial_controller, kinematics, x, y, z, time_ms=2000)
        
        if success:
            print(f"命令已发送，预计执行时间: {exec_time:.2f} 秒")
            print(f"等待机械臂到达目标位置...")
            time.sleep(exec_time/4 + 1.0)
            print("移动完成！")
        else:
            print("移动失败！")

        ###################################移动到指定位置##################################
        x = -150
        print(f"\n移动机械臂到位置: ({x}, {y}, {z})")
        exec_time, success = move_to(serial_controller, kinematics, x, y, z, time_ms=2000)
        
        if success:
            print(f"命令已发送，预计执行时间: {exec_time:.2f} 秒")
            print(f"等待机械臂到达目标位置...")
            time.sleep(exec_time/4 + 1.0)
            print("移动完成！")
        else:
            print("移动失败！")

        ################################松开夹抓###########################################
        print(f"\n关闭夹抓")
        exec_time, success = set_servo(serial_controller, 5, 1000, time_ms=2000)
        
        if success:
            print(f"命令已发送，预计执行时间: {exec_time:.2f} 秒")
            print(f"等待夹抓关闭...")
            time.sleep(exec_time + 1.0)
            print("夹抓关闭完成！")
        else:
            print("夹抓关闭失败！")
    
    finally:
        serial_controller.close()


if __name__ == '__main__':
    main()
