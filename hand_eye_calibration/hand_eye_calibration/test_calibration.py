import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import yaml
import os
import serial
import time

class TestCalibrationNode(Node):
    def __init__(self):
        super().__init__('test_calibration')
        
        self.T = None
        self.serial_controller = None
        self.apple_position = None
        
        self.load_calibration_result()
        
        if self.T is None:
            self.get_logger().error("无法加载标定结果，请先运行标定程序！")
            return
        
        self.apple_subscriber_ = self.create_subscription(
            PointStamped,
            '/apple_location',
            self.apple_location_callback,
            10
        )
        
        self.get_logger().info("Test calibration node initialized")
        self.get_logger().info("等待检测苹果位置...")
    
    def load_calibration_result(self):
        result_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'result', 'calibration_result.yaml')
        
        if not os.path.exists(result_file):
            self.get_logger().error(f"标定结果文件不存在: {result_file}")
            return
        
        try:
            with open(result_file, 'r') as f:
                data = yaml.safe_load(f)
            
            transform_matrix = np.array(data['calibration']['transformation_matrix']).reshape(4, 4)
            self.T = transform_matrix
            
            self.get_logger().info(f"成功加载标定结果")
            self.get_logger().info(f"变换矩阵 T:\n{self.T}")
        except Exception as e:
            self.get_logger().error(f"加载标定结果失败: {e}")
    
    def apple_location_callback(self, msg):
        if self.T is None:
            self.get_logger().error("标定矩阵未加载！")
            return
        
        camera_x = msg.point.x
        camera_y = msg.point.y
        camera_z = msg.point.z
        
        self.get_logger().info(f"检测到苹果位置（相机坐标）: X={camera_x:.4f}, Y={camera_y:.4f}, Z={camera_z:.4f}")
        
        arm_point = self.camera_to_arm_coordinate((camera_x, camera_y, camera_z))
        
        if arm_point is not None:
            self.apple_position = arm_point
            self.get_logger().info(f"转换后的机械臂坐标: X={arm_point[0]:.4f}, Y={arm_point[1]:.4f}, Z={arm_point[2]:.4f}")
            
            self.move_to_apple()
    
    def camera_to_arm_coordinate(self, camera_point):
        if self.T is None:
            return None
        
        xc, yc, zc = camera_point
        xcc = -zc
        ycc = xc
        zcc = -yc
        transformed_point = (xcc, ycc, zcc)
        
        camera_homogeneous = np.array([transformed_point[0], transformed_point[1], transformed_point[2], 1.0])
        arm_homogeneous = self.T @ camera_homogeneous
        
        return tuple(arm_homogeneous[:3])
    
    def move_to_apple(self):
        if self.apple_position is None:
            return
        
        x, y, z = self.apple_position
        
        self.get_logger().info(f"移动机械臂到苹果位置: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        if not self.init_serial("/dev/ttyUSB0", 115200):
            self.get_logger().error("无法初始化串口")
            return
        
        success = self.send_move_command(x, y, z)
        
        if success:
            self.get_logger().info("机械臂移动命令已发送")
        else:
            self.get_logger().error("机械臂移动命令发送失败")
    
    def init_serial(self, port, baud_rate):
        try:
            self.serial_controller = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f"串口初始化成功: {port} @ {baud_rate}")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"串口初始化失败: {e}")
            return False
    
    def send_move_command(self, x, y, z, time=2000):
        if self.serial_controller is None:
            self.get_logger().error("串口未初始化！")
            return False
        
        command = self.kinematics_move(x, y, z, time)
        
        if command is None:
            self.get_logger().error("无法找到有效的逆运动学解！")
            return False
        
        try:
            self.serial_controller.write(command.encode())
            self.get_logger().info(f"发送命令: {command}")
            return True
        except Exception as e:
            self.get_logger().error(f"发送命令失败: {e}")
            return False
    
    def kinematics_move(self, x, y, z, time):
        l1 = 80.0
        l2 = 80.0
        l3 = 105.0
        
        r = np.sqrt(x**2 + y**2)
        z_offset = z - 67.5
        
        d = np.sqrt(r**2 + z_offset**2)
        
        if d > l1 + l2 + l3 or d < abs(l1 - l2 - l3):
            self.get_logger().error(f"目标位置不可达: ({x}, {y}, {z})")
            return None
        
        theta1 = np.arctan2(y, x) * 180 / np.pi
        
        if theta1 < 0:
            theta1 += 360
        
        if abs(theta1 - 180) < 5:
            theta1 = 175
        
        a = (l2**2 + d**2 - l3**2) / (2 * l2 * d)
        if a > 1:
            a = 1
        elif a < -1:
            a = -1
        
        phi = np.arccos(a) * 180 / np.pi
        
        psi = np.arctan2(z_offset, r) * 180 / np.pi
        
        theta2 = 90 - psi + phi
        
        theta3 = 180 - phi
        
        theta4 = 90 - (theta2 + theta3 - 90)
        
        theta1_pwm = int(1500 + theta1 * 10)
        theta2_pwm = int(1500 - theta2 * 10)
        theta3_pwm = int(1500 + theta3 * 10)
        theta4_pwm = int(1500 + theta4 * 10)
        
        if theta1_pwm < 500 or theta1_pwm > 2500:
            self.get_logger().error(f"theta1 PWM 值超出范围: {theta1_pwm}")
            return None
        if theta2_pwm < 500 or theta2_pwm > 2500:
            self.get_logger().error(f"theta2 PWM 值超出范围: {theta2_pwm}")
            return None
        if theta3_pwm < 500 or theta3_pwm > 2500:
            self.get_logger().error(f"theta3 PWM 值超出范围: {theta3_pwm}")
            return None
        if theta4_pwm < 500 or theta4_pwm > 2500:
            self.get_logger().error(f"theta4 PWM 值超出范围: {theta4_pwm}")
            return None
        
        cmd = f"#0P{theta1_pwm:04d}T{time:04d}!#1P{theta2_pwm:04d}T{time:04d}!#2P{theta3_pwm:04d}T{time:04d}!#3P{theta4_pwm:04d}T{time:04d}!"
        
        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = TestCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("测试节点已停止")
    finally:
        if node.serial_controller is not None:
            node.serial_controller.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()