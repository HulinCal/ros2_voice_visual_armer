import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import serial
import os
import yaml
from datetime import datetime
import statistics
import math
import re

class Kinematics:
    def __init__(self):
        self.L0 = 0.0
        self.L1 = 0.0
        self.L2 = 0.0
        self.L3 = 0.0
        self.servo_angle = [0.0, 0.0, 0.0, 0.0]
        self.servo_pwm = [0, 0, 0, 0]
    
    def setup_kinematics(self, L0=100, L1=105, L2=88, L3=200):
        self.L0 = L0 * 10
        self.L1 = L1 * 10
        self.L2 = L2 * 10
        self.L3 = L3 * 10
    
    def kinematics_analysis(self, x, y, z, Alpha):
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
    
    def kinematics_move(self, x, y, z, time):
        if y < 0:
            return None
        
        min_alpha = 0
        flag = 0
        
        for i in range(0, -136, -1):
            if self.kinematics_analysis(x, y, z, i) == 0:
                if i < min_alpha:
                    min_alpha = i
                flag = 1
        
        if flag:
            self.kinematics_analysis(x, y, z, min_alpha)
            cmd_return = "{{#000P{:04d}T{:04d}!#001P{:04d}T{:04d}!#002P{:04d}T{:04d}!#003P{:04d}T{:04d}!}}".format(
                self.servo_pwm[0], time,
                self.servo_pwm[1], time,
                self.servo_pwm[2], time,
                3000 - self.servo_pwm[3], time
            )
            return cmd_return
        
        return None

def calculate_execution_time(command):
    pattern = r'T(\d{4})!'
    matches = re.findall(pattern, command)
    
    if not matches:
        return 1.0
    
    total_ms = sum(int(t) for t in matches)
    num_servos = len(matches)
    total_ms += (num_servos - 1) * 20
    
    return total_ms / 1000.0

class SerialController:
    def __init__(self, port_name="/dev/ttyUSB0", baudrate=115200):
        self.port_name = port_name
        self.baudrate = baudrate
        self.ser = None
    
    def open(self):
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
        if self.ser and self.ser.is_open:
            self.ser.close()

class CalibratorNode(Node):
    def __init__(self):
        super().__init__('calibrator_node')
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_color_image = None
        self.latest_depth_image = None
        self.got_camera_info = False
        self.got_color_image = False
        self.got_depth_image = False
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.color_image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_image_callback,
            10
        )
        
        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10
        )
        
        self.center_publisher = self.create_publisher(
            PointStamped,
            '/calibration_board_center',
            10
        )
        
        self.calibration_board_center = None
        self.center_received = False
        
        self.center_sub = self.create_subscription(
            PointStamped,
            '/calibration_board_center',
            self.center_callback,
            10
        )
        
        self.kinematics = Kinematics()
        self.serial_controller = None
        
        self.calibration_data = []
        self.calibration_poses = []
        
        x_values = (-80, -20, 20)
        y_values = (200, 280, 330)
        z_values = (100, 160, 220)
        
        for x in x_values:
            for y in y_values:
                for z in z_values:
                    self.calibration_poses.append((x, y, z))
        
        self.get_logger().info(f"生成了 {len(self.calibration_poses)} 个标定位置")
        self.current_pose_index = 0
        
        self.T = None
        
        self.init_check_timer = self.create_timer(2.0, self.check_initialization)
        
        self.get_logger().info('Calibrator node initialized')
        self.get_logger().info('等待相机数据...')
    
    def check_initialization(self):
        self.get_logger().info(f"初始化状态: camera_info={self.got_camera_info}, color_image={self.got_color_image}, depth_image={self.got_depth_image}")
        if self.got_camera_info and self.got_color_image and self.got_depth_image:
            self.get_logger().info("所有相机数据已就绪，可以开始标定")
            self.init_check_timer.cancel()
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.got_camera_info = True
    
    def color_image_callback(self, msg):
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.got_color_image = True
            
            if self.got_camera_info and self.got_depth_image:
                self.process_images(msg.header)
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting color image: {e}')
    
    def depth_image_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.got_depth_image = True
            
            if self.got_camera_info and self.got_color_image:
                if hasattr(self, 'latest_color_image') and self.latest_color_image is not None:
                    self.process_images(msg.header)
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def process_images(self, header):
        if self.camera_info is None or self.latest_color_image is None or self.latest_depth_image is None:
            if self.camera_info is None:
                self.get_logger().debug("camera_info is None")
            if self.latest_color_image is None:
                self.get_logger().debug("latest_color_image is None")
            if self.latest_depth_image is None:
                self.get_logger().debug("latest_depth_image is None")
            return
        
        self.get_logger().debug(f"Processing image: color shape={self.latest_color_image.shape}, depth shape={self.latest_depth_image.shape}")
        
        pattern_size = (3, 3)
        square_size = 0.03  # 角点距离为30mm，转换为米
        gray = cv2.cvtColor(self.latest_color_image, cv2.COLOR_BGR2GRAY)
        
        corners = self.find_corners_with_multiple_methods(gray, pattern_size)
        
        if corners is None:
            self.get_logger().warn("无法检测到标定板角点，请检查标定板是否在视野内")
            return
        
        self.get_logger().info(f"成功检测到标定板，找到 {len(corners)} 个角点")
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # 使用 solvePnP 计算标定板位姿
        camera_point = self.calculate_camera_pose(corners_refined, pattern_size, square_size)
        
        if camera_point is None:
            self.get_logger().warn("无法计算标定板位姿")
            return
        
        self.get_logger().info(f"标定板中心相机坐标: ({camera_point[0]:.4f}, {camera_point[1]:.4f}, {camera_point[2]:.4f})")
        
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.point.x = camera_point[0]
        point_msg.point.y = camera_point[1]
        point_msg.point.z = camera_point[2]
        
        self.center_publisher.publish(point_msg)
        self.get_logger().info("已发布标定板中心位置")
    
    def find_corners_with_multiple_methods(self, gray, pattern_size):
        methods = [
            ('original', gray),
            ('gaussian', cv2.GaussianBlur(gray, (5, 5), 0)),
            ('equalized', cv2.equalizeHist(gray)),
            ('clahe', cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)),
            ('sharpened', self.sharpen_image(gray)),
        ]
        
        flags_options = [
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            cv2.CALIB_CB_FAST_CHECK,
            0,
        ]
        
        for method_name, processed_img in methods:
            for flags in flags_options:
                ret, corners = cv2.findChessboardCorners(processed_img, pattern_size, flags=flags)
                if ret and len(corners) == pattern_size[0] * pattern_size[1]:
                    self.get_logger().debug(f"使用方法 {method_name} 和 flags={flags} 成功检测到角点")
                    return corners
        
        return None
    
    def sharpen_image(self, image):
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        return cv2.filter2D(image, -1, kernel)
    
    def calculate_center(self, corners):
        corners = corners.reshape(-1, 2)
        mean_x = np.mean(corners[:, 0])
        mean_y = np.mean(corners[:, 1])
        return (int(mean_x), int(mean_y))
    
    def calculate_camera_pose(self, corners, pattern_size, square_size):
        """使用 solvePnP 计算标定板在相机坐标系中的位姿
        
        Args:
            corners: 检测到的角点像素坐标
            pattern_size: 标定板角点模式 (rows, cols)
            square_size: 角点间距（米）
        
        Returns:
            标定板中心在相机坐标系中的坐标 (x, y, z)，失败返回 None
        """
        if self.camera_info is None:
            self.get_logger().error("相机内参未初始化")
            return None
        
        # 构建标定板角点的世界坐标
        object_points = []
        for i in range(pattern_size[0]):
            for j in range(pattern_size[1]):
                object_points.append([j * square_size, i * square_size, 0])
        
        object_points = np.array(object_points, dtype=np.float32)
        image_points = corners.astype(np.float32)
        
        # 获取相机内参矩阵和畸变系数
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        camera_matrix = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]], dtype=np.float32)
        
        dist_coeffs = np.array(self.camera_info.d, dtype=np.float32) if self.camera_info.d else np.zeros((5, 1), dtype=np.float32)
        
        # 使用 solvePnP 求解位姿
        try:
            ret, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            
            if not ret:
                self.get_logger().warn("solvePnP 求解失败")
                return None
            
            # tvec 就是标定板原点在相机坐标系中的位置
            # 我们需要计算标定板中心的位置
            # 标定板中心相对于原点的偏移
            center_offset_x = (pattern_size[1] - 1) * square_size / 2
            center_offset_y = (pattern_size[0] - 1) * square_size / 2
            
            # 将旋转向量转换为旋转矩阵
            rot_mat, _ = cv2.Rodrigues(rvec)
            
            # 标定板中心在相机坐标系中的坐标
            # center_camera = rot_mat @ [center_offset_x, center_offset_y, 0]^T + tvec
            center_local = np.array([center_offset_x, center_offset_y, 0], dtype=np.float32).reshape(3, 1)
            center_camera = rot_mat @ center_local + tvec
            
            x = float(center_camera[0][0])
            y = float(center_camera[1][0])
            z = float(center_camera[2][0])
            
            self.get_logger().debug(f"solvePnP 计算成功，标定板原点: ({tvec[0][0]:.4f}, {tvec[1][0]:.4f}, {tvec[2][0]:.4f})")
            
            return (x, y, z)
        except Exception as e:
            self.get_logger().error(f"solvePnP 计算异常: {e}")
            return None
    
    def get_depth_at_point(self, point, window_size=5):
        x, y = point
        height, width = self.latest_depth_image.shape
        
        half_window = window_size // 2
        min_x = max(0, x - half_window)
        max_x = min(width - 1, x + half_window)
        min_y = max(0, y - half_window)
        max_y = min(height - 1, y + half_window)
        
        depth_values = []
        for dy in range(min_y, max_y + 1):
            for dx in range(min_x, max_x + 1):
                depth_mm = self.latest_depth_image[dy, dx]
                if depth_mm > 0 and not np.isnan(depth_mm):
                    depth_values.append(depth_mm)
        
        if not depth_values:
            return 0.0
        
        depth_values.sort()
        median_start = len(depth_values) // 4
        median_end = len(depth_values) * 3 // 4
        
        if median_start >= median_end:
            median_start = 0
            median_end = len(depth_values)
        
        filtered_depths = depth_values[median_start:median_end]
        
        if not filtered_depths:
            return 0.0
        
        mean_depth_mm = statistics.mean(filtered_depths)
        return mean_depth_mm / 1000.0
    
    def get_depth_from_corners_area(self, corners):
        if self.latest_depth_image is None:
            self.get_logger().warn("深度图像为空")
            return 0.0
        
        height, width = self.latest_depth_image.shape
        
        corners_np = np.array(corners).reshape(-1, 2)
        
        min_x = max(0, int(np.min(corners_np[:, 0]) - 5))
        max_x = min(width - 1, int(np.max(corners_np[:, 0]) + 5))
        min_y = max(0, int(np.min(corners_np[:, 1]) - 5))
        max_y = min(height - 1, int(np.max(corners_np[:, 1]) + 5))
        
        self.get_logger().debug(f"深度区域: x=[{min_x}, {max_x}], y=[{min_y}, {max_y}]")
        
        depth_values = []
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                depth_mm = self.latest_depth_image[y, x]
                if depth_mm > 0 and not np.isnan(depth_mm) and not np.isinf(depth_mm):
                    depth_values.append(depth_mm)
        
        if not depth_values:
            self.get_logger().warn("角点区域内没有有效深度值")
            return 0.0
        
        self.get_logger().debug(f"角点区域内共有 {len(depth_values)} 个有效深度值")
        
        depth_values.sort()
        median_depth_mm = depth_values[len(depth_values) // 2]
        
        self.get_logger().debug(f"中值滤波后的深度: {median_depth_mm} mm")
        
        return median_depth_mm / 1000.0
    
    def convert_to_camera_coordinate(self, point, depth):
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        x = (point[0] - cx) * depth / fx
        y = (point[1] - cy) * depth / fy
        z = depth
        
        return (x, y, z)
    
    def center_callback(self, msg):
        self.calibration_board_center = (msg.point.x, msg.point.y, msg.point.z)
        self.center_received = True
        self.get_logger().info(f'Received calibration board center: ({msg.point.x:.4f}, {msg.point.y:.4f}, {msg.point.z:.4f})')
    
    def init_serial(self, port_name="/dev/ttyUSB0", baudrate=115200):
        self.serial_controller = SerialController(port_name, baudrate)
        return self.serial_controller.open()
    
    def set_arm_lengths(self, L0=100, L1=105, L2=88, L3=200):
        self.kinematics.setup_kinematics(L0, L1, L2, L3)
    
    def set_servo(self, index, pwm, time=1500):
        if self.serial_controller is None:
            self.get_logger().error("串口未初始化！")
            return (0.0, False)
        
        if index < 0 or index > 5:
            self.get_logger().error(f"舵机编号必须在0-5之间，当前为 {index}")
            return (0.0, False)
        
        if pwm < 0 or pwm > 3000:
            self.get_logger().error(f"PWM值必须在0-3000之间，当前为 {pwm}")
            return (0.0, False)
        
        if index == 3:
            pwm_value = 3000 - pwm
        else:
            pwm_value = pwm
        
        command = "#{:03d}P{:04d}T{:04d}!".format(index, pwm_value, time)
        
        self.get_logger().info(f"设置舵机 {index}: PWM={pwm_value}, 时间={time}ms")
        
        success = self.serial_controller.send_command(command)
        
        exec_time = time / 1000.0 + 0.02
        return (exec_time, success)
    
    def move_to(self, x, y, z, time=1500):
        if self.serial_controller is None:
            self.get_logger().error("串口未初始化！")
            return (0.0, False)
        
        command = self.kinematics.kinematics_move(x, y, z, time)

        self.get_logger().info(f"生成的运动学命令: {command}")
        
        if command is None:
            self.get_logger().error("无法找到有效的逆运动学解！")
            return (0.0, False)
        
        exec_time = calculate_execution_time(command)
        
        success = self.serial_controller.send_command(command)
        
        return (exec_time, success)
    
    def get_calibration_board_center(self, timeout=30.0, sample_count=3, stability_threshold=0.03):
        """获取标定板中心位置，增加稳定性检测（改进版）
        
        Args:
            timeout: 总超时时间（秒）
            sample_count: 采样数量（降低要求）
            stability_threshold: 稳定性阈值（米），标准差小于此值认为稳定（增大阈值）
        
        Returns:
            标定板中心坐标 (x, y, z)，失败返回 None
        """
        self.center_received = False
        self.calibration_board_center = None
        
        start_time = time.time()
        samples = []
        
        self.get_logger().info(f"开始获取标定板中心，需要采集 {sample_count} 个样本...")
        
        while (time.time() - start_time) < timeout and len(samples) < sample_count:
            # 重置状态
            self.center_received = False
            self.calibration_board_center = None
            
            # 等待单个样本（增加等待时间）
            sample_start = time.time()
            while (time.time() - sample_start) < 2.0:  # 每个样本最多等待2秒
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.center_received and self.calibration_board_center is not None:
                    samples.append(self.calibration_board_center)
                    self.get_logger().info(f"获取到样本 {len(samples)}/{sample_count}: {self.calibration_board_center}")
                    break
        
        if len(samples) == 0:
            self.get_logger().error(f"未获取到任何样本！")
            return None
        
        if len(samples) < sample_count:
            self.get_logger().warn(f"采样不足，仅获取到 {len(samples)} 个样本（需要 {sample_count} 个），尝试使用现有样本...")
        
        # 检查稳定性
        samples_np = np.array(samples)
        mean_point = np.mean(samples_np, axis=0)
        std_dev = np.std(samples_np, axis=0)
        
        self.get_logger().info(f"采样完成，均值: ({mean_point[0]:.4f}, {mean_point[1]:.4f}, {mean_point[2]:.4f})")
        self.get_logger().info(f"标准差: ({std_dev[0]:.4f}, {std_dev[1]:.4f}, {std_dev[2]:.4f})")
        
        # 如果样本数量足够且稳定，返回均值
        if len(samples) >= sample_count and np.all(std_dev < stability_threshold):
            self.get_logger().info("标定板位置稳定，返回均值")
            return (float(mean_point[0]), float(mean_point[1]), float(mean_point[2]))
        elif len(samples) >= 2:
            # 如果样本数量不够但至少有2个，仍然返回均值（宽松模式）
            self.get_logger().warn(f"标定板位置稳定性一般，但继续使用（标准差 {std_dev}）")
            return (float(mean_point[0]), float(mean_point[1]), float(mean_point[2]))
        else:
            # 只有一个样本，直接返回
            self.get_logger().warn("只有一个样本，直接返回")
            return samples[0]
    
    def run_calibration(self):
        self.get_logger().info("开始手眼标定流程...")
        
        if not self.init_serial("/dev/ttyUSB0", 115200):
            self.get_logger().error("无法初始化串口")
            return False
        
        self.set_arm_lengths()
        
        self.get_logger().info("关闭夹抓...")

        # exec_time, success = self.set_servo(5, 1860, 2000)
        # if success:
        #     self.get_logger().info("等待夹抓关闭完成...")
        #     time.sleep(exec_time + 10.0)
        
        self.get_logger().info("初始化舵机...")
        exec_time, success = self.move_to(0, 200, 200, time=2000)
        if success:
            time.sleep(exec_time + 2.0)
        
        self.calibration_data = []
        
        total_iterations = len(self.calibration_poses)
        current_iteration = 0
        
        for x, y, z in self.calibration_poses:
            current_iteration += 1
            self.get_logger().info(f"移动到位置 {current_iteration}/{total_iterations}: ({x}, {y}, {z})")
            
            exec_time, success = self.move_to(x, y, z, time=2000)
            if not success:
                self.get_logger().error(f"移动到位置 ({x}, {y}, {z}) 失败")
                continue
            
            self.get_logger().info(f"等待机械臂运动完成...")
            time.sleep(exec_time + 8.0)  # 增加等待时间，确保稳定
            
            # 增加重试机制
            max_retries = 3
            camera_center = None
            for retry in range(max_retries):
                self.get_logger().info(f"获取标定板中心位置... (尝试 {retry + 1}/{max_retries})")
                camera_center = self.get_calibration_board_center(timeout=20.0)
                
                if camera_center is not None:
                    break
                
                self.get_logger().warn(f"第 {retry + 1} 次尝试失败，等待1秒后重试...")
                time.sleep(1.0)
            
            if camera_center is None:
                self.get_logger().warn(f"未能获取位置 ({x}, {y}, {z}) 的标定板中心，跳过")
                continue
            
            xc, yc, zc = camera_center
            
            if abs(xc) > 0.9 or abs(yc) > 0.9 or abs(zc) > 0.9:
                self.get_logger().warn(f"相机位置超出范围 ({xc:.6f}, {yc:.6f}, {zc:.6f})米，跳过")
                continue
            
            xc, yc, zc = xc * 1000, yc * 1000, zc * 1000
            # xcc = -zc
            # ycc = xc
            # zcc = -yc
            # transformed_center = (xcc, ycc, zcc)
            self.get_logger().info(f"原始相机位置: ({xc/1000:.6f}, {yc/1000:.6f}, {zc/1000:.6f})米")
            
            self.calibration_data.append({
                'arm_pose': (x, y, z),
                'camera_center': (xc, yc, zc),
                'camera_center_original': camera_center
            })
            
            self.get_logger().info(f"记录数据: 机械臂位置=({x}, {y}, {z}), 相机位置={camera_center}")
        
        self.serial_controller.close()
        
        if len(self.calibration_data) < 3:
            self.get_logger().error(f"标定数据不足，需要至少3组，实际获得 {len(self.calibration_data)} 组")
            return False
        
        self.get_logger().info(f"标定数据收集完成，共 {len(self.calibration_data)} 组")
        
        self.save_calibration_data()
        
        self.filter_outliers()
        
        self.compute_hand_eye_transformation()
        
        return True
    
    def filter_outliers(self):
        if len(self.calibration_data) < 3:
            return
        
        filtered_data = []
        for data in self.calibration_data:
            center = data['camera_center']
            if center[0] > 900:
                self.get_logger().warn(f"剔除异常数据: 相机坐标X={center[0]:.4f}毫米 > 900毫米 (0.9米)")
            else:
                filtered_data.append(data)
        
        removed_count = len(self.calibration_data) - len(filtered_data)
        self.calibration_data = filtered_data
        
        self.get_logger().info(f"异常值过滤完成，剔除 {removed_count} 个数据对，剩余 {len(self.calibration_data)} 个")
    
    def save_calibration_data(self):
        try:
            pkg_root = os.path.dirname(os.path.dirname(__file__))
            data_file = os.path.join(pkg_root, 'result', 'data.txt')
            os.makedirs(os.path.dirname(data_file), exist_ok=True)
            
            with open(data_file, 'w', encoding='utf-8') as f:
                for i, data in enumerate(self.calibration_data, 1):
                    camera_original = data['camera_center_original']
                    camera_transformed = data['camera_center']
                    arm_pose = data['arm_pose']
                    
                    f.write(f"相机初始值：({camera_original[0]:.6f}, {camera_original[1]:.6f}, {camera_original[2]:.6f})  ")
                    f.write(f"相机变换后值：({camera_transformed[0]:.6f}, {camera_transformed[1]:.6f}, {camera_transformed[2]:.6f})  ")
                    f.write(f"机械臂位置：({arm_pose[0]:.2f}, {arm_pose[1]:.2f}, {arm_pose[2]:.2f})")
                    f.write("\n")
            
            self.get_logger().info(f"标定数据已保存到: {data_file}")
        except Exception as e:
            self.get_logger().error(f"保存标定数据失败: {e}")
    
    def compute_hand_eye_transformation(self):
        self.get_logger().info("计算手眼变换矩阵...")
        
        arm_poses = np.array([data['arm_pose'] for data in self.calibration_data])
        camera_centers = np.array([data['camera_center'] for data in self.calibration_data])
        
        self.get_logger().info(f"数据点数: {len(arm_poses)}")
        
        if len(arm_poses) < 3:
            self.get_logger().error("有效数据点不足，无法计算变换矩阵")
            return
        
        # 1. 计算质心 (Centroids)
        centroid_cam = np.mean(camera_centers, axis=0)
        centroid_rob = np.mean(arm_poses, axis=0)
        
        self.get_logger().info(f"相机点集质心: {centroid_cam}")
        self.get_logger().info(f"机械臂点集质心: {centroid_rob}")
        
        # 2. 去中心化 (Centering the data)
        cam_centered = camera_centers - centroid_cam
        rob_centered = arm_poses - centroid_rob
        
        # 3. 构建协方差矩阵 H
        H = np.dot(cam_centered.T, rob_centered)
        
        self.get_logger().info(f"协方差矩阵 H:\n{H}")
        
        # 4. 奇异值分解 (SVD)
        U, S, Vt = np.linalg.svd(H)
        
        # 5. 计算旋转矩阵 R
        R = np.dot(Vt.T, U.T)
        
        # 检查行列式，确保是右手坐标系旋转矩阵 (det(R) == 1)
        if np.linalg.det(R) < 0:
            self.get_logger().info("检测到反射，修正旋转矩阵...")
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        self.get_logger().info(f"旋转矩阵 R:\n{R}")
        self.get_logger().info(f"行列式 det(R) = {np.linalg.det(R):.6f}")
        
        # 6. 计算平移向量 t
        t = centroid_rob - np.dot(R, centroid_cam)
        
        self.get_logger().info(f"平移向量 t: {t}")
        
        # 7. 构建 4x4 齐次变换矩阵 T
        self.T = np.eye(4)
        self.T[:3, :3] = R
        self.T[:3, 3] = t
        
        self.get_logger().info(f"手眼标定完成！")
        self.get_logger().info(f"4x4 齐次变换矩阵 T:\n{self.T}")
        
        self.save_calibration_result(1.0, R, t)
        
        self.validate_calibration()
    
    def validate_calibration(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("开始验证标定结果...")
        self.get_logger().info("=" * 50)
        
        arm_poses = np.array([data['arm_pose'] for data in self.calibration_data])
        camera_centers = np.array([data['camera_center'] for data in self.calibration_data])
        
        errors = []
        max_error = 0.0
        min_error = float('inf')
        
        self.get_logger().info(f"{'序号':<6} {'实际位置 (mm)':<35} {'预测位置 (mm)':<35} {'误差 (mm)':<10}")
        self.get_logger().info("-" * 90)
        
        for i, (arm_pose, camera_center) in enumerate(zip(arm_poses, camera_centers)):
            camera_homogeneous = np.array([camera_center[0], camera_center[1], camera_center[2], 1.0])
            predicted_arm_homogeneous = self.T @ camera_homogeneous
            predicted_arm = predicted_arm_homogeneous[:3]
            
            error = np.linalg.norm(predicted_arm - arm_pose)
            errors.append(error)
            max_error = max(max_error, error)
            min_error = min(min_error, error)
            
            actual_str = f"({arm_pose[0]:.2f}, {arm_pose[1]:.2f}, {arm_pose[2]:.2f})"
            predicted_str = f"({predicted_arm[0]:.2f}, {predicted_arm[1]:.2f}, {predicted_arm[2]:.2f})"
            
            self.get_logger().info(f"{i+1:<6} {actual_str:<35} {predicted_str:<35} {error:<10.4f}")
        
        mean_error = np.mean(errors)
        std_error = np.std(errors)
        
        self.get_logger().info("-" * 90)
        self.get_logger().info(f"误差统计:")
        self.get_logger().info(f"  最大误差: {max_error:.4f} mm")
        self.get_logger().info(f"  最小误差: {min_error:.4f} mm")
        self.get_logger().info(f"  平均误差: {mean_error:.4f} mm")
        self.get_logger().info(f"  标准差:   {std_error:.4f} mm")
        self.get_logger().info("=" * 50)
        
        if mean_error < 5.0:
            self.get_logger().info("标定质量: 优秀 (平均误差 < 5mm)")
        elif mean_error < 10.0:
            self.get_logger().info("标定质量: 良好 (5mm <= 平均误差 < 10mm)")
        elif mean_error < 20.0:
            self.get_logger().info("标定质量: 一般 (10mm <= 平均误差 < 20mm)")
        else:
            self.get_logger().warn("标定质量: 较差 (平均误差 >= 20mm)，建议重新标定")
        
        self.get_logger().info("=" * 50)
    
    def save_calibration_result(self, scale, R, t):
        result_file = os.path.join(os.path.dirname(__file__), 'calibration_result.yaml')
        
        result_data = {
            'calibration': {
                'timestamp': datetime.now().isoformat(),
                'scale': float(scale),
                'rotation_matrix': R.flatten().tolist(),
                'translation_vector': t.flatten().tolist(),
                'transformation_matrix': self.T.flatten().tolist()
            }
        }
        
        with open(result_file, 'w') as f:
            yaml.dump(result_data, f, default_flow_style=None, sort_keys=False)
        
        self.get_logger().info(f"标定结果已保存到：{result_file}")
    
    def camera_to_arm_coordinate(self, camera_point):
        if self.T is None:
            self.get_logger().error("手眼标定未完成！")
            return None
        
        camera_homogeneous = np.array([camera_point[0], camera_point[1], camera_point[2], 1.0])
        arm_homogeneous = self.T @ camera_homogeneous
        
        return tuple(arm_homogeneous[:3])

def main(args=None):
    rclpy.init(args=args)
    node = CalibratorNode()
    
    try:
        node.run_calibration()
        
        if node.T is not None:
            node.get_logger().info("手眼标定成功完成！")
            node.get_logger().info("可以使用 camera_to_arm_coordinate() 将相机坐标转换为机械臂坐标")
    
    except KeyboardInterrupt:
        node.get_logger().info("标定过程被中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()