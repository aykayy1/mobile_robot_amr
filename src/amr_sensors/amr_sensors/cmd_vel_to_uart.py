#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

import serial
import math


class CmdVelToUARTNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_uart')

        # ===== UART khớp STM32 =====
        port = '/dev/ttyACM0'
        baud = 38400

        # ===== Thông số cơ khí =====
        self.wheel_radius = 0.10   # m
        self.wheel_base   = 0.636  # m

        # Giới hạn rpm bánh
        self.max_rpm_linear  = 95.0
        self.max_rpm_angular = 20.0

        # v_max (m/s) tương ứng với 95 rpm
        self.max_linear = 2.0 * math.pi * self.wheel_radius * self.max_rpm_linear / 60.0

        # Tính max_angular
        factor = 60.0 / (2.0 * math.pi * self.wheel_radius)
        self.max_angular = (self.max_rpm_angular / factor) * 2.0 / self.wheel_base

        # ===== GIỚI HẠN TẦN SỐ GỬI (5 Hz) =====
        self.min_send_period = 0.2  # 0.2s = 5 Hz
        self.last_send_time = self.get_clock().now()

        self.last_rpm_left = 0
        self.last_rpm_right = 0

        # Đảo chiều bánh
        self.invert_left = True
        self.invert_right = False

        # Gain hai bánh
        self.left_gain  = 0.9
        self.right_gain = 0.9

        # Deadzone
        self.angular_deadband = 0.25
        self.rpm_deadband = 1.0

        self.get_logger().info(
            f'Opening UART to STM32 on {port} @ {baud} '
            f'(v_max={self.max_linear:.4f} m/s)'
        )

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.motor_pub = self.create_publisher(
            Int16MultiArray, '/motor_cmd', 10
        )

        self.get_logger().info('CmdVelToUARTNode started.')

    def cmd_vel_callback(self, msg: Twist):
        # Giới hạn tần số gửi (5 Hz)
        now = self.get_clock().now()
        dt = (now - self.last_send_time).nanoseconds / 1e9
        if dt < self.min_send_period:
            return

        v = msg.linear.x
        w = msg.angular.z

        # Deadzone quay
        if abs(w) < self.angular_deadband:
            w = 0.0

        # Clamp tốc độ
        v = max(min(v, self.max_linear),  -self.max_linear)
        w = max(min(w, self.max_angular), -self.max_angular)

        L = self.wheel_base

        # v,w -> vận tốc bánh
        v_left  = v - w * L / 2.0
        v_right = v + w * L / 2.0

        factor = 60.0 / (2.0 * math.pi * self.wheel_radius)
        rpm_left_f  = v_left  * factor
        rpm_right_f = v_right * factor

        # Giới hạn rpm
        rpm_left_f  = max(min(rpm_left_f,  self.max_rpm_linear), -self.max_rpm_linear)
        rpm_right_f = max(min(rpm_right_f, self.max_rpm_linear), -self.max_rpm_linear)

        # Áp dụng gain
        rpm_left_f  *= self.left_gain
        rpm_right_f *= self.right_gain

        # Deadband rpm
        if abs(rpm_left_f) < self.rpm_deadband:
            rpm_left_f = 0.0
        if abs(rpm_right_f) < self.rpm_deadband:
            rpm_right_f = 0.0

        rpm_left  = int(rpm_left_f)
        rpm_right = int(rpm_right_f)

        # Đảo dấu
        if self.invert_left:
            rpm_left = -rpm_left
        if self.invert_right:
            rpm_right = -rpm_right

        # ===== ANTI-FLOOD: CHỈ GỬI KHI THAY ĐỔI ĐỦ LỚN =====
        rpm_change_threshold = 2  # rpm
        if abs(rpm_left - self.last_rpm_left) < rpm_change_threshold and \
           abs(rpm_right - self.last_rpm_right) < rpm_change_threshold:
            return

        # Không gửi nếu giống hệt lệnh cũ
        if rpm_left == self.last_rpm_left and rpm_right == self.last_rpm_right:
            return

        # Cập nhật
        self.last_send_time = now
        self.last_rpm_left  = rpm_left
        self.last_rpm_right = rpm_right

        # Publish ROS topic debug
        motor_msg = Int16MultiArray()
        motor_msg.data = [rpm_left, rpm_right]
        self.motor_pub.publish(motor_msg)

        # Gửi UART
        self.send_to_stm32(rpm_left, rpm_right)

    def send_to_stm32(self, rpm_left: int, rpm_right: int):
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return
        try:
            line = f"{rpm_left},{rpm_right}\n"
            self.ser.write(line.encode('ascii'))
            self.get_logger().info(f"TX: {line.strip()}")
        except Exception as e:
            self.get_logger().error(f'Error sending to STM32: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUARTNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
