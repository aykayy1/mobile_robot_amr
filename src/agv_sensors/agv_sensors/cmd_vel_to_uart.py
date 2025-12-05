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

        # Tính max linear (m/s)
        self.max_linear = 2.0 * math.pi * self.wheel_radius * self.max_rpm_linear / 60.0

        # Tính max angular (rad/s)
        factor = 60.0 / (2.0 * math.pi * self.wheel_radius)
        self.max_angular = (self.max_rpm_angular / factor) * 2.0 / self.wheel_base

        # ============ Tối ưu dữ liệu joystick ============
        # Smoothing (lọc nhiễu joystick)
        self.smooth_gain = 0.2      # 0.1–0.3 mượt, 0.5 nhanh hơn

        # Deadband từ joystick
        self.linear_deadband = 0.02
        self.angular_deadband_js = 0.02

        # Deadband quay
        self.angular_deadband = 0.25  # rad/s

        # Deadband rpm
        self.rpm_deadband = 1.0

        # Giảm tần số gửi xuống STM32
        self.timer_period = 0.1  # 10 Hz

        # Bộ nhớ cmd_vel sau khi smoothing
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.last_rpm_left = 0
        self.last_rpm_right = 0

        # Đảo chiều bánh
        self.invert_left = True
        self.invert_right = False

        # Gain hai bánh
        self.left_gain  = 0.9
        self.right_gain = 0.9

        self.get_logger().info(
            f'Opening UART to STM32 on {port} @ {baud} '
            f'(v_max={self.max_linear:.4f} m/s, '
            f'linear_max_rpm={self.max_rpm_linear}, '
            f'angular_max_rpm={self.max_rpm_angular})'
        )

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        # Subscriber
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Publisher debug
        self.motor_pub = self.create_publisher(
            Int16MultiArray, '/motor_cmd', 10
        )

        # Timer gửi lệnh ổn định 10Hz
        self.timer = self.create_timer(self.timer_period, self.process_cmd_vel)

        self.get_logger().info('CmdVelToUARTNode started.')

    # ========== Callback lưu dữ liệu joystick + smoothing ==========
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Deadband joystick
        if abs(v) < self.linear_deadband:
            v = 0.0
        if abs(w) < self.angular_deadband_js:
            w = 0.0

        # Low-pass filter (EMA smoothing)
        self.cmd_v = self.cmd_v * (1 - self.smooth_gain) + v * self.smooth_gain
        self.cmd_w = self.cmd_w * (1 - self.smooth_gain) + w * self.smooth_gain

    # ========== Hàm xử lý và gửi lệnh xuống STM32 ổn định ==========
    def process_cmd_vel(self):
        v = self.cmd_v
        w = self.cmd_w

        # Deadzone quay
        if abs(w) < self.angular_deadband:
            w = 0.0

        # Clamp v and w
        v = max(min(v, self.max_linear), -self.max_linear)
        w = max(min(w, self.max_angular), -self.max_angular)

        # v,w -> rpm
        L = self.wheel_base
        v_left  = v - w * L / 2.0
        v_right = v + w * L / 2.0

        factor = 60.0 / (2.0 * math.pi * self.wheel_radius)

        rpm_left_f  = v_left * factor
        rpm_right_f = v_right * factor

        # Giới hạn rpm
        rpm_left_f  = max(min(rpm_left_f,  self.max_rpm_linear), -self.max_rpm_linear)
        rpm_right_f = max(min(rpm_right_f, self.max_rpm_linear), -self.max_rpm_linear)

        # Gain
        rpm_left_f  *= self.left_gain
        rpm_right_f *= self.right_gain

        # Deadband rpm
        if abs(rpm_left_f) < self.rpm_deadband:
            rpm_left_f = 0.0
        if abs(rpm_right_f) < self.rpm_deadband:
            rpm_right_f = 0.0

        rpm_left = int(rpm_left_f)
        rpm_right = int(rpm_right_f)

        # Không gửi nếu khác quá nhỏ (< 2 rpm)
        if abs(rpm_left - self.last_rpm_left) < 2 and abs(rpm_right - self.last_rpm_right) < 2:
            return

        self.last_rpm_left  = rpm_left
        self.last_rpm_right = rpm_right

        # Publish debug
        motor_msg = Int16MultiArray()
        motor_msg.data = [rpm_left, rpm_right]
        self.motor_pub.publish(motor_msg)

        # Gửi UART
        self.send_to_stm32(rpm_left, rpm_right)

    # ========== Gửi UART ==========
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
