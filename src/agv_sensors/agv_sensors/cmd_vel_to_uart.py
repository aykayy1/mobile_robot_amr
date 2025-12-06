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
        # - Chạy thẳng: 95 rpm
        # - Quay tại chỗ: 20 rpm
        self.max_rpm_linear  = 95.0
        self.max_rpm_angular = 20.0

        # v_max (m/s) tương ứng với 95 rpm
        self.max_linear = 2.0 * math.pi * self.wheel_radius * self.max_rpm_linear / 60.0

        # Tính max_angular sao cho:
        # v = 0, w = max_angular  => rpm_left = -20, rpm_right = +20
        # Công thức:
        #   v_left  = -w * L / 2
        #   v_right =  w * L / 2
        #   rpm = v * factor,  factor = 60 / (2πR)
        #   => max_angular = (max_rpm_angular / factor) * 2 / L
        factor = 60.0 / (2.0 * math.pi * self.wheel_radius)
        self.max_angular = (self.max_rpm_angular / factor) * 2.0 / self.wheel_base

        # Giới hạn tần số gửi
        self.min_send_period = 0.1  # s
        self.last_send_time = self.get_clock().now()

        self.last_rpm_left = 0
        self.last_rpm_right = 0

        # Đảo chiều: bánh TRÁI âm khi đi tới
        self.invert_left = True
        self.invert_right = False

        # ===== GAIN HAI BÁNH =====
        # Bánh trái đang nhanh hơn -> giảm gain
        self.left_gain  = 0.9   # nếu vẫn nhanh, giảm xuống 0.5 / 0.4
        self.right_gain = 0.9

        # Deadzone
        self.angular_deadband = 0.25  # rad/s, chống lệch khi muốn đi thẳng
        self.rpm_deadband = 1.0       # rpm

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

        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.motor_pub = self.create_publisher(
            Int16MultiArray, '/motor_cmd', 10
        )

        self.get_logger().info('CmdVelToUARTNode started.')

    def cmd_vel_callback(self, msg: Twist):
        # Giới hạn tần số gửi
        now = self.get_clock().now()
        dt = (now - self.last_send_time).nanoseconds / 1e9
        if dt < self.min_send_period:
            return

        v = msg.linear.x
        w = msg.angular.z

        # Deadzone quay
        if abs(w) < self.angular_deadband:
            w = 0.0

        # Clamp theo v_max, w_max đã tách riêng
        v = max(min(v, self.max_linear),  -self.max_linear)
        w = max(min(w, self.max_angular), -self.max_angular)

        L = self.wheel_base

        # v,w -> v_left, v_right (m/s)
        v_left  = v - w * L / 2.0
        v_right = v + w * L / 2.0

        # m/s -> rpm
        if self.wheel_radius > 0.0:
            factor = 60.0 / (2.0 * math.pi * self.wheel_radius)
        else:
            factor = 0.0

        rpm_left_f  = v_left  * factor
        rpm_right_f = v_right * factor

        # Giới hạn rpm tổng thể theo max_rpm_linear (95 rpm)
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

        # Đảo dấu theo hướng thực tế
        if self.invert_left:
            rpm_left = -rpm_left
        if self.invert_right:
            rpm_right = -rpm_right

        # Không gửi lại nếu lệnh y như cũ
        if rpm_left == self.last_rpm_left and rpm_right == self.last_rpm_right:
            return

        self.last_send_time = now
        self.last_rpm_left  = rpm_left
        self.last_rpm_right = rpm_right

        # Publish debug
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