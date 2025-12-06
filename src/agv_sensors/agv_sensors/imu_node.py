#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import serial
from serial import SerialException


def to_int16(lo: int, hi: int) -> int:
    return struct.unpack('<h', bytes([lo, hi]))[0]


def yaw_to_quat(yaw: float):
    """
    Quaternion chỉ quay quanh Z:
    roll = pitch = 0
    """
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


class W901ImuNode(Node):
    def __init__(self):
        super().__init__('w901_imu_node')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.frame_id = self.get_parameter('frame_id').value

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05
            )
            self.get_logger().info(f'Kết nối IMU WT901C trên {port} @ {baud} baud')
        except SerialException as e:
            self.get_logger().error(f'Không mở được serial: {e}')
            raise

        self.pub = self.create_publisher(Imu, '/imu', 10)

        # Accel, gyro
        self.ax = self.ay = self.az = 0.0
        self.wx = self.wy = self.wz = 0.0

        # Chỉ dùng yaw
        self.yaw = 0.0
        self.qx = self.qy = self.qz = 0.0
        self.qw = 1.0

        self.timer = self.create_timer(0.01, self.read_and_publish)

    def read_frame(self):
        while True:
            head = self.ser.read(1)
            if len(head) == 0:
                return None
            if head[0] != 0x55:
                continue

            type_byte = self.ser.read(1)
            if len(type_byte) == 0:
                return None

            data = self.ser.read(9)
            if len(data) < 9:
                return None

            frame = bytes([0x55, type_byte[0]]) + data
            checksum = sum(frame[0:10]) & 0xFF
            if checksum != frame[10]:
                continue
            return frame

    def parse_frame(self, frame: bytes):
        ftype = frame[1]
        d = frame[2:10]

        # 0x51: acceleration
        if ftype == 0x51:
            ax_raw = to_int16(d[0], d[1])
            ay_raw = to_int16(d[2], d[3])
            az_raw = to_int16(d[4], d[5])

            g = 9.80665
            scale = 16.0 * g / 32768.0
            self.ax = ax_raw * scale
            self.ay = ay_raw * scale
            self.az = az_raw * scale

        # 0x52: gyro
        elif ftype == 0x52:
            wx_raw = to_int16(d[0], d[1])
            wy_raw = to_int16(d[2], d[3])
            wz_raw = to_int16(d[4], d[5])

            deg2rad = math.pi / 180.0
            scale = 2000.0 * deg2rad / 32768.0
            self.wx = wx_raw * scale
            self.wy = wy_raw * scale
            self.wz = wz_raw * scale

        # 0x53: angles -> lấy riêng yaw
        elif ftype == 0x53:
            # roll_raw  = to_int16(d[0], d[1])   # bỏ
            # pitch_raw = to_int16(d[2], d[3])   # bỏ
            yaw_raw   = to_int16(d[4], d[5])

            # datasheet: yaw(rad) = raw/32768*pi
            self.yaw = yaw_raw / 32768.0 * math.pi

            self.qx, self.qy, self.qz, self.qw = yaw_to_quat(self.yaw)

    def read_and_publish(self):
        try:
            for _ in range(10):
                frame = self.read_frame()
                if frame is None:
                    break
                self.parse_frame(frame)
        except SerialException as e:
            self.get_logger().error(f'Lỗi đọc serial: {e}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation: chỉ yaw
        msg.orientation.x = self.qx
        msg.orientation.y = self.qy
        msg.orientation.z = self.qz
        msg.orientation.w = self.qw

        # Gyro: bạn vẫn giữ wz để EKF dùng tốc độ quay
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = self.wz

        # Accel (nếu EKF không cần có thể set 0)
        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = self.az

        # Covariance: roll/pitch rất “tệ” (không dùng), yaw tốt
        big = 1e3  # cho EKF biết roll/pitch không tin
        yaw_std = math.radians(2.0)  # bạn thấy yaw tốt -> để nhỏ
        msg.orientation_covariance = [
            big, 0.0, 0.0,
            0.0, big, 0.0,
            0.0, 0.0, yaw_std**2
        ]

        # Gyro covariance: chỉ tin z
        gyro_std_z = math.radians(0.5)
        msg.angular_velocity_covariance = [
            big, 0.0, 0.0,
            0.0, big, 0.0,
            0.0, 0.0, gyro_std_z**2
        ]

        # Acc covariance: để vừa phải (hoặc big nếu không dùng accel)
        acc_std = 0.1
        msg.linear_acceleration_covariance = [
            acc_std**2, 0.0, 0.0,
            0.0, acc_std**2, 0.0,
            0.0, 0.0, acc_std**2
        ]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = W901ImuNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
