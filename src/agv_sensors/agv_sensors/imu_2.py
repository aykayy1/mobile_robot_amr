#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import serial
from serial import SerialException


# Lệnh cấu hình WT901C
CMD_SET_BAUD_115200 = bytes([0xFF, 0xAA, 0x04, 0x08, 0x00])
CMD_SET_6_AXIS      = bytes([0xFF, 0xAA, 0x02, 0x00, 0x00])


def to_int16(lo: int, hi: int) -> int:
    return struct.unpack('<h', bytes([lo, hi]))[0]


class W901ImuNode(Node):
    def __init__(self):
        super().__init__('w901_imu_node')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)   # chỉ dùng để đọc dữ liệu sau khi ép
        self.declare_parameter('frame_id', 'imu_link')

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value

        # ÉP CẤU HÌNH IMU TRƯỚC
        self.force_config()

        # Sau khi ép xong → mở lại IMU ở 115200 để đọc dữ liệu
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=115200,
                timeout=0.05
            )
            self.get_logger().info("Đã mở IMU @115200 sau khi ép cấu hình.")
        except Exception as e:
            self.get_logger().error(f"Lỗi mở IMU @115200: {e}")
            raise

        self.pub = self.create_publisher(Imu, '/imu', 10)

        self.ax = self.ay = self.az = 0.0
        self.wx = self.wy = self.wz = 0.0

        self.timer = self.create_timer(0.01, self.read_and_publish)


    def force_config(self):
        """Ép IMU chuyển sang 115200 + tắt 9-axis."""

        self.get_logger().info("Mở IMU ở 9600 để ép cấu hình...")

        try:
            ser = serial.Serial(self.port, 9600, timeout=0.2)
        except Exception as e:
            self.get_logger().error(f"Không mở được IMU @9600 để cấu hình: {e}")
            return

        # Gửi lệnh tắt 9-axis
        self.get_logger().info("Gửi lệnh ép 6-axis...")
        ser.write(CMD_SET_6_AXIS)
        ser.flush()
        time.sleep(0.1)

        # Gửi lệnh đổi baudrate
        self.get_logger().info("Gửi lệnh ép baudrate 115200...")
        ser.write(CMD_SET_BAUD_115200)
        ser.flush()
        time.sleep(0.1)

        ser.close()
        self.get_logger().info("Đã gửi xong. Đóng cổng 9600 và chuẩn bị mở 115200.")


    def read_frame(self):
        while True:
            head = self.ser.read(1)
            if len(head) == 0:
                return None
            if head[0] != 0x55:
                continue

            type_byte = self.ser.read(1)
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

        if ftype == 0x51:  # Acc
            ax = to_int16(d[0], d[1])
            ay = to_int16(d[2], d[3])
            az = to_int16(d[4], d[5])
            scale = 16 * 9.80665 / 32768.0
            self.ax, self.ay, self.az = ax * scale, ay * scale, az * scale

        elif ftype == 0x52:  # Gyro
            wx = to_int16(d[0], d[1])
            wy = to_int16(d[2], d[3])
            wz = to_int16(d[4], d[5])
            scale = 2000.0 * math.pi / 180 / 32768.0
            self.wx, self.wy, self.wz = wx * scale, wy * scale, wz * scale

        elif ftype == 0x53:
            # KHÔNG DÙNG GÓC – BỎ QUA
            return


    def read_and_publish(self):
        try:
            for _ in range(10):
                frame = self.read_frame()
                if frame:
                    self.parse_frame(frame)
        except:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation fixed (không dùng 9-axis)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        msg.angular_velocity.x = self.wx
        msg.angular_velocity.y = self.wy
        msg.angular_velocity.z = self.wz

        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = self.az

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = W901ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
