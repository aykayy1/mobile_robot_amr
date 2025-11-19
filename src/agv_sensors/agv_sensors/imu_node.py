#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import serial
import threading
import math
import struct


def euler_to_quaternion(roll, pitch, yaw):
    """
    roll, pitch, yaw (rad) -> quaternion (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * cp * sy
    qx = sr * cp * cy - cr * cp * sy
    qy = cr * sp * cy + sr * sp * sy
    qz = cr * cp * sy - sr * cp * cy

    return qx, qy, qz, qw


class W901ImuNode(Node):
    """
    Node đọc IMU W901C-RS232 (WITMotion/HWT901C) và publish sensor_msgs/Imu trên topic /imu.

    Giả định IMU đang output format:
      - 0x55 0x51 ...: Acc (ax, ay, az, Temp)
      - 0x55 0x52 ...: Gyro (wx, wy, wz, Temp)
      - 0x55 0x53 ...: Angle (roll, pitch, yaw, Temp)
    """

    def __init__(self):
        super().__init__('w901_imu_node')

        # ===== Tham số ROS2 =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(f'Opening IMU serial {port} @ {baud}...')

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        # Publisher /imu
        self.pub = self.create_publisher(Imu, '/imu', 50)

        # Lưu giá trị mới nhất
        self.last_accel = (0.0, 0.0, 0.0)   # m/s^2
        self.last_gyro = (0.0, 0.0, 0.0)    # rad/s
        self.last_rpy = (0.0, 0.0, 0.0)     # rad

        # Thread đọc serial
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        self.get_logger().info('W901ImuNode started.')

    def read_loop(self):
        """
        Đọc stream frame 0x55 + type + 8 data + checksum.
        Khi nhận frame angle (0x53) thì publish 1 message Imu.
        """
        while rclpy.ok():
            try:
                head = self.ser.read(1)
                if not head:
                    continue
                if head[0] != 0x55:
                    continue

                # Đọc 10 byte còn lại: type(1) + data(8) + checksum(1)
                frame = self.ser.read(10)
                if len(frame) != 10:
                    continue

                frame_type = frame[0]          # 0x51, 0x52, 0x53
                data_bytes = frame[1:9]        # 8 byte data
                checksum = frame[9]

                # Checksum: (0x55 + sum(type + data[8])) & 0xFF phải bằng checksum
                calc_sum = (0x55 + sum(frame[0:9])) & 0xFF
                if calc_sum != checksum:
                    # self.get_logger().warn('Bad checksum frame')
                    continue

                if frame_type == 0x51:
                    self.parse_acc_frame(data_bytes)
                elif frame_type == 0x52:
                    self.parse_gyro_frame(data_bytes)
                elif frame_type == 0x53:
                    self.parse_angle_frame_and_publish(data_bytes)

            except Exception as e:
                self.get_logger().error(f'Error in read_loop: {e}')

    def parse_acc_frame(self, data_bytes):
        # 4 * int16: ax, ay, az, Temp
        ax_raw, ay_raw, az_raw, _temp = struct.unpack('<hhhh', data_bytes)

        # Theo WITMotion: range 16g -> acc (g) = raw / 32768 * 16
        g = 9.80665
        ax = ax_raw / 32768.0 * 16.0 * g
        ay = ay_raw / 32768.0 * 16.0 * g
        az = az_raw / 32768.0 * 16.0 * g

        self.last_accel = (ax, ay, az)

    def parse_gyro_frame(self, data_bytes):
        # 4 * int16: wx, wy, wz, Temp
        wx_raw, wy_raw, wz_raw, _temp = struct.unpack('<hhhh', data_bytes)

        # Range 2000 deg/s: deg/s = raw / 32768 * 2000
        wx_dps = wx_raw / 32768.0 * 2000.0
        wy_dps = wy_raw / 32768.0 * 2000.0
        wz_dps = wz_raw / 32768.0 * 2000.0

        deg2rad = math.pi / 180.0
        wx = wx_dps * deg2rad
        wy = wy_dps * deg2rad
        wz = wz_dps * deg2rad

        self.last_gyro = (wx, wy, wz)

    def parse_angle_frame_and_publish(self, data_bytes):
        # 4 * int16: roll, pitch, yaw, Temp
        roll_raw, pitch_raw, yaw_raw, _temp = struct.unpack('<hhhh', data_bytes)

        # Theo WITMotion: angle (deg) = raw / 32768 * 180
        roll_deg = roll_raw / 32768.0 * 180.0
        pitch_deg = pitch_raw / 32768.0 * 180.0
        yaw_deg = yaw_raw / 32768.0 * 180.0

        deg2rad = math.pi / 180.0
        roll = roll_deg * deg2rad
        pitch = pitch_deg * deg2rad
        yaw = yaw_deg * deg2rad

        self.last_rpy = (roll, pitch, yaw)

        # Khi có frame angle thì publish luôn 1 Imu (dùng accel + gyro gần nhất)
        self.publish_imu()

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation từ roll, pitch, yaw
        roll, pitch, yaw = self.last_rpy
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Angular velocity (rad/s)
        wx, wy, wz = self.last_gyro
        msg.angular_velocity.x = wx
        msg.angular_velocity.y = wy
        msg.angular_velocity.z = wz

        # Linear acceleration (m/s^2)
        ax, ay, az = self.last_accel
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # ===== Covariance cho robot_localization (có thể tune lại sau) =====
        # Orientation covariance (rad^2)
        # Giả sử sai số ~2 độ cho mỗi trục
        var_roll  = math.radians(2.0) ** 2
        var_pitch = math.radians(2.0) ** 2
        var_yaw   = math.radians(5.0) ** 2   # yaw thường kém chính xác hơn

        msg.orientation_covariance = [
            var_roll, 0.0,      0.0,
            0.0,      var_pitch,0.0,
            0.0,      0.0,      var_yaw
        ]

        # Angular velocity covariance (rad^2/s^2)
        # Giả sử sai số ~0.05 rad/s
        var_gyro = (0.05) ** 2
        msg.angular_velocity_covariance = [
            var_gyro, 0.0,     0.0,
            0.0,      var_gyro,0.0,
            0.0,      0.0,     var_gyro
        ]

        # Linear acceleration covariance (m^2/s^4)
        # Giả sử sai số ~0.1 m/s^2
        var_acc = (0.1) ** 2
        msg.linear_acceleration_covariance = [
            var_acc, 0.0,    0.0,
            0.0,     var_acc,0.0,
            0.0,     0.0,    var_acc
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
