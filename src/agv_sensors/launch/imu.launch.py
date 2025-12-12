#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

HEADER = 0x55

PKT_ACC   = 0x51
PKT_GYRO  = 0x52
PKT_ANGLE = 0x53
PKT_QUAT  = 0x59


def euler_to_quat(roll, pitch, yaw):
    """Convert Euler (rad) -> geometry_msgs/Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def normalize_quat(q: Quaternion):
    n = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if n > 1e-9:
        q.w /= n
        q.x /= n
        q.y /= n
        q.z /= n
    return q


class WT901CImuNode(Node):
    def __init__(self):
        super().__init__('wt901c_imu_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('prefer_quat', True)   # ưu tiên 0x59 nếu có
        self.declare_parameter('yaw_sign', 1.0)       # +1 hoặc -1 để đảo chiều yaw

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.prefer_quat = bool(self.get_parameter('prefer_quat').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)

        # Serial init
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )
        self.get_logger().info(f'WT901C on {port} @ {baud} baud')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 50)
        self.yaw_pub = self.create_publisher(Float64, '/imu/yaw', 10)

        # State
        self.buf = bytearray()
        self.last_acc = None      # (ax, ay, az) m/s^2
        self.last_gyro = None     # (gx, gy, gz) rad/s
        self.last_quat = None     # Quaternion

        self.seen_quat = False    # đã từng nhận 0x59 chưa

        # Unwrap yaw state
        self.last_yaw = None
        self.yaw_unwrapped = 0.0

        # Timer loop
        self.timer = self.create_timer(0.005, self.read_loop)

    def read_loop(self):
        """Đọc dữ liệu serial định kỳ và parse."""
        try:
            data = self.ser.read(256)
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if data:
            self.buf.extend(data)
            self.parse_buffer()

    def parse_buffer(self):
        """Tìm frame 11 byte: 0x55, id, payload(8), crc."""
        while len(self.buf) >= 11:
            # đồng bộ header
            if self.buf[0] != HEADER:
                del self.buf[0]
                continue

            frame = self.buf[:11]
            crc = sum(frame[0:10]) & 0xFF
            if crc != frame[10]:
                # sai checksum, trượt 1 byte
                del self.buf[0]
                continue

            pkt_id = frame[1]
            payload = frame[2:10]
            self.handle_packet(pkt_id, payload)

            # pop frame
            del self.buf[:11]

    def handle_packet(self, pkt_id, payload):
        """Decode từng gói dữ liệu."""
        def to_i16(lo, hi):
            v = (hi << 8) | lo
            return v - 65536 if v >= 32768 else v

        if pkt_id == PKT_ACC:
            ax = to_i16(payload[0], payload[1])
            ay = to_i16(payload[2], payload[3])
            az = to_i16(payload[4], payload[5])

            # range ±16g -> m/s^2
            ax = ax / 32768.0 * 16.0 * 9.81
            ay = ay / 32768.0 * 16.0 * 9.81
            az = az / 32768.0 * 16.0 * 9.81
            self.last_acc = (ax, ay, az)

        elif pkt_id == PKT_GYRO:
            gx = to_i16(payload[0], payload[1])
            gy = to_i16(payload[2], payload[3])
            gz = to_i16(payload[4], payload[5])

            # range ±2000 deg/s -> rad/s
            gx = gx / 32768.0 * 2000.0
            gy = gy / 32768.0 * 2000.0
            gz = gz / 32768.0 * 2000.0
            d2r = math.pi / 180.0
            self.last_gyro = (gx * d2r, gy * d2r, gz * d2r)

        elif pkt_id == PKT_QUAT:
            # quaternion 0x59: q0,q1,q2,q3 scaled by 1/32768
            q0 = to_i16(payload[0], payload[1]) / 32768.0
            q1 = to_i16(payload[2], payload[3]) / 32768.0
            q2 = to_i16(payload[4], payload[5]) / 32768.0
            q3 = to_i16(payload[6], payload[7]) / 32768.0
            q = Quaternion()
            q.w, q.x, q.y, q.z = q0, q1, q2, q3
            self.last_quat = normalize_quat(q)
            self.seen_quat = True

        elif pkt_id == PKT_ANGLE:
            # chỉ dùng Euler nếu chưa thấy quat hoặc prefer_quat=False
            if self.prefer_quat and self.seen_quat:
                return

            roll  = to_i16(payload[0], payload[1]) / 32768.0 * 180.0
            pitch = to_i16(payload[2], payload[3]) / 32768.0 * 180.0
            yaw   = to_i16(payload[4], payload[5]) / 32768.0 * 180.0

            r = roll * math.pi / 180.0
            p = pitch * math.pi / 180.0
            y = yaw * math.pi / 180.0 * self.yaw_sign

            q = euler_to_quat(r, p, y)
            self.last_quat = normalize_quat(q)

        # Khi đã có đủ acc + gyro + quat thì publish
        if (
            self.last_acc is not None
            and self.last_gyro is not None
            and self.last_quat is not None
        ):
            self.publish_imu()

    def publish_imu(self):
        """Publish sensor_msgs/Imu và /imu/yaw (Float64, rad)."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        q = self.last_quat
        msg.orientation = q

        ax, ay, az = self.last_acc
        gx, gy, gz = self.last_gyro

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.04
        ]
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.02
        ]
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.2
        ]

        self.imu_pub.publish(msg)

        # ---- TÍNH YAW TỪ QUAT & UNWRAP ----
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # [-pi, pi]

        if self.last_yaw is None:
            self.yaw_unwrapped = yaw
        else:
            dy = yaw - self.last_yaw
            if dy > math.pi:
                dy -= 2.0 * math.pi
            elif dy < -math.pi:
                dy += 2.0 * math.pi
            self.yaw_unwrapped += dy
        self.last_yaw = yaw

        yaw_msg = Float64()
        yaw_msg.data = self.yaw_unwrapped  # rad (unwrap, có thể >2π nếu quay nhiều vòng)
        self.yaw_pub.publish(yaw_msg)
        # -----------------------------------

def main():
    rclpy.init()
    node = WT901CImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
