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
        self.declare_parameter('yaw_correction_gain', 0.005)  # alpha trong complementary filter

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.prefer_quat = bool(self.get_parameter('prefer_quat').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.yaw_alpha = float(self.get_parameter('yaw_correction_gain').value)

        # Serial
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
        self.last_quat = None     # Quaternion (raw từ IMU, dùng cho roll/pitch + yaw_abs)

        self.seen_quat = False    # đã từng nhận frame 0x59 chưa

        # Gyro-integrated yaw (relative), với correction từ yaw_abs
        self.yaw_est = 0.0        # yaw_gyro (rad), có thể > 2π
        self.last_time = self.get_clock().now()
        self.have_time = False

        # Gyro bias calibration (đơn giản): lấy trung bình gz lúc đứng yên ban đầu
        self.gyro_bias = 0.0
        self.bias_sum = 0.0
        self.bias_count = 0
        self.bias_calib_done = False
        self.bias_max_samples = 500      # ~2.5s nếu 200Hz
        self.bias_gz_thresh = 0.05       # rad/s, dưới ngưỡng coi như đứng yên

        self.timer = self.create_timer(0.005, self.read_loop)

    def read_loop(self):
        """Đọc serial và parse."""
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
            del self.buf[:11]

    def handle_packet(self, pkt_id, payload):
        """Decode từng packet."""
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

        # Khi đủ acc + gyro + quat thì publish
        if (
            self.last_acc is not None
            and self.last_gyro is not None
            and self.last_quat is not None
        ):
            self.publish_imu()

    def publish_imu(self):
        """Publish /imu và /imu/yaw với yaw_est (gyro + correction)."""
        now = self.get_clock().now()
        if not self.have_time:
            self.last_time = now
            self.have_time = True
            dt = 0.0
        else:
            dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
            if dt < 0.0:
                dt = 0.0
            self.last_time = now

        ax, ay, az = self.last_acc
        gx, gy, gz = self.last_gyro
        gz_yaw = gz * self.yaw_sign  # rad/s

        # --- gyro bias calibration đơn giản lúc đầu ---
        if not self.bias_calib_done and dt > 0.0:
            if abs(gz_yaw) < self.bias_gz_thresh:
                self.bias_sum += gz_yaw
                self.bias_count += 1
                if self.bias_count >= self.bias_max_samples:
                    self.gyro_bias = self.bias_sum / self.bias_count
                    self.bias_calib_done = True
                    self.get_logger().info(
                        f"Gyro bias calibrated: {self.gyro_bias:.6f} rad/s"
                    )

        gz_corr = gz_yaw - (self.gyro_bias if self.bias_calib_done else 0.0)

        # --- tính yaw tuyệt đối từ quaternion IMU (để correction) ---
        q_raw = self.last_quat
        # roll, pitch, yaw_raw từ quaternion (REP-103)
        sinr_cosp = 2.0 * (q_raw.w * q_raw.x + q_raw.y * q_raw.z)
        cosr_cosp = 1.0 - 2.0 * (q_raw.x * q_raw.x + q_raw.y * q_raw.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q_raw.w * q_raw.y - q_raw.z * q_raw.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q_raw.w * q_raw.z + q_raw.x * q_raw.y)
        cosy_cosp = 1.0 - 2.0 * (q_raw.y * q_raw.y + q_raw.z * q_raw.z)
        yaw_abs = math.atan2(siny_cosp, cosy_cosp) * self.yaw_sign  # [-pi, pi]

        # --- tích phân yaw từ gyro + bổ sung correction từ yaw_abs ---
        yaw_pred = self.yaw_est + gz_corr * dt  # dự đoán theo gyro

        # unwrap yaw_abs quanh yaw_pred (để không nhảy ±pi)
        yaw_corr = yaw_abs
        dy = yaw_corr - yaw_pred
        if dy > math.pi:
            yaw_corr -= 2.0 * math.pi
        elif dy < -math.pi:
            yaw_corr += 2.0 * math.pi

        alpha = self.yaw_alpha
        self.yaw_est = (1.0 - alpha) * yaw_pred + alpha * yaw_corr

        # --- build quaternion mới với roll, pitch từ IMU + yaw_est ---
        q_fused = euler_to_quat(roll, pitch, self.yaw_est)

        # Fill Imu message
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation = q_fused

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

        # Publish yaw_est ra /imu/yaw
        yaw_msg = Float64()
        yaw_msg.data = self.yaw_est    # rad, liên tục, có thể > 2*pi
        self.yaw_pub.publish(yaw_msg)

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WT901CImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
