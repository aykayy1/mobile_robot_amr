#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import math
import time

# ---- Helper: CRC16 (Modbus) ----
def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)


def build_read_request(slave: int, start_addr: int, count: int) -> bytes:
    pkt = bytes([
        slave,
        0x03,
        (start_addr >> 8) & 0xFF,
        start_addr & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF
    ])
    return pkt + modbus_crc(pkt)


def parse_modbus_response(resp: bytes):
    if len(resp) < 5:
        return None

    payload, crc = resp[:-2], resp[-2:]
    if modbus_crc(payload) != crc:
        return None

    if payload[1] != 0x03:
        return None

    byte_count = payload[2]
    data = payload[3:3 + byte_count]

    if len(data) != byte_count:
        return None

    return data


def regs_to_int16_list(data: bytes):
    vals = []
    for i in range(0, len(data), 2):
        hi = data[i]
        lo = data[i + 1]
        val = struct.unpack('>h', bytes([hi, lo]))[0]
        vals.append(val)
    return vals


class HWT901BModbusNode(Node):
    def __init__(self):
        super().__init__('hwt901b_modbus')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('slave_id', 0x51)  # theo datasheet
        self.declare_parameter('poll_hz', 50.0)
        self.declare_parameter('acc_scale_g', 16.0)
        self.declare_parameter('gyro_scale_dps', 2000.0)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.slave = int(self.get_parameter('slave_id').value)
        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.acc_scale_g = float(self.get_parameter('acc_scale_g').value)
        self.gyro_scale_dps = float(self.get_parameter('gyro_scale_dps').value)

        self.frame_id = 'imu_link'

        self.get_logger().info(
            f"HWT901B-485 Modbus node init: {port}@{baud} "
            f"slave=0x{self.slave:02X} poll={self.poll_hz:.1f} Hz"
        )

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.02,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
        except Exception as e:
            self.get_logger().error(f"Cannot open serial port {port}: {e}")
            raise

        # RS485 mode nếu adapter hỗ trợ
        try:
            from serial.rs485 import RS485Settings
            self.ser.rs485_mode = RS485Settings()
            self.rs485_available = True
            self.get_logger().info("Enabled rs485_mode on adapter")
        except Exception:
            self.rs485_available = False
            self.get_logger().warning(
                "rs485_mode not available, using manual RTS toggle if needed."
            )

        # --------- Covariance (use Witmotion default set you provided) ---------
        self.imu_linear_acceleration_covariance = [
            0.0364, 0.0,    0.0,
            0.0,    0.0048, 0.0,
            0.0,    0.0,    0.0796
        ]
        self.imu_angular_velocity_covariance = [
            0.0663, 0.0,    0.0,
            0.0,    0.1453, 0.0,
            0.0,    0.0,    0.0378
        ]
        self.imu_orientation_covariance = [
            0.0479, 0.0,    0.0,
            0.0,    0.0207, 0.0,
            0.0,    0.0,    0.0041
        ]
        # --------------------------------------------------------------------

        # Publisher
        self.pub = self.create_publisher(Imu, 'imu', 100)

        # Đọc 1 lần từ 0x34, 12 register
        self.addr_all = 0x0034
        self.count_all = 12
        self.req_all = build_read_request(self.slave, self.addr_all, self.count_all)

        # slave(1)+func(1)+byte_count(1)+2N+crc(2)
        self.expected_len = 5 + 2 * self.count_all

        period = 1.0 / max(1.0, self.poll_hz)
        self.timer = self.create_timer(period, self.poll_once)

    def send_request(self, req: bytes, expected_len: int) -> bytes:
        self.ser.reset_input_buffer()

        if not self.rs485_available:
            try:
                self.ser.setRTS(True)
            except Exception:
                pass

        try:
            self.ser.write(req)
            self.ser.flush()
        except Exception as e:
            self.get_logger().warning(f"Serial write error: {e}")
            return b''

        try:
            resp = self.ser.read(expected_len)
        except Exception as e:
            self.get_logger().warning(f"Serial read error: {e}")
            resp = b''

        if not self.rs485_available:
            try:
                self.ser.setRTS(False)
            except Exception:
                pass

        if resp:
            self.get_logger().debug(
                f"Req: {req.hex()} | Resp({len(resp)}B): {resp.hex()}"
            )
        else:
            self.get_logger().debug(f"Req: {req.hex()} | Resp: <empty>")

        return resp

    def poll_once(self):
        resp = self.send_request(self.req_all, self.expected_len)
        data = parse_modbus_response(resp) if resp else None
        if not data:
            return

        vals = regs_to_int16_list(data)
        if len(vals) < self.count_all:
            return

        ax_raw, ay_raw, az_raw = vals[0], vals[1], vals[2]
        gx_raw, gy_raw, gz_raw = vals[3], vals[4], vals[5]
        roll_raw, pitch_raw, yaw_raw = vals[9], vals[10], vals[11]

        g = 9.80665
        acc_scale = (self.acc_scale_g * g) / 32768.0
        ax = ax_raw * acc_scale
        ay = ay_raw * acc_scale
        az = az_raw * acc_scale

        gyro_scale = (self.gyro_scale_dps * math.pi / 180.0) / 32768.0
        gx = gx_raw * gyro_scale
        gy = gy_raw * gyro_scale
        gz = gz_raw * gyro_scale

        roll = roll_raw / 32768.0 * math.pi
        pitch = pitch_raw / 32768.0 * math.pi
        yaw = yaw_raw / 32768.0 * math.pi

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # --------- Set covariance exactly like your witmotion_node params ---------
        msg.linear_acceleration_covariance = self.imu_linear_acceleration_covariance
        msg.angular_velocity_covariance = self.imu_angular_velocity_covariance
        msg.orientation_covariance = self.imu_orientation_covariance
        # ----------------------------------------------------------------------------

        self.pub.publish(msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = HWT901BModbusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
