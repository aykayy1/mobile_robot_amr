#!/usr/bin/env python3
# hwt901b_modbus_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import time
import math

# ---- CRC16 (Modbus RTU) ----
def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)  # little-endian (lo, hi)

# ---- Build/parse Modbus ----
def build_read_request(slave: int, start_addr: int, count: int) -> bytes:
    pkt = bytes([
        slave & 0xFF, 0x03,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (count >> 8) & 0xFF, count & 0xFF
    ])
    return pkt + modbus_crc(pkt)

# resp: [slave][0x03][byte_count][data...][crc_lo][crc_hi]
def parse_modbus_response(resp: bytes):
    if len(resp) < 5:
        return None
    payload, crc_recv = resp[:-2], resp[-2:]
    if modbus_crc(payload) != crc_recv:
        return None
    if payload[1] != 0x03:
        return None
    bc = payload[2]
    data = payload[3:3+bc]
    return data

def regs_to_int16_list(data: bytes):
    vals = []
    for i in range(0, len(data), 2):
        if i + 1 >= len(data):
            break
        vals.append(struct.unpack('>h', data[i:i+2])[0])  # Modbus big-endian
    return vals

def parse_parity(p: str):
    p = (p or 'N').upper()
    if p == 'E': return serial.PARITY_EVEN
    if p == 'O': return serial.PARITY_ODD
    return serial.PARITY_NONE  # default: 8N1

def parse_stopbits(sb: float):
    return serial.STOPBITS_TWO if int(sb) == 2 else serial.STOPBITS_ONE

def parse_bytesize(bs: int):
    return {
        5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS
    }.get(int(bs), serial.EIGHTBITS)

class HWT901BModbusNode(Node):
    def __init__(self):
        super().__init__('hwt901b_modbus')

        # -------- ROS params --------
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)         # theo setup của bạn
        self.declare_parameter('parity', 'N')          # 'N','E','O' (mặc định 8N1)
        self.declare_parameter('stopbits', 1)          # 1 hoặc 2
        self.declare_parameter('bytesize', 8)          # 8 là phổ biến
        self.declare_parameter('slave_id', 0x51)       # yêu cầu của bạn
        self.declare_parameter('poll_hz', 50.0)
        self.declare_parameter('acc_scale_g', 16.0)    # ±16g
        self.declare_parameter('gyro_scale_dps', 2000.0) # ±2000 dps
        self.declare_parameter('frame_id', 'imu')
        self.declare_parameter('debug_raw', False)
        # Cho adapter cần đảo cực RTS (DE active-LOW)
        self.declare_parameter('rts_tx_active_high', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        parity = parse_parity(self.get_parameter('parity').get_parameter_value().string_value)
        stopbits = parse_stopbits(self.get_parameter('stopbits').get_parameter_value().integer_value)
        bytesize = parse_bytesize(self.get_parameter('bytesize').get_parameter_value().integer_value)
        self.slave = self.get_parameter('slave_id').get_parameter_value().integer_value
        self.poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.acc_scale_g = float(self.get_parameter('acc_scale_g').get_parameter_value().double_value)
        self.gyro_scale_dps = float(self.get_parameter('gyro_scale_dps').get_parameter_value().double_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.debug_raw = self.get_parameter('debug_raw').get_parameter_value().bool_value
        self.rts_tx_active_high = self.get_parameter('rts_tx_active_high').get_parameter_value().bool_value

        self.get_logger().info(
            f"HWT901B-485 Modbus node init: {port}@{baud} "
            f"slave=0x{self.slave:02X} poll={self.poll_hz}Hz"
        )

        # -------- Serial open --------
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baud, timeout=0.2,
                bytesize=bytesize, parity=parity, stopbits=stopbits
            )
        except Exception as e:
            self.get_logger().error(f"Cannot open serial port {port}: {e}")
            raise

        # -------- RS485 auto-toggle nếu có --------
        self.rs485_available = False
        try:
            from serial.rs485 import RS485Settings
            self.ser.rs485_mode = RS485Settings(
                rts_level_for_tx=self.rts_tx_active_high,
                rts_level_for_rx=not self.rts_tx_active_high,
                delay_before_tx=0.0,
                delay_before_rx=0.001,
            )
            self.get_logger().info("Enabled serial.rs485_mode() on adapter")
            self.rs485_available = True
        except Exception:
            self.get_logger().warning("rs485_mode not set. Will toggle RTS manually.")

        # -------- ROS publisher --------
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # -------- Register map --------
        self.addr_acc   = 0x0034  # ax ay az
        self.addr_gyro  = 0x0037  # gx gy gz
        self.addr_angle = 0x003D  # roll pitch yaw

        self.req_acc   = build_read_request(self.slave, self.addr_acc, 3)
        self.req_gyro  = build_read_request(self.slave, self.addr_gyro, 3)
        self.req_angle = build_read_request(self.slave, self.addr_angle, 3)

        # -------- Timer --------
        period = 1.0 / max(1.0, self.poll_hz)
        self.timer = self.create_timer(period, self.poll_once)

        # -------- State --------
        self.acc = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.orientation_q = (0.0, 0.0, 0.0, 1.0)

    # ---------- TX/RX với RTS đúng thứ tự ----------
    def _set_rts(self, tx: bool):
        if self.rs485_available:
            return
        try:
            # Nếu adapter DE active-LOW thì tx=True => RTS=False
            level = self.rts_tx_active_high if tx else (not self.rts_tx_active_high)
            self.ser.setRTS(level)
        except Exception:
            pass

    def send_request(self, req: bytes) -> bytes:
        # 1) TX
        self._set_rts(tx=True)
        time.sleep(0.001)

        # 2) Gửi
        try:
            self.ser.write(req)
            self.ser.flush()
        except Exception as e:
            self.get_logger().warning(f"Serial write exception: {e}")

        # 3) Chuyển sang RX NGAY sau khi gửi
        time.sleep(0.001)
        self._set_rts(tx=False)
        time.sleep(0.002)  # thời gian chuyển hướng

        # 4) Đọc theo deadline, dừng sớm nếu đủ khung
        resp = bytearray()
        deadline = time.monotonic() + 0.1
        try:
            while time.monotonic() < deadline:
                chunk = self.ser.read(256)
                if chunk:
                    resp.extend(chunk)
                    if self.debug_raw:
                        self.get_logger().debug(f"RAW ({len(resp)}B): {resp.hex()}")
                    if len(resp) >= 5 and len(resp) >= (resp[2] + 5):
                        break
                else:
                    time.sleep(0.002)
        except Exception as e:
            self.get_logger().warning(f"Serial read exception: {e}")

        return bytes(resp)

    def poll_once(self):
        # ---- ACC ----
        resp = self.send_request(self.req_acc)
        data = parse_modbus_response(resp) if resp else None
        if data:
            vals = regs_to_int16_list(data)
            if len(vals) >= 3:
                s = self.acc_scale_g * 9.80665 / 32768.0
                self.acc = (vals[0]*s, vals[1]*s, vals[2]*s)

        # ---- GYRO ----
        resp = self.send_request(self.req_gyro)
        data = parse_modbus_response(resp) if resp else None
        if data:
            vals = regs_to_int16_list(data)
            if len(vals) >= 3:
                s = (self.gyro_scale_dps * math.pi / 180.0) / 32768.0
                self.gyro = (vals[0]*s, vals[1]*s, vals[2]*s)

        # ---- ANGLE -> quaternion ----
        resp = self.send_request(self.req_angle)
        data = parse_modbus_response(resp) if resp else None
        if data:
            vals = regs_to_int16_list(data)
            if len(vals) >= 3:
                roll  = vals[0] / 32768.0 * math.pi
                pitch = vals[1] / 32768.0 * math.pi
                yaw   = vals[2] / 32768.0 * math.pi
                self.orientation_q = self.euler_to_quaternion(roll, pitch, yaw)

        # ---- Publish ----
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = self.acc
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = self.gyro
        qx, qy, qz, qw = self.orientation_q
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qx, qy, qz, qw
        self.pub.publish(msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw*0.5); sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
        return (
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy,
        )

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
