#!/usr/bin/env python3
import threading
import math
import time

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String


class WheelVelNode(Node):
    def __init__(self):
        super().__init__('wheel_vel_node')

        # ===== Tham số ROS2 =====
        self.declare_parameter('port', '/dev/ttyACM0')      # Port UART từ STM32
        self.declare_parameter('baud', 38400)              # Baudrate

        self.declare_parameter('wheel_radius', 0.1)        # [m]
        self.declare_parameter('wheel_separation', 0.636)  # [m]
        self.declare_parameter('gear_ratio', 10.0)         # nếu rpm STM32 là rpm motor
        self.declare_parameter('rpm_max', 200.0)           # giới hạn rpm gửi xuống

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ✅ set default luôn trong code
        self.declare_parameter('invert_right_wheel', True)  # mặc định đảo bánh phải
        self.declare_parameter('send_period', 0.2)          # mặc định 200ms

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.rpm_max = float(self.get_parameter('rpm_max').value)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.invert_right = bool(self.get_parameter('invert_right_wheel').value)
        self.send_period = float(self.get_parameter('send_period').value)

        # ===== Serial =====
        self.get_logger().info(f"Opening serial {port} @ {baud}...")
        try:
            self.ser = serial.Serial(
                port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
        except Exception as e:
            self.get_logger().error(f"Cannot open serial: {e}")
            raise

        # ===== Publishers =====
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/wheel_vel',
            10
        )

        self.pub_raw = self.create_publisher(
            String,
            '/wheel_vel_raw',
            10
        )

        # ===== Sub cmd_vel từ Nav2 =====
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            20
        )

        # Cache cmd_vel mới nhất
        self._cmd_lock = threading.Lock()
        self._last_v = 0.0
        self._last_w = 0.0
        self._last_cmd_time = self.get_clock().now()

        # ===== Biến lưu rpm đọc được từ STM32 =====
        self._lock = threading.Lock()
        self._last_left_rpm = 0.0
        self._last_right_rpm = 0.0
        self._running = True

        # ===== Thread đọc serial STM32 -> Jetson =====
        self.serial_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
        self.serial_thread.start()

        # ===== Timer publish odom từ rpm STM32 =====
        self.timer_period = 0.02  # 50Hz để odom mượt
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # ===== Timer gửi rpm xuống STM32 (throttle) =====
        if self.send_period > 0.0:
            self.send_timer = self.create_timer(self.send_period, self.send_timer_callback)
        else:
            self.send_timer = None  # gửi trực tiếp trong callback cmd_vel

        self.get_logger().info(
            f"WheelVelNode ready. cmd_vel={self.cmd_vel_topic}, "
            f"send_period={self.send_period}s, invert_right={self.invert_right}"
        )

    # =========================================================
    # 1) NHẬN DỮ LIỆU TỪ STM32 -> publish /wheel_vel_raw, /wheel_vel
    # =========================================================
    def serial_read_thread(self):
        while self._running and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                raw_msg = String()
                raw_msg.data = line
                self.pub_raw.publish(raw_msg)

                parts = line.split(',')

                if len(parts) == 3 and parts[0].upper() == "VEL":
                    left_rpm = float(parts[1])
                    right_rpm = float(parts[2])
                elif len(parts) == 2:
                    left_rpm = float(parts[0])
                    right_rpm = float(parts[1])
                else:
                    continue

                with self._lock:
                    self._last_left_rpm = left_rpm
                    self._last_right_rpm = right_rpm

            except Exception:
                continue

    def rpm_to_ms(self, rpm: float) -> float:
        wheel_rpm = rpm / self.gear_ratio if self.gear_ratio != 0.0 else rpm
        omega = wheel_rpm * (2.0 * math.pi / 60.0)
        return omega * self.wheel_radius

    def timer_callback(self):
        with self._lock:
            left_rpm = self._last_left_rpm
            right_rpm = self._last_right_rpm

        v_left = self.rpm_to_ms(left_rpm)
        v_right = self.rpm_to_ms(right_rpm)

        v = 0.5 * (v_right + v_left)
        w = (v_right - v_left) / self.wheel_separation if self.wheel_separation != 0.0 else 0.0

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "Base_footprint"

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        for i in range(36):
            msg.twist.covariance[i] = 0.0
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.01

        self.pub.publish(msg)

    # =========================================================
    # 2) NHẬN /cmd_vel -> đổi sang rpm -> gửi xuống STM32
    # =========================================================
    def ms_to_rpm(self, v, w):
        w_left  = (v - w * self.wheel_separation / 2.0) / self.wheel_radius
        w_right = (v + w * self.wheel_separation / 2.0) / self.wheel_radius

        rpm_left  = w_left  * 60.0 / (2.0 * math.pi)
        rpm_right = w_right * 60.0 / (2.0 * math.pi)

        rpm_left  *= self.gear_ratio
        rpm_right *= self.gear_ratio

        if self.invert_right:
            rpm_right = -rpm_right

        rpm_left  = max(min(rpm_left,  self.rpm_max), -self.rpm_max)
        rpm_right = max(min(rpm_right, self.rpm_max), -self.rpm_max)

        return rpm_left, rpm_right

    def cmd_vel_callback(self, msg: Twist):
        with self._cmd_lock:
            self._last_v = msg.linear.x
            self._last_w = msg.angular.z

        if self.send_period <= 0.0:
            self.send_latest_cmd_to_stm32()

    def send_timer_callback(self):
        self.send_latest_cmd_to_stm32()

    def send_latest_cmd_to_stm32(self):
        with self._cmd_lock:
            v = self._last_v
            w = self._last_w

        rpm_left, rpm_right = self.ms_to_rpm(v, w)

        try:
            out = f"{int(rpm_left)},{int(rpm_right)}\r\n"
            self.ser.write(out.encode("ascii"))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelVelNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._running = False
            time.sleep(0.1)
            try:
                node.ser.close()
            except Exception:
                pass
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
