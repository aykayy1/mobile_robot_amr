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

        # ===== ROS2 Parameters =====
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.636)
        self.declare_parameter('gear_ratio', 10.0)
        self.declare_parameter('rpm_max', 800)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('invert_right_wheel', False)
        self.declare_parameter('send_period', 0.2)

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
        self.ser = serial.Serial(
            port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )

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

        # ===== Subscriber =====
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            20
        )

        # ===== Internal State =====
        self._cmd_lock = threading.Lock()
        self._last_v = 0.0
        self._last_w = 0.0

        self._lock = threading.Lock()
        self._last_left_rpm = 0.0
        self._last_right_rpm = 0.0
        self._running = True

        # ===== Threads & Timers =====
        self.serial_thread = threading.Thread(
            target=self.serial_read_thread,
            daemon=True
        )
        self.serial_thread.start()

        self.timer = self.create_timer(0.02, self.timer_callback)

        if self.send_period > 0.0:
            self.send_timer = self.create_timer(
                self.send_period,
                self.send_timer_callback
            )
        else:
            self.send_timer = None

        self.get_logger().info("WheelVelNode READY")

    # =====================================================
    # STM32 -> Jetson
    # =====================================================
    def serial_read_thread(self):
        while self._running and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                self.pub_raw.publish(String(data=line))
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

    def rpm_to_ms(self, rpm):
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

        # ✅ TẤT CẢ COVARIANCE = 0
        msg.twist.covariance = [0.0] * 36

        self.pub.publish(msg)

    # =====================================================
    # cmd_vel -> STM32
    # =====================================================
    def ms_to_rpm(self, v, w):
        w_left = (v - w * self.wheel_separation / 2.0) / self.wheel_radius
        w_right = (v + w * self.wheel_separation / 2.0) / self.wheel_radius

        rpm_left = w_left * 60.0 / (2.0 * math.pi)
        rpm_right = w_right * 60.0 / (2.0 * math.pi)

        rpm_left *= self.gear_ratio
        rpm_right *= self.gear_ratio

        if self.invert_right:
            rpm_right = -rpm_right

        rpm_left = max(min(rpm_left, self.rpm_max), -self.rpm_max)
        rpm_right = max(min(rpm_right, self.rpm_max), -self.rpm_max)

        return rpm_left, rpm_right

    def cmd_vel_callback(self, msg):
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
            self.ser.write(f"{int(rpm_left)},{int(rpm_right)}\r\n".encode())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        time.sleep(0.1)
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
