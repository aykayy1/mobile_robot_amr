#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import math
import time

import serial
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped


def getch():
    """Đọc 1 phím từ bàn phím (không cần Enter)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class WheelVelNode(Node):
    def __init__(self):
        super().__init__('wheel_vel_node')

        # ===== ROS2 PARAMETERS =====
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 38400)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.636)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('base_rpm', 20.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.base_rpm = self.get_parameter('base_rpm').value

        # ===== MỞ SERIAL =====
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

        # ===== SUBSCRIBER /cmd_vel =====
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # ===== PUBLISH /wheel_vel =====
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/wheel_vel',
            10
        )

        # ===== BIẾN LƯU RPM BÁNH =====
        self._lock = threading.Lock()
        self._last_left_rpm = 0.0
        self._last_right_rpm = 0.0
        self._running = True

        # ===== THREAD ĐỌC SERIAL STM32 =====
        self.serial_thread = threading.Thread(
            target=self.serial_read_thread,
            daemon=True
        )
        self.serial_thread.start()

        # ===== THREAD ĐIỀU KHIỂN BẰNG PHÍM =====
        if sys.stdin.isatty():
            self.keyboard_thread_handle = threading.Thread(
                target=self.keyboard_thread,
                daemon=True
            )
            self.keyboard_thread_handle.start()
        else:
            self.get_logger().warn("Keyboard control disabled (STDIN not TTY).")

        # ===== TIMER 50Hz PUBLISH VELOCITY =====
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # ======================================================
    #  SERIAL → đọc rpm từ STM32
    # ======================================================
    def serial_read_thread(self):
        while self._running and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if not line.startswith("VEL"):  # ví dụ: "VEL,12,-13"
                    continue

                parts = line.split(',')
                if len(parts) < 3:
                    continue

                left_rpm = float(parts[1])
                right_rpm = float(parts[2])

                with self._lock:
                    self._last_left_rpm = left_rpm
                    self._last_right_rpm = right_rpm

            except Exception:
                continue

    # ======================================================
    #  BÀN PHÍM → điều khiển thủ công
    # ======================================================
    def keyboard_thread(self):
        try:
            while self._running and rclpy.ok():
                key = getch()
                rpm_left = 0.0
                rpm_right = 0.0

                if key == "w":
                    rpm_left = +self.base_rpm
                    rpm_right = -self.base_rpm
                elif key == "s":
                    rpm_left = -self.base_rpm
                    rpm_right = -self.base_rpm
                elif key == "a":
                    rpm_left = -self.base_rpm
                    rpm_right = +self.base_rpm
                elif key == "d":
                    rpm_left = +self.base_rpm
                    rpm_right = -self.base_rpm
                elif key == "x":
                    rpm_left = 0.0
                    rpm_right = 0.0
                elif key == "q":
                    self._running = False
                    rclpy.shutdown()
                    break
                else:
                    continue

                try:
                    msg = f"{int(rpm_left)},{int(rpm_right)}\r\n"
                    self.ser.write(msg.encode("ascii"))
                except Exception:
                    pass

        except Exception:
            pass

    # ======================================================
    #  Hàm CONVERT: RPM → m/s
    # ======================================================
    def rpm_to_ms(self, rpm: float) -> float:
        wheel_rpm = rpm / self.gear_ratio if self.gear_ratio != 0 else rpm
        omega = wheel_rpm * (2.0 * math.pi / 60.0)
        return omega * self.wheel_radius

    # ======================================================
    #  CALLBACK TỪ /cmd_vel (Joystick)
    # ======================================================
    def cmd_vel_callback(self, msg: Twist):

        v = msg.linear.x          # m/s
        w = msg.angular.z         # rad/s

        # differential drive
        v_r = v + (self.wheel_separation * w) / 2.0
        v_l = v - (self.wheel_separation * w) / 2.0

        # m/s → RPM
        rpm_r = (v_r / (2.0 * math.pi * self.wheel_radius)) * 60.0
        rpm_l = (v_l / (2.0 * math.pi * self.wheel_radius)) * 60.0

        # gửi xuống STM32
        try:
            cmd = f"{int(rpm_l)},{int(rpm_r)}\r\n"
            self.ser.write(cmd.encode("ascii"))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    # ======================================================
    #  TIMER → publish /wheel_vel
    # ======================================================
    def timer_callback(self):
        with self._lock:
            left_rpm = self._last_left_rpm
            right_rpm = self._last_right_rpm

        v_left = self.rpm_to_ms(left_rpm)
        v_right = self.rpm_to_ms(right_rpm)

        v = 0.5 * (v_right + v_left)
        w = (v_right - v_left) / self.wheel_separation

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        self.pub.publish(msg)


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
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
