#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String

import serial
import math
import threading
import time


class WheelVelFromSTM32Node(Node):
    """
    Node đọc tốc độ bánh từ STM32 qua UART và publish:
      - /wheel_vel_raw : String (dữ liệu thô STM32 gửi, ví dụ "12,-13")
      - /wheel_vel     : TwistWithCovarianceStamped (v, w tính từ rpm)

    Format UART STM32 gửi:
      - "L,R\\r\\n"    (ví dụ "12,-13\\r\\n")
      - hoặc "VEL,L,R\\r\\n" (ví dụ "VEL,12,-13\\r\\n")
    """

    def __init__(self):
        super().__init__('wheel_vel_from_stm32')

        # ===== Tham số ROS2 =====
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 38400)
        self.declare_parameter('wheel_radius', 0.10)        # m
        self.declare_parameter('wheel_separation', 0.636)   # m
        self.declare_parameter('gear_ratio', 10.0)           # nếu rpm là rpm bánh -> 1.0

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value

        self.get_logger().info(
            f"Opening UART (feedback) {port} @ {baud} "
            f"(R={self.wheel_radius} m, L={self.wheel_separation} m, gear={self.gear_ratio})"
        )

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
            self.get_logger().error(f"Cannot open serial port {port}: {e}")
            raise

        # ===== Publishers =====
        # Dữ liệu thô: "L,R" hoặc "VEL,L,R"
        self.pub_raw = self.create_publisher(
            String,
            '/wheel_vel_raw',
            10
        )

        # Vận tốc robot (v,w) dạng TwistWithCovarianceStamped
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped,
            '/wheel_vel',
            10
        )

        # ===== Biến lưu rpm cuối cùng =====
        self._lock = threading.Lock()
        self._last_left_rpm = 0.0
        self._last_right_rpm = 0.0
        self._running = True

        # Thread đọc UART
        self.serial_thread = threading.Thread(
            target=self.serial_read_thread,
            daemon=True
        )
        self.serial_thread.start()

        # Timer publish Twist (50 Hz)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("WheelVelFromSTM32Node started.")

    # -------------------- UART RX thread --------------------
    def serial_read_thread(self):
        while self._running and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                # Publish thô
                raw_msg = String()
                raw_msg.data = line
                self.pub_raw.publish(raw_msg)

                # Parse rpm
                parts = line.split(',')
                left_rpm = None
                right_rpm = None

                # Hỗ trợ 2 kiểu: "VEL,L,R" hoặc "L,R"
                if len(parts) == 3 and parts[0].upper() == "VEL":
                    left_rpm = float(parts[1])
                    right_rpm = float(parts[2])
                elif len(parts) == 2:
                    left_rpm = float(parts[0])
                    right_rpm = float(parts[1])
                else:
                    # không đúng format
                    continue

                with self._lock:
                    self._last_left_rpm = left_rpm
                    self._last_right_rpm = right_rpm

            except Exception:
                # không để crash vì lỗi lặt vặt
                continue

    # -------------------- helper rpm -> m/s --------------------
    def rpm_to_ms(self, rpm: float) -> float:
        if self.gear_ratio != 0.0:
            wheel_rpm = rpm / self.gear_ratio
        else:
            wheel_rpm = rpm

        omega = wheel_rpm * (2.0 * math.pi / 60.0)  # rad/s
        v = omega * self.wheel_radius               # m/s
        return v

    # -------------------- Timer: publish /wheel_vel ---------
    def timer_callback(self):
        with self._lock:
            left_rpm = self._last_left_rpm
            right_rpm = self._last_right_rpm

        v_left = self.rpm_to_ms(left_rpm)
        v_right = self.rpm_to_ms(right_rpm)

        v = 0.5 * (v_right + v_left)
        if self.wheel_separation != 0.0:
            w = (v_right - v_left) / self.wheel_separation
        else:
            w = 0.0

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "Base_footprint"

        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = w

        # covariance đơn giản
        for i in range(36):
            msg.twist.covariance[i] = 0.0
        msg.twist.covariance[0] = 0.01   # var(v_x)
        msg.twist.covariance[35] = 0.01  # var(w_z)

        self.pub_twist.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelVelFromSTM32Node()
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


if __name__ == '__main__':
    main()
