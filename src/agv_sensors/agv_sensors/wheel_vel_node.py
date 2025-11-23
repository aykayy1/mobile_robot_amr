#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped

import serial
import threading
import math


class WheelVelNode(Node):
    def __init__(self):
        super().__init__('wheel_vel_node')

        # ===== Tham số ROS2 (có thể set bằng ros2 param set) =====
        self.declare_parameter('port', '/dev/ttyACM0')     # Port UART từ STM32
        self.declare_parameter('baud', 38400)              # Baudrate phải trùng STM32
        self.declare_parameter('wheel_radius', 0.1)        # [m]  - bán kính bánh xe
        self.declare_parameter('wheel_separation', 0.636)  # [m]  - khoảng cách 2 bánh
        self.declare_parameter('gear_ratio', 1.0)          # nếu rpm là rpm BÁNH thì để 1.0

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value

        self.get_logger().info(f"Opening serial port {port} @ {baud} ...")

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        except Exception as e:
            self.get_logger().error(f"Cannot open serial: {e}")
            raise

        # Publisher cho robot_localization (twist input)
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped,
            '/wheel_vel',
            10
        )

        # Dữ liệu share giữa thread đọc serial và ROS timer
        self._lock = threading.Lock()
        self._last_left_rpm = 0.0
        self._last_right_rpm = 0.0

        # Thread đọc serial liên tục
        self._running = True
        self._thread = threading.Thread(target=self.serial_reader_thread, daemon=True)
        self._thread.start()

        # Timer ROS: định kỳ 50Hz publish Twist
        self.timer_period = 0.02  # 20ms ~ 50 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("wheel_vel_node started.")

    # ===== Thread đọc UART =====
    def serial_reader_thread(self):
        """
        Đọc từng dòng từ UART, parse VEL,<left_rpm>,<right_rpm>
        Ví dụ: VEL,120,-130
        """
        while self._running and self.rclpy_ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                # Kiểm tra prefix
                if not line.startswith("VEL"):
                    continue

                # Tách chuỗi: VEL,<left>,<right>
                parts = line.split(',')
                if len(parts) < 3:
                    self.get_logger().warn(f"Bad line: {line}")
                    continue

                left_rpm = float(parts[1])
                right_rpm = float(parts[2])

                with self._lock:
                    self._last_left_rpm = left_rpm
                    self._last_right_rpm = right_rpm

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def rclpy_ok(self):
        # tiện cho thread để dừng khi node shutdown
        return rclpy.ok()

    # ===== Hàm chuyển RPM -> m/s (bánh) =====
    def rpm_to_ms(self, rpm: float) -> float:
        """
        rpm: vòng/phút của trục BÁNH
        wheel_radius: [m]
        gear_ratio: nếu rpm là rpm motor, còn wheel là rpm_motor / gear_ratio
                    nếu rpm đã là rpm bánh thì gear_ratio = 1.0
        """
        # Nếu rpm là rpm motor, đổi sang rpm bánh:
        if self.gear_ratio != 0.0:
            wheel_rpm = rpm / self.gear_ratio
        else:
            wheel_rpm = rpm

        # rpm -> rad/s: omega = rpm * 2π / 60
        omega = wheel_rpm * (2.0 * math.pi / 60.0)

        # v = omega * R
        v = omega * self.wheel_radius
        return v

    # ===== Timer callback: publish TwistWithCovarianceStamped =====
    def timer_callback(self):
        with self._lock:
            left_rpm = self._last_left_rpm
            right_rpm = self._last_right_rpm

        # Đổi rpm -> m/s cho từng bánh
        v_left = self.rpm_to_ms(left_rpm)
        v_right = self.rpm_to_ms(right_rpm)

        # Kinematic mô hình differential drive:
        # v = (v_r + v_l)/2
        # w = (v_r - v_l)/wheel_sep
        v = (v_right + v_left) * 0.5
        w = (v_right - v_left) / self.wheel_sep if self.wheel_sep != 0.0 else 0.0

        # Đóng gói vào TwistWithCovarianceStamped
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'Base_footprint'   # hoặc base_footprint tùy bạn cấu hình RL

        msg.twist.twist.linear.x = v     # m/s
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = w    # rad/s

        # Covariance đơn giản (có thể chỉnh lại cho hợp lý):
        # ma trận 6x6 (row-major)
        cov = [0.0] * 36
        cov[0 * 6 + 0] = 0.01   # var(vx)
        cov[1 * 6 + 1] = 0.01   # var(vy) (mình cho nhỏ nhưng RL có thể bỏ qua vy)
        cov[5 * 6 + 5] = 0.02   # var(wz)
        msg.twist.covariance = cov

        self.pub_twist.publish(msg)
        # self.get_logger().info(f"v={v:.3f} m/s, w={w:.3f} rad/s")

    def destroy_node(self):
        self._running = False
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
