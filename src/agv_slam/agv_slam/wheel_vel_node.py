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
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String   # <-- thêm để publish /wheel_vel_raw


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

        # ===== Tham số ROS2 (có thể set bằng ros2 param set) =====
        self.declare_parameter('port', '/dev/ttyACM0')      # Port UART từ STM32
        self.declare_parameter('baud', 115200)               # Baudrate
        self.declare_parameter('wheel_radius', 0.1)         # [m]
        self.declare_parameter('wheel_separation', 0.636)   # [m]
        self.declare_parameter('gear_ratio', 10.0)           # nếu rpm là rpm bánh thì = 1.0
        self.declare_parameter('base_rpm', 350.0)
        self.declare_parameter('base_rpm_rota', 40.0)               # rpm cơ bản khi nhấn phím

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.base_rpm = self.get_parameter('base_rpm').get_parameter_value().double_value
        self.base_rpm_rota = self.get_parameter('base_rpm_rota').get_parameter_value().double_value

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

        # Publisher /wheel_vel (TwistWithCovarianceStamped)
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/wheel_vel',
            10
        )

        # Publisher /wheel_vel_raw (dữ liệu thô từ STM32)
        self.pub_raw = self.create_publisher(
            String,
            '/wheel_vel_raw',
            10
        )

        # Biến lưu rpm cuối cùng đọc được từ STM32
        self._lock = threading.Lock()
        self._last_left_rpm = 0.0
        self._last_right_rpm = 0.0
        self._running = True

        # Thread đọc serial từ STM32
        self.serial_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
        self.serial_thread.start()

        # Thread đọc bàn phím (chỉ hoạt động khi chạy trực tiếp trong terminal)
        if sys.stdin.isatty():
            self.keyboard_thread_handle = threading.Thread(target=self.keyboard_thread, daemon=True)
            self.keyboard_thread_handle.start()
        else:
            self.get_logger().warn("STDIN is not a TTY, keyboard control disabled.")

        # Timer ROS: định kỳ 50Hz publish Twist
        self.timer_period = 0.02  # 20ms ~ 50 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # ===== Thread đọc dữ liệu từ STM32 =====
    def serial_read_thread(self):
        """
        STM32 (code mới) gửi lên dạng:
          <rpm_left>,<rpm_right>\\r\\n
        Ví dụ:
          12,-13\\r\\n

        (Nếu sau này bạn vẫn còn bản STM32 cũ gửi "VEL,L,R", code bên dưới
         cũng cố gắng hỗ trợ cả 2 kiểu.)
        """
        while self._running and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                # Publish thô lên /wheel_vel_raw đúng như STM32 gửi
                raw_msg = String()
                raw_msg.data = line          # ví dụ "1,0" hoặc "12,-13"
                self.pub_raw.publish(raw_msg)

                # --------- Parse rpm từ chuỗi ---------
                parts = line.split(',')

                left_rpm = None
                right_rpm = None

                # Hỗ trợ 2 kiểu:
                # 1) "VEL,L,R"
                # 2) "L,R"
                if len(parts) == 3 and parts[0].upper() == "VEL":
                    # Kiểu cũ: "VEL,12,-13"
                    left_rpm = float(parts[1])
                    right_rpm = float(parts[2])
                elif len(parts) == 2:
                    # Kiểu mới: "12,-13"
                    left_rpm = float(parts[0])
                    right_rpm = float(parts[1])
                else:
                    # Không đúng format thì bỏ qua
                    continue

                with self._lock:
                    self._last_left_rpm = left_rpm
                    self._last_right_rpm = right_rpm

            except Exception:
                # Bỏ qua lỗi, tiếp tục vòng lặp
                continue

    # ===== Thread đọc bàn phím & gửi lệnh rpm xuống STM32 =====
    def keyboard_thread(self):
        """
        Điều khiển bằng phím:
          w: tiến
          s: lùi
          a: quay trái tại chỗ
          d: quay phải tại chỗ
          x: dừng
          q: thoát node
        Không in debug ra màn hình, chỉ gửi lệnh xuống STM32.
        """
        try:
            while self._running and rclpy.ok():
                key = getch()

                rpm_left = 0.0
                rpm_right = 0.0

                if key == "w":
                    # đi thẳng tới
                    rpm_left = +self.base_rpm
                    rpm_right = +self.base_rpm
                elif key == "s":
                    # đi lùi
                    rpm_left = -self.base_rpm
                    rpm_right = -self.base_rpm
                elif key == "a":
                    # quay trái tại chỗ
                    rpm_left = -self.base_rpm_rota
                    rpm_right = +self.base_rpm_rota
                elif key == "d":
                    # quay phải tại chỗ
                    rpm_left = +self.base_rpm_rota
                    rpm_right = -self.base_rpm_rota
                elif key == "x":
                    # dừng
                    rpm_left = 0.0
                    rpm_right = 0.0
                elif key == "q":
                    # thoát node
                    self._running = False
                    rclpy.shutdown()
                    break
                else:
                    # phím khác thì bỏ qua
                    continue

                # Gửi lệnh xuống STM32 dạng "L,R\r\n"
                try:
                    msg = f"{int(rpm_left)},{int(rpm_right)}\r\n"
                    self.ser.write(msg.encode("ascii"))
                except Exception:
                    # Không để crash vì lỗi serial
                    pass

        except Exception:
            # tránh crash vì getch lỗi
            pass

    # ===== Hàm chuyển RPM -> m/s =====
    def rpm_to_ms(self, rpm: float) -> float:
        """
        rpm: vòng/phút của trục BÁNH (hoặc motor nếu có gear_ratio)
        wheel_radius: [m]
        gear_ratio: nếu rpm là rpm motor, còn wheel là rpm_motor / gear_ratio
                    nếu rpm đã là rpm bánh thì gear_ratio = 1.0
        """
        if self.gear_ratio != 0.0:
            wheel_rpm = rpm / self.gear_ratio
        else:
            wheel_rpm = rpm

        omega = wheel_rpm * (2.0 * math.pi / 60.0)  # rad/s
        v = omega * self.wheel_radius               # m/s
        return v

    # ===== Timer callback: publish TwistWithCovarianceStamped =====
    def timer_callback(self):
        with self._lock:
            left_rpm = self._last_left_rpm
            right_rpm = self._last_right_rpm

        # Đổi rpm -> m/s cho từng bánh
        v_left = self.rpm_to_ms(left_rpm)
        v_right = self.rpm_to_ms(right_rpm)

        # Differential drive:
        # v = (v_r + v_l) / 2
        # w = (v_r - v_l) / wheel_sep
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

        # Covariance đơn giản: chỉ điền cho v_x và w_z
        for i in range(36):
            msg.twist.covariance[i] = 0.0
        msg.twist.covariance[0] = 0.01   # var(v_x)
        msg.twist.covariance[35] = 0.01  # var(w_z)

        self.pub.publish(msg)


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
