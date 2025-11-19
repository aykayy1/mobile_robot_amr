# /home/manh/riclab_ui/page/nav_page.py
# -*- coding: utf-8 -*-

import math
import subprocess
import yaml
from collections import deque

from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget, QGridLayout, QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QMessageBox, QSizePolicy
)
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont

# ========= CẤU HÌNH ĐƯỜNG DẪN =========
LAUNCH_FILE  = "/home/anhkhoa/Mobile_robot/agv_ws/src/agv0509test6/launch/navigation_launch.py"
PRESET_GOALS = "/home/anhkhoa/Mobile_robot/agv_ws/src/agv0509test6/config/preset_goals.yaml"

# ========= ROS2 (tùy chọn) =========
ROS_OK = True
HAVE_NAV2 = False
HAVE_SCAN = False
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, Twist   # /cmd_vel
    from sensor_msgs.msg import LaserScan
    HAVE_SCAN = True
    try:
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        HAVE_NAV2 = True
    except Exception:
        HAVE_NAV2 = False
except Exception:
    ROS_OK = False
    HAVE_NAV2 = False
    HAVE_SCAN = False


# ---------- Worker ROS2 chạy trong thread ----------
class RosWorker(QThread):
    # Giữ odom_update cho tương thích
    odom_update  = pyqtSignal(float, float, float, float)   # (t, v_odom, x, y)
    # Mẫu cho biểu đồ & đo lường
    vel_sample   = pyqtSignal(float, float, float)          # (t, v_cmd, v_odom)
    ros_error    = pyqtSignal(str)
    lidar_points = pyqtSignal(list)                         # [(x,y), ...]
    goal_done    = pyqtSignal(str)                          # emit khi Nav2 báo hoàn tất (key_tag)

    def __init__(self):
        super().__init__()
        self._running = False
        self._start_time = None
        self.node = None
        self.goal_pub = None
        self.nav_client = None
        self._last_v_cmd = 0.0   # từ /cmd_vel

    def run(self):
        if not ROS_OK:
            self.ros_error.emit("ROS2 (rclpy) không khả dụng trên máy.")
            return
        try:
            rclpy.init(args=None)
            self.node = Node("ric_nav_gui")
            # /odom
            self.sub = self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
            # /cmd_vel
            self.cmd_sub = self.node.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
            # LiDAR
            if HAVE_SCAN:
                self.scan_sub = self.node.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
            # goal fallback topic
            self.goal_pub = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
            # Nav2 action
            if HAVE_NAV2:
                self.nav_client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

            self._running = True
            self._start_time = self.node.get_clock().now()
            while rclpy.ok() and self._running:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            self.ros_error.emit(f"ROS worker error: {e}")
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
                rclpy.try_shutdown()
            except Exception:
                pass

    def stop(self):
        self._running = False

    def _cmd_cb(self, msg: Twist):
        self._last_v_cmd = math.hypot(msg.linear.x, msg.linear.y)

    def _odom_cb(self, msg: Odometry):
        now = self.node.get_clock().now()
        t = (now - self._start_time).nanoseconds / 1e9 if self._start_time else 0.0
        v_odom = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_update.emit(t, v_odom, x, y)
        self.vel_sample.emit(t, self._last_v_cmd, v_odom)

    def _scan_cb(self, msg: LaserScan):
        pts = []
        ang = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                x = r * math.cos(ang)
                y = r * math.sin(ang)
                pts.append((x, y))
            ang += msg.angle_increment
        self.lidar_points.emit(pts)

    def send_pose_goal(self, x, y, yaw=0.0, frame_id="map", key_tag: str = ""):
        """Gửi 1 goal. Nếu có Nav2: emit goal_done(key_tag) khi hoàn tất."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        if HAVE_NAV2 and self.nav_client is not None:
            self.goal_pub.publish(pose)
            if not self.nav_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().warn("Nav2 action chưa sẵn sàng, fallback /goal_pose")
                self.goal_pub.publish(pose)
                return
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            send_future = self.nav_client.send_goal_async(goal_msg)

            def _on_goal_response(fut):
                goal_handle = fut.result()
                if not goal_handle:
                    self.node.get_logger().warn("Goal bị từ chối.")
                    return
                result_future = goal_handle.get_result_async()

                def _on_result(_):
                    if key_tag:
                        self.goal_done.emit(key_tag)

                result_future.add_done_callback(_on_result)

            send_future.add_done_callback(_on_goal_response)
        else:
            # Fallback: publish topic /goal_pose (không biết lúc nào tới)
            self.goal_pub.publish(pose)


# ---------- Matplotlib chart widget ----------
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class ChartsWidget(QWidget):
    """
    Trên: Velocity (v_cmd và v_odom)
    Dưới: Velocity Error (e = v_cmd - v_odom)
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.fig = Figure(figsize=(4, 6), tight_layout=True)
        self.canvas = FigureCanvas(self.fig)

        # Trên: Velocity (cmd vs odom)
        self.ax_v = self.fig.add_subplot(2, 1, 1)
        self.ax_v.set_title("Velocity")
        self.ax_v.set_xlabel("t (s)")
        self.ax_v.set_ylabel("m/s")

        # Dưới: Velocity Error
        self.ax_e = self.fig.add_subplot(2, 1, 2)
        self.ax_e.set_title("Velocity Error")
        self.ax_e.set_xlabel("t (s)")
        self.ax_e.set_ylabel("e (m/s)")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(6, 6, 6, 6)
        vbox.addWidget(self.canvas)

        # buffers
        self.t      = deque(maxlen=1200)
        self.v_cmd  = deque(maxlen=1200)
        self.v_odom = deque(maxlen=1200)
        self.e      = deque(maxlen=1200)

        # lines
        (self.line_v_cmd,)  = self.ax_v.plot([], [], lw=1.6, label="cmd")
        (self.line_v_odom,) = self.ax_v.plot([], [], lw=1.6, label="odom")
        (self.line_err,)    = self.ax_e.plot([], [], lw=1.6, label="e = cmd - odom")
        self.ax_v.legend(loc="upper right")
        self.ax_e.legend(loc="upper right")

    def push_sample(self, t, v_cmd, v_odom):
        self.t.append(t)
        self.v_cmd.append(v_cmd)
        self.v_odom.append(v_odom)
        self.e.append(v_cmd - v_odom)

    def redraw(self):
        # velocity
        self.line_v_cmd.set_data(self.t, self.v_cmd)
        self.line_v_odom.set_data(self.t, self.v_odom)
        self.ax_v.relim(); self.ax_v.autoscale_view()

        # error
        self.line_err.set_data(self.t, self.e)
        self.ax_e.relim(); self.ax_e.autoscale_view()

        self.canvas.draw_idle()


# ---------- GroupBox style ----------
def make_group(title: str) -> QGroupBox:
    gb = QGroupBox(title)
    gb.setObjectName("Card")
    return gb


# ---------- CAR VIEW (vẽ bán nguyệt & điểm LiDAR) ----------
class CarView(QWidget):
    """Vẽ bán nguyệt phía trước xe, trục x lên trên, y sang trái. Hiển thị LiDAR realtime."""
    def __init__(self, max_range_m=10.0, fov_deg=180.0, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.max_range = float(max_range_m)
        self.fov = math.radians(fov_deg)
        self.points = []

    def update_points(self, pts_xy):
        self.points = pts_xy or []
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        W, H = self.width(), self.height()
        margin = 12
        x0, y0 = margin, margin
        x1, y1 = W - margin, H - margin
        w, h  = x1 - x0, y1 - y0

        header_h = 20; footer_h = 18
        p.fillRect(self.rect(), QColor("#ffffff"))
        p.fillRect(x0, y0, w, header_h, QColor("#5b7f27"))
        p.fillRect(x0, y1 - footer_h, w, footer_h, QColor("#5b7f27"))
        p.setPen(Qt.white); p.setFont(QFont("", 9, QFont.DemiBold))
        p.drawText(x0 + 8, y0 + header_h - 6, "CAR VIEW")
        p.drawText(x0 + 8, y1 - 6, "Descartes Coordinate")

        usable_top = y0 + header_h + 3
        usable_bot = y1 - footer_h - 3
        usable_h   = max(usable_bot - usable_top, 1)

        cx = x0 + w // 2
        pad = 4
        R = max(10, min(w // 2 - pad, usable_h - pad))
        cy = usable_bot

        p.setPen(QPen(QColor("#222"), 2, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        arc_rect = (cx - R, cy - R, 2 * R, 2 * R)
        p.drawArc(*arc_rect, 0 * 16, 180 * 16)
        p.drawLine(cx - R, cy, cx + R, cy)

        p.setPen(QPen(QColor("#2c6e99"))); p.setFont(QFont("", 8))
        p.drawText(cx - R - 18, cy - 2, "180°")
        p.drawText(cx - 8,      cy - R - 6, "90°")
        p.drawText(cx + R + 4,  cy - 2, "0°")

        body_w, body_h = 36, 22
        body_x = cx - body_w // 2
        body_y = cy - body_h - 4

        wheel_w, wheel_h = 6, 14
        p.setPen(QPen(QColor("#333"), 1))
        p.setBrush(QBrush(QColor("#222")))
        p.drawRoundedRect(body_x - wheel_w - 2, body_y + (body_h - wheel_h) // 2,
                          wheel_w, wheel_h, 2, 2)
        p.drawRoundedRect(body_x + body_w + 2, body_y + (body_h - wheel_h) // 2,
                          wheel_w, wheel_h, 2, 2)

        p.setBrush(QBrush(QColor("#e9ecef"))); p.setPen(QPen(QColor("#4a4a4a"), 1))
        p.drawRoundedRect(body_x, body_y, body_w, body_h, 6, 6)

        p.setBrush(QBrush(QColor("#cfd8dc"))); p.setPen(Qt.NoPen)
        p.drawRoundedRect(body_x + 5, body_y + 3, body_w - 10, body_h // 2, 6, 6)

        p.setBrush(QBrush(QColor("#21c3b0"))); p.setPen(QPen(QColor("#0d6860"), 1))
        puck_r = 5
        p.drawEllipse(cx - puck_r, body_y + 4, 2 * puck_r, 2 * puck_r)

        p.setBrush(QBrush(QColor("#d9534f"))); p.setPen(QPen(QColor("#7a2d2b"), 1))
        p.drawEllipse(cx - 4, body_y - 8, 8, 8)

        p.setBrush(QBrush(QColor("#ff8c42"))); p.setPen(QPen(QColor("#b85b24"), 1))
        tri = [(cx, body_y - 12), (cx - 6, body_y - 2), (cx + 6, body_y - 2)]
        from PyQt5.QtCore import QPoint
        p.drawPolygon(*[QPoint(*pt) for pt in tri])

        if self.max_range <= 0:
            self.max_range = 1.0
        scale = R / self.max_range
        p.setBrush(QBrush(QColor(255, 80, 0, 200))); p.setPen(Qt.NoPen)
        for (x, y) in self.points:
            if x < 0:
                continue
            sx = cx + int(y * scale)
            sy = cy - int(x * scale)
            if (sx - cx) ** 2 + (sy - cy) ** 2 <= R ** 2 + 1:
                p.drawEllipse(sx - 2, sy - 2, 4, 4)


# ---------- Runner cho Traveling (đọc GOALx từ YAML, đợi tới đích nếu có Nav2) ----------
class TravelRunner(QThread):
    status   = pyqtSignal(str)  # thông điệp trạng thái (UI)
    finished = pyqtSignal(str)  # thông điệp khi xong / dừng

    def __init__(self, ros_worker: RosWorker, yaml_path: str, sequence_name: str, parent=None):
        super().__init__(parent)
        self.ros = ros_worker
        self.yaml_path = yaml_path
        self.sequence_name = sequence_name
        self._stop = False
        # chờ tới đích (chỉ dùng khi HAVE_NAV2)
        self._await_key = None
        self._reached   = False

        if HAVE_NAV2 and self.ros:
            self.ros.goal_done.connect(self._on_goal_done)

    def stop(self):
        self._stop = True

    def _on_goal_done(self, key_tag: str):
        if key_tag == self._await_key:
            self._reached = True

    def run(self):
        if not self.ros or (not self.ros.isRunning()):
            self.status.emit("Chưa Connect ROS2.")
            self.finished.emit("Traveling dừng (ROS chưa chạy).")
            return

        # Đọc YAML
        try:
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.status.emit(f"Lỗi YAML: {e}")
            self.finished.emit("Traveling dừng (YAML lỗi).")
            return

        seq = data.get(self.sequence_name, {}).get("sequence", [])
        if not isinstance(seq, list) or not seq:
            self.status.emit(f"Không tìm thấy sequence '{self.sequence_name}'.")
            self.finished.emit("Traveling dừng.")
            return

        # Hàm đọc 1 điểm (C205A1/C205A2/C20A3/C20A4)
        def read_point(key):
            d = data.get(key, {})
            if not isinstance(d, dict):
                return None
            frame = d.get("frame_id", "map")
            pos = d.get("position", {}) or {}
            ori = d.get("orientation", {}) or {}
            x = float(pos.get("x", 0.0))
            y = float(pos.get("y", 0.0))
            zq = float(ori.get("z", 0.0))
            wq = float(ori.get("w", 1.0))
            yaw = math.atan2(2.0 * wq * zq, 1.0 - 2.0 * zq * zq)
            return (x, y, yaw, frame)

        # Chạy từng bước
        for step in seq:
            if self._stop:
                self.finished.emit("Traveling dừng (bị hủy).")
                return
            if not isinstance(step, dict) or "point" not in step:
                continue

            point_key = str(step["point"])
            wait_sec  = float(step.get("wait_sec", 0))

            pose = read_point(point_key)
            if pose is None:
                self.status.emit(f"Không tìm thấy điểm '{point_key}'. Bỏ qua.")
                continue

            x, y, yaw, frame = pose
            self.status.emit(f"Đang tới {point_key}…")

            if HAVE_NAV2 and self.ros.nav_client is not None:
                self._await_key = point_key
                self._reached = False
                self.ros.send_pose_goal(x, y, yaw, frame_id=frame, key_tag=point_key)
                while (not self._reached) and (not self._stop):
                    self.msleep(100)
                if self._stop:
                    self.finished.emit("Traveling dừng (bị hủy).")
                    return
                self.status.emit(f"Đã tới {point_key}. Đợi {int(wait_sec)}s…")
                waited = 0
                total_ms = max(0, int(wait_sec * 1000))
                while waited < total_ms and not self._stop:
                    self.msleep(100)
                    waited += 100
                if self._stop:
                    self.finished.emit("Traveling dừng (bị hủy).")
                    return
            else:
                self.ros.send_pose_goal(x, y, yaw, frame_id=frame)
                self.status.emit(f"(Fallback) Đang tới {point_key}… chờ {int(wait_sec)}s")
                waited = 0
                total_ms = max(0, int(wait_sec * 1000))
                while waited < total_ms and not self._stop:
                    self.msleep(100)
                    waited += 100
                if self._stop:
                    self.finished.emit("Traveling dừng (bị hủy).")
                    return

        self.finished.emit(f"Traveling '{self.sequence_name}' hoàn tất.")


class NavPage(QWidget):
    """
    Cột trái 4 hàng (3 gọn + 1 giãn):
      0) Navigation Connect (gọn)
      1) Static Obstacle (gọn)
      2) Traveling (gọn)
      3) CAR VIEW (giãn)
    Cột phải: Chart chiếm nửa màn hình, cao bằng 4 hàng trái.
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        grid = QGridLayout(self)
        grid.setContentsMargins(10, 10, 10, 10)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        # ===== 1) Navigation Connect =====
        self.gb_connect = make_group("Navigation Connect")
        self.gb_connect.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        grid.addWidget(self.gb_connect, 0, 0, 1, 1)

        c_lay = QVBoxLayout(self.gb_connect)
        c_lay.setContentsMargins(10, 10, 10, 10)
        c_lay.setSpacing(8)
        row = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect"); self.btn_disconnect.setEnabled(False)
        row.addWidget(self.btn_connect); row.addWidget(self.btn_disconnect); row.addStretch(1)
        c_lay.addLayout(row)

        self.state_label = QLabel("")
        self.state_label.setAlignment(Qt.AlignCenter)
        c_lay.addWidget(self.state_label)

        # ===== 2) Static Obstacle =====
        self.gb_static = make_group("Static Obstacle")
        self.gb_static.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        grid.addWidget(self.gb_static, 1, 0, 1, 1)

        s_lay = QVBoxLayout(self.gb_static)
        s_lay.setContentsMargins(10, 10, 10, 10)
        s_lay.setSpacing(8)
        buttons = QHBoxLayout()
        self.goal_buttons = []
        pairs = [("C205A1", "C205A1"),
                 ("C205A2", "C205A2"),
                 ("C205A3", "C20A3"),
                 ("C205A4", "C20A4")]
        for btn_name, yaml_key in pairs:
            b = QPushButton(btn_name)
            b.setProperty("goal_key", yaml_key)
            self.goal_buttons.append(b)
            buttons.addWidget(b)
        buttons.addStretch(1)
        s_lay.addLayout(buttons)

        # ===== 3) Traveling =====
        self.gb_travel = make_group("Traveling")
        self.gb_travel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        grid.addWidget(self.gb_travel, 2, 0, 1, 1)

        t_lay = QVBoxLayout(self.gb_travel)
        t_lay.setContentsMargins(10, 10, 10, 10)
        t_lay.setSpacing(8)
        trow = QHBoxLayout()
        self.btn_goal1 = QPushButton("GOAL1")
        self.btn_goal2 = QPushButton("GOAL2")
        self.btn_goal3 = QPushButton("GOAL3")
        for b in (self.btn_goal1, self.btn_goal2, self.btn_goal3):
            trow.addWidget(b)
        trow.addStretch(1)
        t_lay.addLayout(trow)

        # Runner traveling
        self.travel_runner = None
        self.btn_goal1.clicked.connect(lambda: self._start_travel("GOAL1"))
        self.btn_goal2.clicked.connect(lambda: self._start_travel("GOAL2"))
        self.btn_goal3.clicked.connect(lambda: self._start_travel("GOAL3"))

        # ===== 4) CAR VIEW (giãn) =====
        self.gb_car = make_group("CAR VIEW")
        self.gb_car.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        grid.addWidget(self.gb_car, 3, 0, 1, 1)

        cv_lay = QVBoxLayout(self.gb_car)
        cv_lay.setContentsMargins(6, 8, 6, 8)
        self.car_view = CarView(max_range_m=10.0, fov_deg=180.0)
        cv_lay.addWidget(self.car_view)

        # ===== Chart (cột phải, cao 4 hàng) =====
        self.gb_chart = make_group("Chart")
        grid.addWidget(self.gb_chart, 0, 1, 4, 1)
        chart_lay = QVBoxLayout(self.gb_chart)
        chart_lay.setContentsMargins(8, 12, 8, 12)
        self.charts = ChartsWidget()
        chart_lay.addWidget(self.charts)

        # ===== style =====
        self.setStyleSheet("""
            QGroupBox#Card {
                border: 2px solid #386fa4;
                border-radius: 6px;
                margin-top: 22px;
                background: #ffffff;
            }
            QGroupBox#Card::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 2px 8px;
                background: #274c77;
                color: white;
                border-radius: 4px;
                margin-left: 8px;
                font-weight: bold;
            }
            QPushButton { min-width: 90px; min-height: 30px; }
            QLabel { font-size: 14px; }
        """)

        # ===== ROS worker + timer =====
        self.ros = RosWorker() if ROS_OK else None
        if self.ros:
            # dùng mẫu vận tốc cho chart và đo lường
            self.ros.vel_sample.connect(self._on_vel_sample)
            self.ros.ros_error.connect(self._on_ros_error)
            self.ros.lidar_points.connect(self.car_view.update_points)
            if HAVE_NAV2:
                self.ros.goal_done.connect(self._on_goal_done)  # để kết thúc phiên đo

        self.redraw_timer = QTimer(self)
        self.redraw_timer.setInterval(200)  # 5Hz
        self.redraw_timer.timeout.connect(self.charts.redraw)

        # ===== events =====
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        self.btn_disconnect.clicked.connect(self._on_disconnect_clicked)
        for b in self.goal_buttons:
            b.clicked.connect(self._on_goal_clicked)

        # ===== chia tỉ lệ lưới =====
        grid.setColumnStretch(0, 1)  # trái
        grid.setColumnStretch(1, 1)  # phải (Chart)
        grid.setRowStretch(0, 0)     # Connect gọn
        grid.setRowStretch(1, 0)     # Static gọn
        grid.setRowStretch(2, 0)     # Traveling gọn
        grid.setRowStretch(3, 1)     # CarView giãn phần còn lại

        self.proc = None

        # ====== PHIÊN ĐO LƯỜNG (mới) ======
        self._measuring = False
        self._measure_key = None
        self._measure_started_t = None
        self._samples_cmd = []
        self._samples_odom = []
        self._fallback_timer = QTimer(self)   # dùng khi không có Nav2
        self._fallback_timer.setSingleShot(True)
        self._fallback_timer.timeout.connect(self._end_measure_fallback)

    # --------- Handlers ---------
    def _on_connect_clicked(self):
        try:
            cmd = f"ros2 launch {LAUNCH_FILE}"
            self.proc = subprocess.Popen(["bash", "-lc", cmd])
        except Exception as e:
            QMessageBox.critical(self, "Launch error", f"Không thể chạy launch:\n{e}")
            return

        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.state_label.setText('<span style="color:#1e7e34;font-weight:600;">Connected</span>')

        if self.ros and not self.ros.isRunning():
            self.ros.start()
            self.redraw_timer.start()

    def _on_disconnect_clicked(self):
        try:
            if self.proc:
                self.proc.terminate()
                self.proc = None
        except Exception:
            pass
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.state_label.setText("")
        if self.travel_runner and self.travel_runner.isRunning():
            self.travel_runner.stop()
            self.travel_runner = None
        # kết thúc phiên đo nếu còn
        if self._measuring:
            self._finish_measure(force=True, note="Đã ngắt kết nối - dừng phiên đo.")
        if self.ros and self.ros.isRunning():
            self.ros.stop()

    def _on_vel_sample(self, t, v_cmd, v_odom):
        self.charts.push_sample(t, v_cmd, v_odom)
        # thu thập mẫu cho phiên đo
        if self._measuring:
            if self._measure_started_t is None:
                self._measure_started_t = t
            self._samples_cmd.append(v_cmd)
            self._samples_odom.append(v_odom)

    def _on_ros_error(self, msg):
        QMessageBox.warning(self, "ROS2", msg)

    def _on_goal_clicked(self):
        sender = self.sender()
        yaml_key = sender.property("goal_key")
        goal = self._read_goal_from_yaml(yaml_key)
        if goal is None:
            QMessageBox.warning(self, "Preset goals",
                                f"Không tìm thấy '{yaml_key}' trong:\n{PRESET_GOALS}")
            return
        if not self.ros or not self.ros.isRunning():
            QMessageBox.information(self, "Navigation",
                                    "Chưa Connect ROS2. Hãy nhấn Connect trước.")
            return

        # BẮT ĐẦU PHIÊN ĐO cho điểm này
        self._start_measure(yaml_key)

        x, y, yaw, frame_id = goal
        self.ros.send_pose_goal(x, y, yaw, frame_id=frame_id, key_tag=yaml_key)

        # Thông báo
        btn_label = sender.text()
        msg = f"Vị trí {btn_label} đã được set. Robot di chuyển!"
        self.state_label.setText(f'<span style="color:#0d6efd;font-weight:600;">{msg}</span>')
        QMessageBox.information(self, "Navigation", msg)

    # ---- Traveling ----
    def _start_travel(self, seq_name: str):
        if not self.ros or not self.ros.isRunning():
            QMessageBox.information(self, "Traveling", "Chưa Connect ROS2. Hãy nhấn Connect trước.")
            return
        # Hủy runner cũ nếu đang chạy
        if self.travel_runner and self.travel_runner.isRunning():
            self.travel_runner.stop()
            self.travel_runner.wait(100)

        self.travel_runner = TravelRunner(self.ros, PRESET_GOALS, seq_name)
        self.travel_runner.status.connect(self._travel_status)
        self.travel_runner.finished.connect(self._travel_finished)
        self.state_label.setText(f'<span style="color:#0d6efd;font-weight:600;">Đang chạy {seq_name}…</span>')
        self.travel_runner.start()

    def _travel_status(self, text: str):
        self.state_label.setText(f'<span style="color:#0d6efd;font-weight:600;">{text}</span>')

    def _travel_finished(self, text: str):
        QMessageBox.information(self, "Traveling", text)
        self.state_label.setText(f'<span style="color:#198754;font-weight:600;">{text}</span>')

    # --------- YAML parser ---------
    def _read_goal_from_yaml(self, key):
        try:
            with open(PRESET_GOALS, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            QMessageBox.critical(self, "YAML error", f"Không đọc được YAML:\n{e}")
            return None
        if key not in data or not isinstance(data[key], dict):
            return None

        d = data[key]
        frame = d.get("frame_id", "map")
        pos = d.get("position", {}) or {}
        ori = d.get("orientation", {}) or {}
        x = float(pos.get("x", 0.0))
        y = float(pos.get("y", 0.0))
        zq = float(ori.get("z", 0.0))
        wq = float(ori.get("w", 1.0))
        yaw = math.atan2(2.0 * wq * zq, 1.0 - 2.0 * zq * zq)
        return (x, y, yaw, frame)

    # ====== ĐO LƯỜNG: bắt đầu/kết thúc/hoàn tất ======
    def _start_measure(self, key: str):
        """Bắt đầu phiên đo cho 1 điểm (ví dụ 'C205A2')."""
        # Reset cũ nếu đang đo
        if self._measuring:
            self._finish_measure(force=True, note="Bắt đầu điểm mới, kết thúc phiên cũ.")

        self._measuring = True
        self._measure_key = key
        self._measure_started_t = None
        self._samples_cmd = []
        self._samples_odom = []

        # Nếu không có Nav2: tự dừng sau 120 giây (ước lượng)
        if not HAVE_NAV2:
            self._fallback_timer.start(120_000)

    def _on_goal_done(self, key_tag: str):
        """Nav2 báo goal hoàn tất -> nếu trùng điểm đang đo thì kết thúc đo."""
        if self._measuring and key_tag == self._measure_key:
            self._finish_measure()

    def _end_measure_fallback(self):
        """Không có Nav2 -> dừng sau timeout."""
        if self._measuring:
            self._finish_measure(note="(Ước lượng) Dừng sau 120s do không có tín hiệu hoàn tất từ Nav2.")

    def _finish_measure(self, force: bool = False, note: str = ""):
        """Tính và hiển thị kết quả phiên đo hiện tại (popup-only)."""
        if not self._measuring:
            return

        self._measuring = False
        self._fallback_timer.stop()

        n = len(self._samples_odom)
        if n == 0 or self._measure_started_t is None:
            QMessageBox.information(self, "Kết quả đo",
                                    "Chưa có dữ liệu mẫu nào để tính toán.")
            return

        # Trung bình vận tốc (odom)
        v_avg = sum(self._samples_odom) / n
        # Sai số trung bình tuyệt đối |cmd - odom|
        abs_errs = [abs(vc - vo) for vc, vo in zip(self._samples_cmd, self._samples_odom)]
        e_avg = sum(abs_errs) / n

        # Thời gian chạy: dùng t đầu/cuối đã lưu khi nhận mẫu vel_sample
        duration = 0.0
        if hasattr(self, "_measure_t_first") and hasattr(self, "_measure_t_last") \
        and self._measure_t_first is not None and self._measure_t_last is not None:
            duration = max(0.0, self._measure_t_last - self._measure_t_first)

        # reset mốc thời gian phiên đo
        self._measure_t_first = None
        self._measure_t_last  = None

        title = f"Kết quả đo – {self._measure_key}"
        lines = [
            f"Thời gian chạy: {duration:.2f} s",
            f"Vận tốc trung bình (odom): {v_avg:.3f} m/s",
            f"Sai số vận tốc trung bình |cmd-odom|: {e_avg:.3f} m/s",
        ]
        if note:
            lines.append(note)

        # >>> POPUP ONLY (không đụng tới self.state_label ở đây) <<<
        QMessageBox.information(self, title, "\n".join(lines))

        # clear session
        self._measure_key = None
        self._samples_cmd.clear()
        self._samples_odom.clear()


    # --------- cleanup ---------
    def closeEvent(self, e):
        try:
            if self.travel_runner and self.travel_runner.isRunning():
                self.travel_runner.stop()
        except Exception:
            pass
        try:
            if self.ros and self.ros.isRunning():
                self.ros.stop()
        except Exception:
            pass
        try:
            if self.proc:
                self.proc.terminate()
        except Exception:
            pass
        super().closeEvent(e)

    # === Ghi đè _on_vel_sample để lưu t đầu/cuối cho phiên đo ===
    def _on_vel_sample(self, t, v_cmd, v_odom):
        # cập nhật chart
        self.charts.push_sample(t, v_cmd, v_odom)
        # lưu mẫu cho phiên đo
        if self._measuring:
            if self._measure_started_t is None:
                self._measure_started_t = t
            # Lưu t_first/t_last chính xác từ tín hiệu vẽ (theo odom time)
            if not hasattr(self, "_measure_t_first") or self._measure_t_first is None:
                self._measure_t_first = t
            self._measure_t_last = t

            self._samples_cmd.append(v_cmd)
            self._samples_odom.append(v_odom)
