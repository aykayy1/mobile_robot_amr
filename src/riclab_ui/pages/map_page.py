# pages/map_page.py
from __future__ import annotations

# ========== Qt ==========
from PyQt5.QtCore import Qt, QTimer, QObject, pyqtSignal, QRectF
from PyQt5.QtGui import QPixmap, QPainter, QTransform, QImage, QPen, QColor
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGraphicsView, QGraphicsScene,
    QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsLineItem
)

# ========== ROS2 ==========
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor


# TF2: dùng nếu có; nếu thiếu vẫn chạy với /amcl_pose
HAVE_TF2 = True
try:
    from tf2_ros import Buffer, TransformListener
    try:
        from rclpy.duration import Duration  # không phải bản nào cũng có
    except Exception:
        Duration = None
except Exception:
    HAVE_TF2 = False
    Buffer = None
    TransformListener = None
    Duration = None


# ========== Utils ==========
def quat_to_yaw(q) -> float:
    """Yaw (rad) từ quaternion geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ========== Qt View (pan/zoom/rotate) ==========
class MapView(QGraphicsView):
    """Pan (chuột trái), Zoom (wheel), Rotate (giữ chuột phải kéo ngang), phím 0 để reset/fit."""
    def __init__(self):
        super().__init__()
        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setBackgroundBrush(Qt.gray)  # nền xám giống RViz
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self._scale = 1.0
        self._angle = 0.0
        self._rot_drag = None

    def wheelEvent(self, e):
        delta = e.angleDelta().y()
        factor = 1.0 + (abs(delta) / 120.0) * 0.1
        if delta < 0:
            factor = 1.0 / factor
        self._scale *= factor
        self._apply_transform()
        e.accept()

    def mousePressEvent(self, e):
        if e.button() == Qt.RightButton:
            self._rot_drag = e.pos()
            e.accept()
        else:
            super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._rot_drag is not None:
            dx = e.x() - self._rot_drag.x()
            self._angle += dx * 0.3
            self._apply_transform()
            self._rot_drag = e.pos()
            e.accept()
        else:
            super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.RightButton:
            self._rot_drag = None
            e.accept()
        else:
            super().mouseReleaseEvent(e)

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_0:
            self._scale, self._angle = 1.0, 0.0
            self.resetTransform()
            self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
            e.accept()
        else:
            super().keyPressEvent(e)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self.resetTransform()
        self._apply_transform(fit=True)

    def _apply_transform(self, fit=False):
        if fit:
            self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
        t = QTransform()
        t.rotate(self._angle)
        t.scale(self._scale, self._scale)
        self.setTransform(t)


# ========== Bridge (ROS -> Qt) ==========
class _QtBridge(QObject):
    map_image_ready = pyqtSignal(QImage, float, float, float, int, int)  # image, res, ox, oy, w, h
    robot_pose_ready = pyqtSignal(float, float, float)  # x, y, yaw (rad)
    goal_pose_ready  = pyqtSignal(float, float, float)  # x, y, yaw (rad)


# ========== ROS Node ==========
class RosMapNode(Node):
    def __init__(self, bridge: _QtBridge, *, context=None):
        super().__init__('map_page_ros_node', context=context)
        self.bridge = bridge

        # /map cần TRANSIENT_LOCAL để nhận lại latched map
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        # Các topic khác QoS mặc định
        default_qos = QoSProfile(depth=10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, default_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, default_qos)

        # TF (tùy chọn)
# TF (tùy chọn) – ưu tiên Base_footprint và tự dò
# TF (tùy chọn) – ép cứng dùng map -> Base_footprint
        self.use_tf = False
        self.tf_ok = False
        self.current_pair = ('map', 'Base_footprint')  # <<< ÉP CỨNG Ở ĐÂY
        self.tf_ok = True
        self.create_timer(0.1, self._tick_tf)
        self.get_logger().info(f"Using TF frames: {self.current_pair[0]} -> {self.current_pair[1]}")

        if HAVE_TF2:
            try:
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.use_tf = True
                # tick 10 Hz
                self.create_timer(0.1, self._tick_tf)
                self.get_logger().info("TF2 enabled: prefer map->Base_footprint, auto-detect fallback…")
            except Exception as e:
                self.get_logger().warn(f"TF2 init failed ({e}); fallback to /amcl_pose")
        else:
            self.get_logger().warn("tf2_ros not found; fallback to /amcl_pose")
    # OccupancyGrid -> QImage (RGBA8888) + meta
    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        w, h = int(info.width), int(info.height)
        if w == 0 or h == 0:
            return

        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        # 0 (free)->255 trắng; 100 (occ)->0 đen; -1 (unknown)->200 xám nhạt
        gray = np.empty((h, w), dtype=np.uint8)
        unknown = data < 0
        occ = (~unknown) & (data > 0)
        free = (~unknown) & (data == 0)
        gray[unknown] = 200
        gray[free] = 255
        scale = (255 - (np.clip(data, 0, 100) * 255 // 100)).astype(np.uint8)
        gray[occ] = scale[occ]

        # Lật dọc (OccupancyGrid gốc trục Y lên; ảnh Y xuống)
        gray = np.flipud(gray)

        rgba = np.dstack([gray, gray, gray, np.full_like(gray, 255)])
        qimg = QImage(rgba.data, w, h, w * 4, QImage.Format_RGBA8888).copy()

        res = float(info.resolution)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        self.bridge.map_image_ready.emit(qimg, res, ox, oy, w, h)

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        # Fallback nếu không có TF
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.bridge.robot_pose_ready.emit(float(p.x), float(p.y), float(yaw))

    def _on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        yaw = quat_to_yaw(msg.pose.orientation)
        self.bridge.goal_pose_ready.emit(float(p.x), float(p.y), float(yaw))

    def _tick_tf(self):
        if not (self.use_tf and self.tf_ok and self.current_pair):
            return
        try:
            map_f, base_f = self.current_pair
            if Duration is not None:
                tf = self.tf_buffer.lookup_transform(map_f, base_f, rclpy.time.Time(),
                                                    timeout=Duration(seconds=0.05))
            else:
                tf = self.tf_buffer.lookup_transform(map_f, base_f, rclpy.time.Time())
            tr = tf.transform.translation
            q  = tf.transform.rotation
            yaw = quat_to_yaw(q)
            self.bridge.robot_pose_ready.emit(float(tr.x), float(tr.y), float(yaw))
            self.get_logger().debug(f"TF pose: x={tr.x:.3f}, y={tr.y:.3f}, yaw={yaw:.3f} rad")
        except Exception as e:
            # comment dòng dưới nếu muốn im lặng
            self.get_logger().debug(f"TF lookup failed ({e})")



# ========== MapPage Widget ==========
class MapPage(QWidget):
    """
    - Hiển thị /map (OccupancyGrid) bằng QGraphicsView.
    - KHÔNG cho chấm điểm. Chỉ hiển thị robot (TF hoặc /amcl_pose) + hướng (đường đỏ).
    - Nhận /goal_pose để vẽ mục tiêu.
    """
    def __init__(self):
        super().__init__()

        # UI
        self.view = MapView()
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.view)

        # Map pixmap + overlay items
        self._pixitem: QGraphicsPixmapItem | None = None

        self._robot_item = QGraphicsEllipseItem()
        self._heading_item = QGraphicsLineItem()
        self._goal_item = QGraphicsEllipseItem()

        # Styles
        red = QPen(QColor("#d9534f")); red.setWidth(2)
        self._heading_item.setPen(red); self._heading_item.setZValue(10)

        robot_pen = QPen(QColor("#004085")); robot_pen.setWidth(2)
        self._robot_item.setPen(robot_pen)
        self._robot_item.setBrush(QColor("#5bc0de"))
        self._robot_item.setZValue(9)

        goal_pen = QPen(QColor("#198754")); goal_pen.setWidth(2)
        self._goal_item.setPen(goal_pen)
        self._goal_item.setBrush(QColor(25, 135, 84, 60))
        self._goal_item.setZValue(8)

        self.scene.addItem(self._heading_item)
        self.scene.addItem(self._robot_item)
        self.scene.addItem(self._goal_item)
        self._robot_item.setVisible(False)
        self._heading_item.setVisible(False)
        self._goal_item.setVisible(False)

        # Map meta
        self._res = 0.05
        self._ox = 0.0
        self._oy = 0.0
        self._map_w = 0
        self._map_h = 0

        # Pose cache
        self._last_robot = None  # (x, y, yaw)
        self._last_goal  = None  # (x, y, yaw)

        # Bridge & ROS executor
        self._bridge = _QtBridge()
        self._bridge.map_image_ready.connect(self._on_qimage_map)
        self._bridge.robot_pose_ready.connect(self._on_robot_pose)
        self._bridge.goal_pose_ready.connect(self._on_goal_pose)
    #    ===== ROS2 context riêng cho MapPage =====
        self._ctx = Context()
        rclpy.init(args=None, context=self._ctx)

        try:
            self._executor = SingleThreadedExecutor(context=self._ctx)
        except TypeError:
            # Phòng khi bản rclpy cũ không nhận tham số context
            self._executor = SingleThreadedExecutor()

        # Tạo node và add vào executor
        try:
            self._node = RosMapNode(self._bridge, context=self._ctx)
        except TypeError:
            self._node = RosMapNode(self._bridge)

        try:
            self._executor.add_node(self._node)
        except Exception:
            pass

        # QTimer để spin non-blocking (KHÔNG dùng lambda gọi spin_once trực tiếp)
        self._spin_timer = QTimer(self)
        self._spin_timer.setInterval(10)  # có thể đổi 20–50ms nếu muốn CPU nhẹ hơn
        self._spin_timer.timeout.connect(self._on_spin_tick)
        self._spin_timer.start()

    # ---- helpers: world (m) -> pixel (ảnh đã lật Y) ----
    def _world_to_px(self, x_m: float, y_m: float):
        if self._res <= 0.0 or self._map_w == 0 or self._map_h == 0:
            return None
        px = (x_m - self._ox) / self._res
        py_raw = (y_m - self._oy) / self._res
        py = self._map_h - 1 - py_raw  # vì đã flipud
        return (px, py)

    # ---- Qt slots ----
    def _on_qimage_map(self, img: QImage, res: float, ox: float, oy: float, w: int, h: int):
        self._res, self._ox, self._oy, self._map_w, self._map_h = res, ox, oy, w, h
        pm = QPixmap.fromImage(img)
        if self._pixitem is None:
            self._pixitem = self.scene.addPixmap(pm)
            self.scene.setSceneRect(self._pixitem.boundingRect())  # QRectF
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        else:
            self._pixitem.setPixmap(pm)
            self.scene.setSceneRect(self._pixitem.boundingRect())

    def _on_spin_tick(self):
    # Chỉ spin khi context còn hợp lệ
        try:
            if getattr(self, "_ctx", None) is not None and rclpy.ok(context=self._ctx):
                try:
                    self._executor.spin_once(timeout_sec=0.01)
                except Exception:
                    # Có race-condition khi đang tắt → dừng timer
                    if hasattr(self, "_spin_timer"):
                        self._spin_timer.stop()
            else:
                if hasattr(self, "_spin_timer"):
                    self._spin_timer.stop()
        except Exception:
            try:
                if hasattr(self, "_spin_timer"):
                    self._spin_timer.stop()
            except Exception:
                pass



    def closeEvent(self, e):
    # Dừng timer trước
        try:
            if hasattr(self, "_spin_timer"):
                self._spin_timer.stop()
        except Exception:
            pass

        # Tháo node và shutdown
        try:
            if hasattr(self, "_executor") and hasattr(self, "_node"):
                try:
                    self._executor.remove_node(self._node)
                except Exception:
                    pass
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        except Exception:
            pass

        try:
            if hasattr(self, "_executor"):
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
        except Exception:
            pass

        try:
            if hasattr(self, "_ctx") and self._ctx is not None:
                try:
                    rclpy.shutdown(context=self._ctx)
                except Exception:
                    pass
        except Exception:
            pass

        super().closeEvent(e)


    def _on_robot_pose(self, x: float, y: float, yaw: float):
        self._last_robot = (x, y, yaw)
        self._update_robot_overlay(x, y, yaw)

    def _on_goal_pose(self, x: float, y: float, yaw: float):
        self._last_goal = (x, y, yaw)
        self._update_goal_overlay(x, y, yaw)

    def _update_robot_overlay(self, x: float, y: float, yaw: float):
        pt = self._world_to_px(x, y)
        if not pt:
            return
        px, py = pt

        # DEBUG: in px/py và kích thước map
        print(f"[map_page] robot world=({x:.3f},{y:.3f}) -> px=({px:.1f},{py:.1f}) map=({self._map_w},{self._map_h})")

        # Nếu nằm ngoài ảnh thì ẩn (tránh lạc trục) và thoát
        if not (0 <= px < self._map_w and 0 <= py < self._map_h):
            self._robot_item.setVisible(False)
            self._heading_item.setVisible(False)
            return

        # Chấm robot to hơn + đặt Z cao để chắc chắn nổi lên trên
        r = 10.0
        self._robot_item.setRect(px - r, py - r, 2 * r, 2 * r)
        self._robot_item.setBrush(QColor("#ff4d4f"))   # đỏ dễ thấy
        self._robot_item.setPen(QPen(Qt.black, 2))
        self._robot_item.setZValue(100)
        self._robot_item.setVisible(True)

        # Hướng robot (đường đỏ dài hơn)
        L = 40.0
        hx = px + L * math.cos(yaw)
        hy = py - L * math.sin(yaw)  # y ảnh đi xuống
        pen = QPen(QColor("#ff4d4f"), 3)
        self._heading_item.setPen(pen)
        self._heading_item.setZValue(101)
        self._heading_item.setLine(px, py, hx, hy)
        self._heading_item.setVisible(True)



    def _update_goal_overlay(self, x: float, y: float, yaw: float):
        pt = self._world_to_px(x, y)
        if not pt:
            return
        px, py = pt
        r = 6.0
        self._goal_item.setRect(px - r, py - r, 2 * r, 2 * r)
        self._goal_item.setVisible(True)

    # ---- Lifecycle ----
    def closeEvent(self, e):
        try:
            self._spin_timer.stop()
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(e)
