#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from pathlib import Path
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap, QIcon, QPainter
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QToolBar, QAction, QToolButton,
    QHBoxLayout, QVBoxLayout, QStatusBar, QStyle, QSizePolicy, QFrame,
    QStackedWidget
)

# ====== PATHS ======
LOGO_PATH = "/home/manh/picture/logo_ric.jpg"
CENTER_IMAGE_PATH = "/home/manh/picture/main.png"
MAP_PGM_PATH = "/home/anhkhoa/Mobile_robot/agv_ws/maps/new_map.pgm"

# ====== IMPORT MapPage từ file ngoài (/home/manh/riclab_ui/page/map_page.py) ======
BASE_DIR = Path(__file__).resolve().parent
PAGE_DIR = BASE_DIR / "page"
# Cho phép import trực tiếp 'map_page' dù chưa có __init__.py
if str(PAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PAGE_DIR))
from pages.map_page import MapPage  # <-- lớp MapPage(QWidget) trong page/map_page.py
from pages.nav_page import NavPage

# ---- GHIM icon sát lề trái ----
def left_pinned_icon(icon: QIcon, box=QSize(28, 28)) -> QIcon:
    src = icon.pixmap(128, 128)
    if src.isNull():
        pm = QPixmap(box); pm.fill(Qt.transparent); return QIcon(pm)
    pm = QPixmap(box); pm.fill(Qt.transparent)
    tgt = src.scaled(box, Qt.KeepAspectRatio, Qt.SmoothTransformation)
    p = QPainter(pm); p.drawPixmap(0, (box.height() - tgt.height()) // 2, tgt); p.end()
    return QIcon(pm)

# ---- Icon hệ thống ----
def std_icon(style: QStyle, name: QStyle.StandardPixmap) -> QIcon:
    ic = style.standardIcon(name)
    if not ic.isNull(): return ic
    dummy = QPixmap(24, 24); dummy.fill(Qt.transparent); return QIcon(dummy)

class ScaledImageLabel(QLabel):
    """Ảnh home co giãn theo tỉ lệ (contain)."""
    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pix = None
    def setImage(self, pix: QPixmap):
        self._pix = pix; self._update()
    def resizeEvent(self, e):
        super().resizeEvent(e); self._update()
    def _update(self):
        if not self._pix or self._pix.isNull():
            self.setText("No image"); return
        target = self.size() - QSize(8, 8)
        if target.width() <= 0 or target.height() <= 0: return
        self.setPixmap(self._pix.scaled(target, Qt.KeepAspectRatio, Qt.SmoothTransformation))

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(1200, 800)
        self.setMinimumSize(1024, 640)
        self.setWindowTitle("RICLAB – AMR Control Panel")

        # ================== STACK (nội dung trung tâm) ==================
        self.stack = QStackedWidget()

        # Trang Home (ảnh)
        self.home_image = ScaledImageLabel()
        pm = QPixmap(CENTER_IMAGE_PATH)
        if pm.isNull(): self.home_image.setText("Slide image not found")
        else: self.home_image.setImage(pm)

        # Trang Map
        self.map_page = MapPage()
        # Trang Navigation
        self.nav_page = NavPage()
        

        self.stack.addWidget(self.home_image)  # index 0
        self.stack.addWidget(self.map_page)    # index 1
        self.stack.addWidget(self.nav_page)

        # ================== Sidebar trái ==================
        left_panel = QFrame(objectName="LeftPanel")
        left_panel.setFixedWidth(180)
        lv = QVBoxLayout(left_panel)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(8)

        # logo
        logo_lbl = QLabel()
        logo_pix = QPixmap(LOGO_PATH)
        if not logo_pix.isNull():
            logo_lbl.setPixmap(logo_pix.scaledToHeight(56, Qt.SmoothTransformation))
        else:
            logo_lbl.setText("<b>RICLAB</b>")
        lv.addWidget(logo_lbl)

        # toolbar dọc
        side_tb = QToolBar("Sidebar", objectName="SideTB")
        side_tb.setOrientation(Qt.Vertical)
        side_tb.setMovable(False)
        side_tb.setFloatable(False)
        side_tb.setIconSize(QSize(28, 28))
        side_tb.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        side_tb.setContentsMargins(0, 0, 0, 0)

        st = self.style()
        act_intro = QAction(left_pinned_icon(std_icon(st, QStyle.SP_FileDialogInfoView)), "Introduction", self)
        act_sensor = QAction(left_pinned_icon(std_icon(st, QStyle.SP_ComputerIcon)), "Sensor Conc", self)
        act_map   = QAction(left_pinned_icon(std_icon(st, QStyle.SP_DirIcon)), "Create Maps", self)
        act_nav   = QAction(left_pinned_icon(std_icon(st, QStyle.SP_ArrowForward)), "Navigation", self)

        side_tb.addAction(act_intro)
        side_tb.addAction(act_sensor)
        side_tb.addAction(act_map)
        side_tb.addAction(act_nav)

        act_intro.triggered.connect(lambda: self.goto_page("home"))
        act_sensor.triggered.connect(lambda: self.goto_page("sensor"))
        act_nav.triggered.connect(lambda: self.goto_page("nav"))
        act_map.triggered.connect(lambda: self.goto_page("map"))

        lv.addWidget(side_tb)
        lv.addStretch(1)

        # ================== APP BAR NỘI BỘ (nằm TRONG vùng nội dung) ==================
        self.appbar = QToolBar("AppBar")
        self.appbar.setObjectName("AppBar")
        self.appbar.setMovable(False)
        self.appbar.setIconSize(QSize(20, 20))

        # icon + chữ trang hiện tại
        self.pageIconLabel = QLabel()
        self.pageTitleLabel = QLabel()
        self.pageTitleLabel.setObjectName("AppBarTitle")
        self.appbar.addWidget(self.pageIconLabel)
        self.appbar.addWidget(self.pageTitleLabel)

        # spacer đẩy nút về bên phải
        spacer = QWidget(); spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.appbar.addWidget(spacer)

        # nút Home bên phải
        home_act = QAction(self.style().standardIcon(QStyle.SP_DirHomeIcon), "Home", self)
        home_act.triggered.connect(lambda: self.goto_page("home"))
        self.appbar.addAction(home_act)

        # ================== CỘT NỘI DUNG (appbar + stack) ==================
        content_col = QWidget()
        content_v = QVBoxLayout(content_col)
        content_v.setContentsMargins(0, 0, 0, 0)
        content_v.setSpacing(0)
        content_v.addWidget(self.appbar)   # << thanh xanh TRONG vùng nội dung
        content_v.addWidget(self.stack, 1) # << nội dung trang bên dưới

        # ================== Layout tổng: sidebar | cột nội dung ==================
        central = QWidget()
        h = QHBoxLayout(central)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(0)
        h.addWidget(left_panel)
        h.addWidget(content_col, 1)   # thay vì add self.stack trực tiếp
        self.setCentralWidget(central)

        # Status bar
        sb = QStatusBar(); sb.showMessage("Ready"); self.setStatusBar(sb)

        # ================== Style ==================
        self.setStyleSheet("""
            /* App bar xanh đậm trong vùng nội dung */
            QToolBar#AppBar {
                background-color: #0f5132;  /* xanh đậm */
                border: none;
                padding: 6px 10px;
            }
            #AppBarTitle {
                color: #ffffff;
                font-size: 16px;
                font-weight: 600;
                padding-left: 8px;
            }
            QToolBar#AppBar QToolButton { color: #ffffff; }

            /* Sidebar tối */
            QFrame#LeftPanel { background-color: #2b2f3a; color: #e6e6e6; }
            QFrame#LeftPanel QLabel { color: #e6e6e6; }
            QToolBar#SideTB { background-color: #2b2f3a; border: none; padding: 0; }
            QToolBar#SideTB QToolButton{
                color:#e6e6e6; padding:6px 10px 6px 6px; margin:4px 0;
                border-radius:6px; text-align:left; font-size:15px; min-height:40px;
            }
            QToolBar#SideTB QToolButton:hover{ background:#3a4050; }
            QToolBar#SideTB QToolButton:checked{ background:#4b5368; }

            QStatusBar { color: #666; }
        """)

        # giãn nút sidebar
        for btn in side_tb.findChildren(QToolButton):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setLayoutDirection(Qt.LeftToRight)

        # Trang mặc định + cập nhật app bar
        self.goto_page("home")

    # ======= Cập nhật nội dung + app bar =======
    def goto_page(self, key: str):
        # map keys -> (stack_index, title, icon)
        st = self.style()
        mapping = {
            "home":   (0, "Introduction", std_icon(st, QStyle.SP_FileDialogInfoView)),
            "sensor": (0, "Sensor",       std_icon(st, QStyle.SP_ComputerIcon)),   # tạm dùng Home page
            "nav":    (2, "Navigation",   std_icon(st, QStyle.SP_ArrowForward)),   # tạm dùng Home page
            "map":    (1, "Create Maps",  std_icon(st, QStyle.SP_DirIcon)),
        }
        idx, title, qic = mapping.get(key, mapping["home"])
        self.stack.setCurrentIndex(idx)
        # Cập nhật AppBar: icon + chữ
        icon = left_pinned_icon(qic, QSize(22, 22))
        self.pageIconLabel.setPixmap(icon.pixmap(22, 22))
        self.pageTitleLabel.setText(title)
        # status
        self.statusBar().showMessage(f"Page: {title}", 2000)

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
