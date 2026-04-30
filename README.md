# Autonomous Mobile Robot (AMR) Project

Dự án Robot tự hành (Differential Drive) sử dụng ROS 2 để định vị, tạo bản đồ (SLAM) và tự động tìm đường (Navigation).

📍 **Repository:** [https://github.com/aykayy1/mobile_robot_amr](https://github.com/aykayy1/mobile_robot_amr)

## 📂 Cấu trúc dự án
Dự án được tổ chức thành các module chức năng chính:
* `/slam`: Chứa các file cấu hình và launch file cho việc tạo bản đồ (Mapping).
* `/navigation`: Chứa cấu hình Nav2 và launch file cho việc di chuyển tự động.
* `/agv0509test6`: *(Thư mục kiểm thử nội bộ - Vui lòng bỏ qua)*.

## 🛠️ Cài đặt (Installation)

### 1. Yêu cầu hệ thống
* Ubuntu 22.04 LTS
* ROS 2 Humble Hawksbill

### 2. Tải và Build
```bash```
# 1. Tạo workspace
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src

# 2. Clone repo
git clone [https://github.com/aykayy1/mobile_robot_amr.git](https://github.com/aykayy1/mobile_robot_amr.git)

# 3. Cài đặt dependencies
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build package
colcon build --symlink-install
source install/setup.bash


🚀 Hướng dẫn vận hành (Usage)
Để vận hành robot, vui lòng mở các Terminal riêng biệt cho từng tác vụ dưới đây.

📍 Giai đoạn 1: Tạo bản đồ (SLAM)
Sử dụng module trong thư mục slam để quét và xây dựng bản đồ môi trường.

Khởi động SLAM:

```bash
# Chạy file launch trong thư mục slam (thay tên file .py nếu khác)
ros2 launch amr_slam agv_runall.py
(Lệnh này sẽ khởi động Lidar và thuật toán SLAM Toolbox/Cartographer).
```
```bash
ros2 run amr_slam wheel_vel_node
(Lệnh này để chạy bàn phím điều khiển robot)
```

Lưu bản đồ (Sau khi quét xong):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/amr_ws/src/mobile_robot_amr/navigation/maps/my_map
```
📍 Giai đoạn 2: Dẫn đường (Navigation)
Sử dụng module trong thư mục navigation để robot tự chạy trên bản đồ đã lưu.

Khởi động Navigation:

```bash
# Chạy file launch trong thư mục navigation (thay tên file .py nếu khác)
ros2 launch amr_navigation bringup_localization.launch.py
ros2 launch amr_navigation navigation_real_launch.py
ros2 run amr_navigation wheel_vel_node_nav
(Hệ thống sẽ tải bản đồ my_map và khởi động Nav2).
```
Điều khiển:

Sử dụng công cụ 2D Pose Estimate trên Rviz để xác định vị trí ban đầu của robot.

Sử dụng Nav2 Goal để chọn điểm đến.

⚠️ Lưu ý (Note)
Thư mục agv0509test6 chứa các file test cũ, không sử dụng cho quy trình vận hành chuẩn.

Đảm bảo bạn đã source install/setup.bash trong mọi terminal mới mở.

👥 Liên hệ
Tác giả: [Trần Anh Khoa - Lê Đức Mạnh]

Email: [trankhoavt85@gmail.com]
       [manh1472003@gmail.com]

VIDEO DEMO 1:
<div align="center">
  <a href="https://www.youtube.com/watch?v=YFsDgfYz9JY">
    <img src="http://img.youtube.com/vi/97mRKKiSMAY/0.jpg" width="500" alt="Demo Mobile Robot">
  </a>
</div>

VIDEO DEMO 2:
<div align="center">
  <a href="https://www.youtube.com/watch?v=3QMen4Jf0rc">
    <img src="http://img.youtube.com/vi/Di32CT20gGQ/0.jpg" width="500" alt="Demo 2 _ ROS 2-Based Autonomous Mobile Robots (AMRs)">
  </a>
</div>
