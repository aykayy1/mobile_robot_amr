cmake_minimum_required(VERSION 3.8)
project(agv0509test6)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rplidar_ros REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Cài đặt các tài nguyên chia sẻ (không cần tồn tại hết; nếu thiếu CMake vẫn cấu hình OK)
install(
  DIRECTORY meshes/
    config
    launch
    meshes
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Nếu sau này có script Python, bỏ comment và thêm vào đây:
# install(PROGRAMS
#   scripts/my_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
