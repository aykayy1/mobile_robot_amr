#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyUSB1"

# Lệnh cấu hình WT901C
CMD_SET_BAUD_115200 = bytes([0xFF, 0xAA, 0x04, 0x08, 0x00])  # baud = 115200
CMD_SET_6_AXIS      = bytes([0xFF, 0xAA, 0x02, 0x00, 0x00])  # tắt output góc (0x53)


def send_cmd(ser, cmd, delay=0.1):
    ser.write(cmd)
    ser.flush()
    time.sleep(delay)


def main():
    print(">>> Mở IMU @ 9600 để gửi lệnh cấu hình...")
    ser = serial.Serial(PORT, 9600, timeout=0.2)

    print(">>> Ép IMU chỉ còn 6-axis...")
    send_cmd(ser, CMD_SET_6_AXIS)

    print(">>> Ép IMU chuyển sang 115200 baud...")
    send_cmd(ser, CMD_SET_BAUD_115200)

    ser.close()
    print(">>> HOÀN TẤT. Tắt nguồn IMU rồi bật lại.")

    time.sleep(1)

    print(">>> Kiểm tra IMU @ 115200...")
    try:
        ser2 = serial.Serial(PORT, 115200, timeout=0.2)
        print(">>> OK! IMU đã chạy ở 115200.")
        ser2.close()
    except:
        print("!!! KHÔNG đọc được IMU @115200 → IMU có thể không lưu được cấu hình.")


if __name__ == "__main__":
    main()
