import socket
import struct

# UDP config (phải trùng với Arm.cpp)
UDP_IP = "192.168.88.132"    # nhận trên mọi địa chỉ
UDP_PORT = 1111       # cổng giống udpPortIMU

# Tạo socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP {UDP_IP}:{UDP_PORT} ...")

# Định nghĩa format unpack: 9 float (w,x,y,z + 5 ngón)
# "f" = float 4 bytes, "<" = little endian (ESP32 mặc định little endian)
packet_format = "<9f"   # 9 floats = 36 bytes

while True:
    data, addr = sock.recvfrom(1024)  # nhận tối đa 1024 bytes
    if len(data) == 36:  # đúng 36 bytes
        packet = struct.unpack(packet_format, data)
        w, x, y, z, thumb, index, middle, ring, pinky = packet

        print(f"{w:.3f}, {x:.3f}, {y:.3f}, {z:.3f}")
        print(f"{thumb:.1f}, {index:.1f}, "
              f"{middle:.1f}, {ring:.1f}, {pinky:.1f}\n")
    else:
        print(f"⚠️ Wrong packet size: {len(data)} bytes")
