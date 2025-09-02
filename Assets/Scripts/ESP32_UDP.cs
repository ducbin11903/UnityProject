using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class ESP32_UDP : MonoBehaviour
{
    // --- Cấu hình có thể chỉnh sửa trong Unity Editor ---
    [Tooltip("Cổng UDP để lắng nghe. Phải khớp với cổng gửi của ESP.")]
    public int udpPort = 1111;

    // --- Biến lưu trữ dữ liệu ---
    // Giữ nguyên tên biến static của bạn để logic animation cũ vẫn hoạt động
    public static float qw, qx, qy, qz; // Quaternion
    public static float serce, yuzuk, orta, isaret, bas; // 5 ngón tay (Pinky, Ring, Middle, Index, Thumb)

    // --- Biến cho thread và UDP client ---
    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool isRunning;

    // Đối tượng khóa để đảm bảo an toàn khi cập nhật và đọc biến từ các thread khác nhau
    private readonly object dataLock = new object();

    void Start()
    {
        // Khởi tạo và bắt đầu thread nhận UDP
        try
        {
            udpClient = new UdpClient(udpPort);
            isRunning = true;
            receiveThread = new Thread(new ThreadStart(ReceiveData)) { IsBackground = true };
            receiveThread.Start();
            Debug.Log($"✅ Bắt đầu lắng nghe UDP trên cổng {udpPort}");
        }
        catch (Exception ex)
        {
            Debug.LogError("Lỗi khi khởi tạo UDP server: " + ex.Message);
        }
    }

    private void ReceiveData()
    {
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
        // Kích thước mong đợi của gói tin: 9 giá trị float * 4 byte/float = 36 bytes
        const int EXPECTED_PACKET_SIZE = 36;

        while (isRunning)
        {
            try
            {
                // Chờ và nhận dữ liệu từ ESP
                byte[] data = udpClient.Receive(ref anyIP);

                // Kiểm tra xem gói tin nhận được có đúng 36 byte không
                if (data.Length != EXPECTED_PACKET_SIZE)
                {
                    Debug.LogWarning($"Nhận được gói tin có kích thước không hợp lệ: {data.Length} bytes. Cần {EXPECTED_PACKET_SIZE} bytes.");
                    continue; // Bỏ qua gói tin này và chờ gói tiếp theo
                }

                // Dùng lock để cập nhật các biến static một cách an toàn
                lock (dataLock)
                {
                    // Sử dụng BitConverter để chuyển đổi mảng byte thành các giá trị float.
                    // BitConverter hoạt động theo chuẩn IEEE 754.
                    // Cú pháp: BitConverter.ToSingle(mảng_byte, vị_trí_byte_bắt_đầu)

                    // Phân tích quaternion (16 byte đầu tiên)
                    qw = BitConverter.ToSingle(data, 0);  // Bytes 0-3
                    qx = BitConverter.ToSingle(data, 4);  // Bytes 4-7
                    qy = BitConverter.ToSingle(data, 8);  // Bytes 8-11
                    qz = BitConverter.ToSingle(data, 12); // Bytes 12-15

                    // Phân tích dữ liệu 5 ngón tay (20 byte tiếp theo)
                    // Thứ tự này phải khớp với thứ tự trong struct của code ESP:
                    // thumb, index, middle, ring, pinky
                    bas = BitConverter.ToSingle(data, 16);    // Ngón cái (Thumb)
                    isaret = BitConverter.ToSingle(data, 20); // Ngón trỏ (Index)
                    orta = BitConverter.ToSingle(data, 24);   // Ngón giữa (Middle)
                    yuzuk = BitConverter.ToSingle(data, 28);  // Ngón áp út (Ring)
                    serce = BitConverter.ToSingle(data, 32);  // Ngón út (Pinky)
                }
            }
            catch (ObjectDisposedException)
            {
                break; // Thoát vòng lặp khi UdpClient được đóng
            }
            catch (Exception ex)
            {
                if (isRunning)
                {
                    Debug.LogError("Lỗi khi nhận dữ liệu UDP: " + ex.Message);
                }
            }
        }
    }

    void Update()
    {
        // Hàm Update bây giờ rất gọn gàng và không bị block.
        // Bạn có thể đặt logic điều khiển/animation của mình ở đây.
        // Các biến qw, qx, isaret... đã được cập nhật ở thread nền.

        // In ra để debug (bạn có thể comment dòng này đi nếu không cần)
        Debug.Log($"Quat W: {qw:F3} | Index Angle: {isaret:F1}");

        // BỎ COMMENT PHẦN NÀY VÀ GÁN CÁC BIẾN TRANSFORM ĐỂ ĐIỀU KHIỂN MODEL CỦA BẠN
        /*
        // El Hareketi (Chuyển động bàn tay): 
        // Lưu ý: Phép chuyển đổi hệ tọa độ có thể cần điều chỉnh (ví dụ: new Quaternion(x, z, -y, w))
        hand.transform.rotation = new Quaternion(-qy, -qz, qx, qw);
        
        // Parmak Hareketleri (Chuyển động ngón tay):
        index1.transform.localRotation = Quaternion.Euler(-74, 106, 61 - isaret);
        // ... (phần còn lại của logic animation) ...
        */
    }

    void OnApplicationQuit()
    {
        // Dọn dẹp tài nguyên khi thoát ứng dụng một cách an toàn
        isRunning = false;

        if (udpClient != null)
        {
            udpClient.Close();
        }

        if (receiveThread != null && receiveThread.IsAlive)
        {
            // Chờ thread kết thúc
            receiveThread.Join();
        }
        Debug.Log("Đã đóng kết nối UDP.");
    }
}