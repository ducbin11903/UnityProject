using UnityEngine;
using System;
using System.IO.Ports;
using System.Threading;

public class ESP32_com : MonoBehaviour
{
    [Tooltip("Tên cổng COM. Ví dụ: COM4 (Windows) hoặc /dev/ttyUSB0 (Linux)")]
    public string portName = "COM4";

    [Tooltip("Baudrate phải khớp với ESP32")]
    public int baudRate = 230400;

    // Biến static lưu quaternion và góc ngón tay
    public static float qw, qx, qy, qz;
    public static float serce, yuzuk, orta, isaret, bas;

    private SerialPort serialPort;
    private Thread readThread;
    private volatile bool isRunning;
    private readonly object dataLock = new object();

    void Start()
    {
        try
        {
            serialPort = new SerialPort(portName, baudRate);
            serialPort.ReadTimeout = 100; // ms
            serialPort.Open();

            isRunning = true;
            readThread = new Thread(new ThreadStart(ReadData)) { IsBackground = true };
            readThread.Start();

            Debug.Log($"✅ Serial mở tại {portName} @ {baudRate} baud");
        }
        catch (Exception ex)
        {
            Debug.LogError("❌ Lỗi khi mở Serial: " + ex.Message);
        }
    }

    private void ReadData()
    {
        // Kích thước gói tin: 9 float = 36 byte
        const int EXPECTED_PACKET_SIZE = 36;
        byte[] buffer = new byte[EXPECTED_PACKET_SIZE];

        while (isRunning)
        {
            try
            {
                int bytesRead = 0;
                while (bytesRead < EXPECTED_PACKET_SIZE)
                {
                    int b = serialPort.ReadByte();
                    if (b == -1) break; // không đọc được
                    buffer[bytesRead++] = (byte)b;
                }

                if (bytesRead == EXPECTED_PACKET_SIZE)
                {
                    lock (dataLock)
                    {
                        qw = BitConverter.ToSingle(buffer, 0);
                        qx = BitConverter.ToSingle(buffer, 4);
                        qy = BitConverter.ToSingle(buffer, 8);
                        qz = BitConverter.ToSingle(buffer, 12);

                        bas = BitConverter.ToSingle(buffer, 16); // Thumb
                        isaret = BitConverter.ToSingle(buffer, 20); // Index
                        orta = BitConverter.ToSingle(buffer, 24); // Middle
                        yuzuk = BitConverter.ToSingle(buffer, 28); // Ring
                        serce = BitConverter.ToSingle(buffer, 32); // Pinky
                    }
                }
            }
            catch (TimeoutException)
            {
                // bỏ qua timeout, thử đọc tiếp
            }
            catch (Exception ex)
            {
                if (isRunning)
                    Debug.LogError("❌ Lỗi khi đọc Serial: " + ex.Message);
            }
        }
    }

    void Update()
    {
        // Debug quaternion + ngón trỏ
        Debug.Log($"Quat W: {qw:F3} | Index Angle: {isaret:F1}");

        // 👉 Chỗ này bạn gán vào model 3D giống như code UDP cũ
        // hand.transform.rotation = new Quaternion(-qy, -qz, qx, qw);
        // index1.transform.localRotation = Quaternion.Euler(-74, 106, 61 - isaret);
    }

    void OnApplicationQuit()
    {
        isRunning = false;

        if (readThread != null && readThread.IsAlive)
        {
            readThread.Join();
        }

        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
        }

        Debug.Log("Đã đóng Serial.");
    }
}
