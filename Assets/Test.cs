using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;

public class UDPKeyTest : MonoBehaviour
{
    [Header("ESP32 UDP Config")]
    public string esp32IP = "192.168.88.184";  // Đặt đúng IP ESP32 bạn in ra Serial Monitor
    public int esp32Port = 2222;

    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;

    void Start()
    {
        // Ép IPv4
        udpClient = new UdpClient(AddressFamily.InterNetwork);
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(esp32IP), esp32Port);
        Debug.Log($"📡 UDPKeyTest sẵn sàng gửi UDP → {esp32IP}:{esp32Port}");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) SendCommand("1");
        if (Input.GetKeyDown(KeyCode.Alpha2)) SendCommand("2");
        if (Input.GetKeyDown(KeyCode.Alpha3)) SendCommand("3");
        if (Input.GetKeyDown(KeyCode.Alpha4)) SendCommand("4");
        if (Input.GetKeyDown(KeyCode.Alpha5)) SendCommand("5");

        if (Input.GetKeyDown(KeyCode.A)) SendCommand("A");
        if (Input.GetKeyDown(KeyCode.B)) SendCommand("B");
        if (Input.GetKeyDown(KeyCode.C)) SendCommand("C");
        if (Input.GetKeyDown(KeyCode.D)) SendCommand("D");
        if (Input.GetKeyDown(KeyCode.E)) SendCommand("E");
    }

    void SendCommand(string command)
    {
        try
        {
            byte[] data = Encoding.ASCII.GetBytes(command);
            udpClient.Send(data, data.Length, remoteEndPoint);
            Debug.Log($"👉 Gửi UDP: {command} tới {remoteEndPoint}");
        }
        catch (System.Exception e)
        {
            Debug.LogError("❌ Lỗi gửi UDP: " + e);
        }
    }

    void OnApplicationQuit()
    {
        udpClient.Close();
    }
}
