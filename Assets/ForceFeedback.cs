using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Collections.Generic;

public class FingerCollisionSender : MonoBehaviour
{
    [Header("ESP32 UDP Config")]
    public string esp32IP = "192.168.88.184";   // đổi theo IP tĩnh ESP32
    public int esp32Port = 2222;

    [Header("Ngón tay (gán nhiều collider nếu có)")]
    public Collider[] thumb;   // ngón cái
    public Collider[] index;   // ngón trỏ
    public Collider[] middle;  // ngón giữa
    public Collider[] ring;    // ngón áp út
    public Collider[] pinky;   // ngón út

    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;

    void Start()
    {
        udpClient = new UdpClient();
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(esp32IP), esp32Port);

        // gắn trigger cho từng collider trong mỗi ngón
        AddTriggers(thumb, "1", "A", "Ngón cái");
        AddTriggers(index, "2", "B", "Ngón trỏ");
        AddTriggers(middle, "3", "C", "Ngón giữa");
        AddTriggers(ring, "4", "D", "Ngón áp út");
        AddTriggers(pinky, "5", "E", "Ngón út");

        Debug.Log($"📡 FingerCollisionSender sẵn sàng gửi UDP → {esp32IP}:{esp32Port}");
    }

    private void AddTriggers(Collider[] colliders, string pressCmd, string releaseCmd, string fingerName)
    {
        if (colliders == null) return;

        foreach (var col in colliders)
        {
            if (col == null) continue;
            var trigger = col.gameObject.AddComponent<FingerTrigger>();
            trigger.Init(this, pressCmd, releaseCmd, fingerName);
        }
    }

    public void SendCommand(string command, string fingerName, string action)
    {
        try
        {
            byte[] data = Encoding.UTF8.GetBytes(command);
            udpClient.Send(data, data.Length, remoteEndPoint);
            Debug.Log($"👉 {fingerName} {action} → gửi {command}");
        }
        catch (System.Exception e)
        {
            Debug.LogError("❌ Lỗi gửi UDP: " + e.ToString());
        }
    }

    void OnApplicationQuit()
    {
        udpClient.Close();
    }

    // class phụ: trigger cho từng collider
    public class FingerTrigger : MonoBehaviour
    {
        private FingerCollisionSender manager;
        private string pressCommand;
        private string releaseCommand;
        private string fingerName;

        private int contactCount = 0; // để tránh gửi lệnh trùng khi nhiều collider cùng chạm

        public void Init(FingerCollisionSender mgr, string press, string release, string name)
        {
            manager = mgr;
            pressCommand = press;
            releaseCommand = release;
            fingerName = name;
        }

        void OnCollisionEnter(Collision collision)
        {
            contactCount++;
            if (contactCount == 1) // lần đầu tiên chạm
                manager.SendCommand(pressCommand, fingerName, $"chạm {collision.collider.name}");
        }

        void OnCollisionExit(Collision collision)
        {
            contactCount--;
            if (contactCount <= 0)
            {
                contactCount = 0;
                manager.SendCommand(releaseCommand, fingerName, $"rời {collision.collider.name}");
            }
        }
    }
}
