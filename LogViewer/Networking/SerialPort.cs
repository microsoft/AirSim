using System;
using Microsoft.Networking.Mavlink;
using System.Threading.Tasks;
using System.Threading;
using System.Management;
using System.Collections.Generic;
using System.Text;

namespace Microsoft.Networking
{
    public class SerialPort : IPort
    {
        System.IO.Ports.SerialPort port;
        
        public void Connect(string portName, int baudRate)
        {
            Close();
            port = new System.IO.Ports.SerialPort();
            port.PortName = portName;
            port.BaudRate = baudRate;
            port.Open();
            // initialize mavlink.
            port.Write("sh /etc/init.d/rc.usb\n");
        }

        public string Name { get; set; }

        public string Id { get; set; }

        public void Write(byte[] buffer, int count)
        {
            port.Write(buffer, 0, count);
        }

        public void Write(string msg)
        {
            byte[] buffer = Encoding.UTF8.GetBytes(msg);
            Write(buffer, buffer.Length);
        }

        public int Read(byte[] buffer, int bytesToRead)
        {
            if (port == null)
            {
                return 0;
            }
            return port.Read(buffer, 0, bytesToRead);
        }

        public override string ToString()
        {
            return this.Name;
        }

        public static async Task<IEnumerable<SerialPort>> FindPorts()
        {
            List<SerialPort> ports = new List<Networking.SerialPort>();
            await Task.Run(() =>
            {
                ObjectQuery query = new ObjectQuery("SELECT * FROM Win32_SerialPort");                // Win32_USBControllerDevice
                using (ManagementObjectSearcher searcher = new ManagementObjectSearcher(query))
                {
                    foreach (ManagementObject obj2 in searcher.Get())
                    {
                        //DeviceID       
                        string id = obj2.Properties["DeviceID"].Value.ToString();
                        string name = obj2.Properties["Name"].Value.ToString();
                        ports.Add(new SerialPort() { Id = id, Name = name });
                    }
                }
            });
            return ports;
        }

        public void Close()
        {
            if (port != null)
            {
                port.Close();
                port = null;
            }
        }

    }
}