using Microsoft.Networking.Mavlink;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Microsoft.Networking
{
    public class UdpPort : IPort
    {
        System.Net.Sockets.UdpClient udp;
        IPEndPoint remoteEndPoint;
        AutoResetEvent received = new AutoResetEvent(false);
        bool receivingPending;
        IPEndPoint localEndPoint;

        public IPEndPoint LocalEndPoint
        {
            get { return this.localEndPoint; }
        }

        /// <summary>
        /// Connect to any remote machine that wants to send to the given local port.
        /// </summary>
        public void Connect(IPEndPoint localEndPoint)
        {
            this.localEndPoint = localEndPoint;
            udp = new System.Net.Sockets.UdpClient(localEndPoint);
            udp.Client.ReceiveBufferSize = 1000000;
            receivingPending = true;
            udp.BeginReceive(new AsyncCallback(OnPacketReceived), this);
        }

        public static IEnumerable<NetworkInterface> GetLocalAddresses()
        {
            foreach (var network in System.Net.NetworkInformation.NetworkInterface.GetAllNetworkInterfaces())
            {
                if (network.NetworkInterfaceType != System.Net.NetworkInformation.NetworkInterfaceType.Loopback &&
                    network.OperationalStatus == System.Net.NetworkInformation.OperationalStatus.Up)
                {
                    var ipProps = network.GetIPProperties();
                    if (ipProps != null)
                    {
                        yield return network;
                    }
                }
            }
        }

        /// <summary>
        /// Connect to specific remote end point and only send and receive from that endpoint.
        /// </summary>
        /// <param name="endPoint"></param>
        public void Connect(IPEndPoint localEndPoint, IPEndPoint remoteEndPoint)
        {
            this.remoteEndPoint = remoteEndPoint;
            udp = new UdpClient(localEndPoint);
            receivingPending = true;
            udp.BeginReceive(new AsyncCallback(OnPacketReceived), this);
        }

        public IPEndPoint WaitForOneMessage(TimeSpan delay)
        {
            received.WaitOne(delay);
            return remoteEndPoint;
        }

        private void OnPacketReceived(IAsyncResult ar)
        {
            if (udp != null)
            {
                lock (udp)
                {
                    receivingPending = false;
                    IPEndPoint remoteEP = null;
                    try
                    {
                        byte[] bytes = udp.EndReceive(ar, ref remoteEP);

                        if (bytes != null && bytes.Length > 0)
                        {
                            remoteEndPoint = remoteEP;
                            lock (packets)
                            {
                                bool first = packets.Count == 0;
                                packets.Add(bytes);
                                if (first)
                                {
                                    received.Set();
                                }
                            }
                            // begin another.
                            if (udp != null)
                            {
                                receivingPending = true;
                                udp.BeginReceive(new AsyncCallback(OnPacketReceived), this);
                            }
                        }
                    }
                    catch (SocketException ex)
                    {
                        // receive is forceably closed on send, so ignore that
                        if (ex.SocketErrorCode != SocketError.ConnectionReset)
                        {
                            //throw;
                        }
                    }
                    catch (Exception)
                    {
                        // perhaps it was closed...
                    }
                }
            }
        }

        public void Write(byte[] buffer, int count)
        {
            if (udp == null)
            {
                throw new Exception("UdpPort is not connected");
            }
            lock (udp)
            {
                if (this.remoteEndPoint != null)
                {
                    udp.Send(buffer, count, remoteEndPoint);
                }
            }

            if (!receivingPending)
            {
                // then previous receive would be cancelled by this send, so start receiving again.
                receivingPending = true;
                udp.BeginReceive(new AsyncCallback(OnPacketReceived), this);
            }
        }

        public void Write(string msg)
        {
            byte[] buffer = Encoding.UTF8.GetBytes(msg);
            Write(buffer, buffer.Length);
        }

        private void OnSent(IAsyncResult ar)
        {
            if (udp != null)
            {
                int bytesSent = udp.EndSend(ar);
            }
        }

        public int Read(byte[] buffer, int bytesToRead)
        {
            if (packets.Count == 0)
            {
                received.WaitOne();
            }
            int pos = 0;
            while (pos < bytesToRead)
            {
                if (current == null)
                {
                    lock (packets)
                    {
                        current = packets[0];
                        packets.RemoveAt(0);
                        currentPos = 0;
                    }
                }
                int available = current.Length - currentPos;
                if (available > bytesToRead - pos)
                {
                    // partial
                    int toCopy = bytesToRead - pos;
                    Array.Copy(current, currentPos, buffer, pos, toCopy);
                    pos = bytesToRead;
                    currentPos += toCopy;
                }
                else
                {
                    // consume all.
                    Array.Copy(current, currentPos, buffer, pos, available);
                    pos += available;
                    current = null;
                    currentPos = 0;
                }
            }
            return pos;
        }

        public void Close()
        {
            if (udp != null)
            {
                udp.Close();
                udp = null;
            }
        }

        byte[] current;
        int currentPos;
        List<byte[]> packets = new List<byte[]>();
    }
}
