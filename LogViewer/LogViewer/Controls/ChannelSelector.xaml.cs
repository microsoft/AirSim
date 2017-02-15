using LogViewer.Utilities;
using Microsoft.Networking;
using Microsoft.Networking.Mavlink;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.NetworkInformation;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using static Microsoft.Networking.Mavlink.MAVLink;

namespace LogViewer.Controls
{
    /// <summary>
    /// Interaction logic for ConnectionSelector.xaml
    /// </summary>
    public partial class ChannelSelector : UserControl
    {
        MavlinkChannel channel;
        bool unloaded;
        DelayedActions delayedActions = new DelayedActions();

        ObservableCollection<LogEntryModel> logList = new ObservableCollection<LogEntryModel>();

        // downloading state

        IPort port;
        List<mavlink_log_data_t> data = new List<mavlink_log_data_t>();
        mavlink_log_entry_t selectedEntry; // downloading
        ulong totalDownloaded;
        DispatcherTimer progressTimer;
        int lastChunkTime;
        bool checking;
        List<Tuple<uint, uint>> holes = new List<Tuple<uint, uint>>();
        Tuple<uint, uint> hole;
        uint holeCount;
        bool verifying;
        bool complete;

        public event EventHandler Connected;
        public event EventHandler Disconnected;

        public MavlinkChannel Channel { get { return this.channel; } }

        public ChannelSelector()
        {
            InitializeComponent();
            this.Loaded += OnPanelLoaded;
            this.Unloaded += OnPanelUnloaded;

            SerialConnectButton.IsEnabled = false;
            SerialPorts.SelectionChanged += OnSerialPortSelected;

            LogFiles.ItemsSource = logList;

            this.Tabs.SelectedItem = SerialTab;
        }

        public void Start()
        {
            this.Visibility = Visibility.Visible;

            this.data = new List<mavlink_log_data_t>();
            this.totalDownloaded = 0;
            StopProgress();
            this.lastChunkTime = 0;
            this.checking = false;
            holes = new List<Tuple<uint, uint>>();
            this.holeCount = 0;
            this.verifying = false;
            this.complete = false;
        }

        private void OnSerialPortSelected(object sender, SelectionChangedEventArgs e)
        {
            EnableButtons();
        }

        private void EnableButtons()
        {
            if (SerialConnectButton != null)
            {
                int baudRate = 0;
                SerialConnectButton.IsEnabled = SerialPorts != null && (SerialPorts.SelectedItem != null) && int.TryParse(BaudRate.Text, out baudRate) && baudRate > 0;
            }
            if (DownloadButton != null)
            {
                DownloadButton.IsEnabled = (LogFiles.SelectedItem != null);
            }
            if (UdpConnectButton != null)
            {
                int port = 0;
                UdpConnectButton.IsEnabled = NetworkList.SelectedItem != null && (int.TryParse(PortNumber.Text, out port) && port > 0);
            }
        }

        private void OnPanelUnloaded(object sender, RoutedEventArgs e)
        {
            unloaded = true;
            //if (port != null)
            //{
            //    port.Close();
            //    port = null;
            //}
        }

        private async void OnPanelLoaded(object sender, RoutedEventArgs e)
        {
            List<SerialPort> ports = new List<SerialPort>(await SerialPort.FindPorts());
            if (!unloaded)
            {
                foreach (var port in ports)
                {
                    SerialPorts.Items.Add(port);
                }
            }

            NetworkList.Items.Add(new NetworkModel()
            {
                Network = null,
                Name = "localhost",
                Description = "Connect to an app on your local computer"
            });

            foreach (var network in UdpPort.GetLocalAddresses())
            {
                NetworkList.Items.Add(new NetworkModel()
                {
                    Network = network,
                    Name = network.Name,
                    Description = network.Description
                });
            }
            EnableButtons();
        }

        class NetworkModel
        {
            public NetworkInterface Network { get; set; }
            public string Name { get; set; }
            public string Description { get; set; }
        }


        private void OnSerialConnect(object sender, RoutedEventArgs e)
        {
            if (port != null)
            {
                port.Close();
                port = null;
                OnPortClosed();
            }
            else
            {
                OnSerialPortSelected();
            }
        }

        private void OnSerialPortSelected()
        {
            Microsoft.Networking.SerialPort port = (SerialPort)SerialPorts.SelectedItem;
            if (port == null)
            {
                return;
            }

            int baudRate = 0;
            int.TryParse(BaudRate.Text, out baudRate);

            try
            {
                port.Connect(port.Id, baudRate);
                SerialConnectButton.Content = "Disconnect";
                OnPortConnected(port);
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "Error Connecting", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        void OnPortConnected(IPort port)
        {
            this.port = port;

            port.Write("\n\n\n\nexit\r\nlogs\r\n");

            channel = new Microsoft.Networking.Mavlink.MavlinkChannel();
            channel.Start(port);
            channel.MessageReceived += OnMessageReceived;

            if (Connected != null)
            {
                Connected(this, EventArgs.Empty);
            }
        }

        bool logTabSelected;

        private void OnTabSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (this.port != null && this.Tabs.SelectedItem == LogTab && !logTabSelected)
            {
                logList.Clear();
                delayedActions.StartDelayedAction("GetList", OnGetList, TimeSpan.FromMilliseconds(100));
            }
            logTabSelected = (this.Tabs.SelectedItem == LogTab);
        }

        void OnGetList()
        {
            if (channel != null)
            {
                MavLinkMessage msg = new MavLinkMessage();
                msg.ComponentId = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
                msg.SystemId = 255;
                msg.MsgId = MAVLINK_MSG_ID.LOG_REQUEST_LIST;
                msg.TypedPayload = new mavlink_log_request_list_t()
                {
                    start = 0,
                    end = ushort.MaxValue,
                    target_system = 1,
                    target_component = 1
                };
                channel.SendMessage(msg);
                delayedActions.StartDelayedAction("GetList", OnCheckList, TimeSpan.FromMilliseconds(1000));
            }
        }

        void OnCheckList()
        {
            if (logList.Count == 0)
            {
                // try again!
                OnGetList();
            }
        }

        private void OnPortClosed()
        {
            SerialConnectButton.Content = "Connect";
            UdpConnectButton.Content = "Connect";
            channel.MessageReceived -= OnMessageReceived;
            channel.Stop();
            channel = null;

            if (Disconnected != null)
            {
                Disconnected(this, EventArgs.Empty);
            }
            logList.Clear();
        }


        private void OnMessageReceived(object sender, MavLinkMessage e)
        {
            if (e.MsgId == MAVLINK_MSG_ID.LOG_ENTRY)
            {
                if (e.TypedPayload is mavlink_log_entry_t)
                {
                    List<LogEntryModel> list = new List<LogEntryModel>();
                    mavlink_log_entry_t entry = (mavlink_log_entry_t)e.TypedPayload;
                    UiDispatcher.RunOnUIThread(() =>
                    {
                        logList.Add(new LogEntryModel(entry));
                        delayedActions.CancelDelayedAction("GetList");
                    });
                }
            }
            else if (e.MsgId == MAVLINK_MSG_ID.LOG_DATA)
            {
                if (e.TypedPayload is mavlink_log_data_t)
                {
                    var logdata = (mavlink_log_data_t)e.TypedPayload;
                    if (logdata.id == selectedEntry.id)
                    {
                        lock (data)
                        {
                            lastChunkTime = Environment.TickCount;
                            totalDownloaded += logdata.count;
                            data.Add(logdata);
                        }

                        if (holes.Count > 0)
                        {
                            FetchNextHole(logdata);
                        }
                    }
                }
            }
        }

        void StartProgress()
        {
            progressTimer = new DispatcherTimer() { Interval = TimeSpan.FromMilliseconds(30) };
            progressTimer.Tick += OnProgressTick;
            progressTimer.Start();
            DownloadProgress.Visibility = Visibility.Visible;
            DownloadProgress.IsIndeterminate = false;
            DownloadProgress.Maximum = selectedEntry.size;
        }

        void StopProgress()
        {
            if (progressTimer != null)
            {
                progressTimer.Stop();
                progressTimer.Tick -= OnProgressTick;
                progressTimer = null;
            }
        }

        private void OnProgressTick(object sender, EventArgs e)
        {
            DownloadProgress.Value = totalDownloaded;
            DownloadProgress.Maximum = Math.Max(DownloadProgress.Maximum, totalDownloaded);

            if (lastChunkTime < Environment.TickCount - 1000)
            {
                // more than a second since last chunk, let's see if we're done.
                CheckDone();
                lastChunkTime = Environment.TickCount;
            }
        }

        private void FetchNextHole(mavlink_log_data_t? downloaded)
        {
            bool next = false;
            if (downloaded == null)
            {
                next = true;
            }
            else
            {
                holeCount += downloaded.Value.count;
                if (holeCount == hole.Item2)
                {
                    // done!, get the next one.
                    next = true;
                }
            }
            if (next && holes.Count > 0)
            {
                holeCount = 0;
                hole = holes[0];
                holes.RemoveAt(0);
                Debug.WriteLine("Filling hole at {0}, size {1}", hole.Item1, hole.Item2);
                RequestLogData(selectedEntry.id, hole.Item1, hole.Item2);
                if (!verifying)
                {
                    verifying = true;
                    UiDispatcher.RunOnUIThread(() =>
                    {
                        DownloadStatus.Text = "verifying...";
                    });
                }
            }
            else
            {
                // we should be done, the CheckDone will fine out for sure after we resort the data and check for holes again...
            }
        }


        private void CheckDone()
        {
            if (checking)
            {
                return;
            }
            checking = true;
            try
            {
                bool foundHole = false;
                uint pos = 0;
                var holes = new List<Tuple<uint, uint>>();

                lock (data)
                {
                    // sort packets in order of ofs.
                    data.Sort((a, b) => { return (int)a.ofs - (int)b.ofs; });

                    foreach (var e in data.ToArray())
                    {
                        if (e.ofs < pos)
                        {
                            // duplicate?
                            data.Remove(e);
                        }
                        else if (e.ofs != pos)
                        {
                            // we have a hole!!
                            foundHole = true;
                            holes.Add(new Tuple<uint, uint>(pos, e.ofs - pos));
                            pos = e.ofs + e.count;
                        }
                        else
                        {
                            pos += e.count;
                        }
                    }
                }

                if (!foundHole)
                {
                    if (selectedEntry.size <= totalDownloaded)
                    {
                        // ok, we're done!
                        Debug.WriteLine("Log is complete!");
                        if (!complete)
                        {
                            complete = true;
                            UiDispatcher.RunOnUIThread(() =>
                            {
                                StopProgress();
                                OnSaveLog();
                            });
                        }
                    }
                }
                else
                {
                    this.holes = holes;
                    FetchNextHole(null);
                }
            }
            finally
            {
                checking = false;
            }
        }

        private void OnSaveLog()
        {
            Microsoft.Win32.SaveFileDialog fo = new Microsoft.Win32.SaveFileDialog();
            fo.Filter = "bin files (*.bin)|*.bin";
            fo.CheckPathExists = true;

            DateTime logTime = GetLogTime(selectedEntry);
            fo.FileName = "Log" + selectedEntry.id + " " + logTime.ToString("yyyy-MM-dd hh-mm") + ".bin";
            if (fo.ShowDialog() == true && !string.IsNullOrEmpty(fo.FileName))
            {
                using (var stream = new FileStream(fo.FileName, FileMode.Create))
                {
                    SaveLog(stream);
                }

                OnDownloadComplete(fo.FileName);
            }
        }

        private void OnDownloadComplete(string fileName)
        {
            if (DownloadCompleted != null)
            {
                DownloadCompleted(this, fileName);
            }
        }

        public event EventHandler<string> DownloadCompleted;

        private void SaveLog(FileStream stream)
        {
            lock (data)
            {
                // sort packets in order of ofs.
                foreach (var entry in data)
                {
                    stream.Write(entry.data, 0, entry.count);
                }
            }
        }

        private void OnBaudRateChanged(object sender, TextChangedEventArgs e)
        {
            EnableButtons();
        }

        private void OnDownloadClick(object sender, RoutedEventArgs e)
        {
            var entry = (LogEntryModel)LogFiles.SelectedItem;
            if (entry != null)
            {
                selectedEntry = entry.Entry;
                RequestLogData(entry.Entry.id, 0, uint.MaxValue);
                StartProgress();
                DownloadStatus.Text = "downloading...";
            }
        }

        private void RequestLogData(ushort logFileId, uint ofs, uint count)
        {
            MavLinkMessage msg = new MavLinkMessage();
            msg.ComponentId = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
            msg.SystemId = 255;
            msg.MsgId = MAVLINK_MSG_ID.LOG_REQUEST_DATA;
            msg.TypedPayload = new mavlink_log_request_data_t()
            {
                id = logFileId,
                ofs = ofs,
                count = count,
                target_system = 1,
                target_component = 1
            };
            channel.SendMessage(msg);
        }

        class LogEntryModel
        {
            mavlink_log_entry_t entry;

            public LogEntryModel(mavlink_log_entry_t entry)
            {
                this.entry = entry;
            }

            public mavlink_log_entry_t Entry { get { return entry; } }

            public override string ToString()
            {
                return string.Format("{0} {1} (Size {2})", entry.id, GetLogTime(entry).ToShortDateString(), entry.size);
            }
        }

        private static DateTime GetLogTime(mavlink_log_entry_t entry)
        {
            return new DateTime(1970, 1, 1).AddSeconds(entry.time_utc).ToLocalTime();
        }

        private void OnLogFileSelected(object sender, SelectionChangedEventArgs e)
        {
            EnableButtons();
        }

        public int DefaultUdpPort
        {
            get
            {
                int i = 0;
                int.TryParse(PortNumber.Text, out i);
                return i;
            }
            set
            {
                PortNumber.Text = value.ToString();
            }
        }

        private void OnUdpConnect(object sender, RoutedEventArgs e)
        {
            if (port != null)
            {
                port.Close();
                port = null;
                OnPortClosed();
            }
            else
            {
                var model = (NetworkModel)NetworkList.SelectedItem;
                IPAddress addr = null;
                if (model.Network != null)
                {
                    foreach (UnicastIPAddressInformation ip in model.Network.GetIPProperties().UnicastAddresses)
                    {
                        if (ip.Address.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                        {
                            addr = ip.Address;
                        }
                    }
                }
                else
                {
                    addr = IPAddress.Parse("127.0.0.1");
                }

                int i = 0;
                if (int.TryParse(PortNumber.Text, out i))
                {
                    try
                    {
                        var udp = new UdpPort();
                        udp.Connect(new IPEndPoint(addr, i), null);
                        OnPortConnected(udp);
                        UdpConnectButton.Content = "Disconnect";
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show(ex.Message, "Error Connectin", MessageBoxButton.OK, MessageBoxImage.Error);
                    }
                }
            }
        }

        private void OnPortNumberChanged(object sender, TextChangedEventArgs e)
        {
            EnableButtons();
        }

        private void OnNetworkSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            EnableButtons();
        }

        private void OnCloseClicked(object sender, RoutedEventArgs e)
        {
            this.Visibility = Visibility.Collapsed;
        }

    }
}
