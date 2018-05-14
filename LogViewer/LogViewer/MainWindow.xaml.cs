using LogViewer.Controls;
using LogViewer.Model;
using LogViewer.Model.ULog;
using LogViewer.Utilities;
using Microsoft.Maps.MapControl.WPF;
using Microsoft.Networking.Mavlink;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Xml.Linq;

namespace LogViewer
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        const double MaxChartHeight = 400;
        ProgressUtility progress;
        List<IDataLog> logs = new List<IDataLog>();
        ObservableCollection<Flight> allFlights = new ObservableCollection<Flight>();
        DelayedActions delayedActions = new DelayedActions();
        Quaternion initialAttitude;
        MapPolyline currentFlight;
        MavlinkLog currentFlightLog;
        long lastAttitudeMessage;
        List<LogEntry> mappedLogEntries;
        MapLayer annotationLayer;

        public MainWindow()
        {
            InitializeComponent();

            UiDispatcher.Initialize();

            this.progress = new ProgressUtility(MyProgress);
            MyProgress.Visibility = Visibility.Collapsed;
            UpdateButtons();
            FlightView.ItemsSource = allFlights;
            ConnectionPanel.DownloadCompleted += OnLogDownloadComplete;
            ConnectionPanel.Connected += OnChannelConnected;
            ConnectionPanel.Disconnected += OnChannelDisconnected;
            this.Visibility = Visibility.Hidden;
            RestoreSettings();
            this.SizeChanged += OnWindowSizeChanged;
            this.LocationChanged += OnWindowLocationChanged;
            ChartStack.Visibility = Visibility.Collapsed;
            ChartStack.ZoomChanged += OnZoomChanged;
            initialAttitude = ModelViewer.ModelAttitude;
            CameraPanel.Visibility = Visibility.Collapsed;
            SystemConsole.Visibility = Visibility.Collapsed;
        }

        private void OnZoomChanged(object sender, EventArgs e)
        {
            double total = 0;
            double count = 0;
            foreach (var chart in ChartStack.FindCharts())
            {
                total += chart.GetVisibleCount();
                count++;
            }
            ShowStatus(string.Format("zoom shwowing {0} data values", (int)(total / count)));
        }

        private void OnWindowLocationChanged(object sender, EventArgs e)
        {
            delayedActions.StartDelayedAction("SaveWindowLocation", SavePosition, TimeSpan.FromMilliseconds(1000));
        }

        private void OnWindowSizeChanged(object sender, SizeChangedEventArgs e)
        {
            delayedActions.StartDelayedAction("SaveWindowLocation", SavePosition, TimeSpan.FromMilliseconds(1000));
        }

        private async void RestoreSettings()
        {
            Settings settings = await ((App)App.Current).LoadSettings();
            if (settings.WindowLocation.X != 0 && settings.WindowSize.Width != 0 && settings.WindowSize.Height != 0)
            {
                // make sure it is visible on the user's current screen configuration.
                var bounds = new System.Drawing.Rectangle(
                    XamlExtensions.ConvertFromDeviceIndependentPixels(settings.WindowLocation.X),
                    XamlExtensions.ConvertFromDeviceIndependentPixels(settings.WindowLocation.Y),
                    XamlExtensions.ConvertFromDeviceIndependentPixels(settings.WindowSize.Width),
                    XamlExtensions.ConvertFromDeviceIndependentPixels(settings.WindowSize.Height));
                var screen = System.Windows.Forms.Screen.FromRectangle(bounds);
                bounds.Intersect(screen.WorkingArea);

                this.Left = XamlExtensions.ConvertToDeviceIndependentPixels(bounds.X);
                this.Top = XamlExtensions.ConvertToDeviceIndependentPixels(bounds.Y);
                this.Width = XamlExtensions.ConvertToDeviceIndependentPixels(bounds.Width);
                this.Height = XamlExtensions.ConvertToDeviceIndependentPixels(bounds.Height);
            }
            ConnectionPanel.DefaultUdpPort = settings.Port;
            this.Visibility = Visibility.Visible;
        }

        async void SavePosition()
        {
            var bounds = this.RestoreBounds;

            Settings settings = await ((App)App.Current).LoadSettings();
            settings.WindowLocation = bounds.TopLeft;
            settings.WindowSize = bounds.Size;
            await settings.SaveAsync();
        }

        private async void OnChannelConnected(object sender, EventArgs e)
        {
            ConnectorControl.Connected = true;

            QuadButton.IsChecked = true;

            var channel = ConnectionPanel.Channel;
            channel.MessageReceived += OnMavlinkMessageReceived;

            SystemConsole.Channel = channel;

            Settings settings = await ((App)App.Current).LoadSettings();
            settings.Port = ConnectionPanel.DefaultUdpPort;
            await settings.SaveAsync();

        }

        private void OnMavlinkMessageReceived(object sender, MavLinkMessage e)
        {
            if (currentFlightLog != null && !pauseRecording)
            {
                currentFlightLog.AddMessage(e);
            }

            switch (e.MsgId)
            {
                case MAVLink.MAVLINK_MSG_ID.ATTITUDE_QUATERNION:
                    {
                        // Only do this if drone is not sending MAVLINK_MSG_ID.ATTITUDE...
                        if (Environment.TickCount - lastAttitudeMessage > 1000)
                        {                            
                            var payload = (MAVLink.mavlink_attitude_quaternion_t)e.TypedPayload;
                            var q = new System.Windows.Media.Media3D.Quaternion(payload.q1, payload.q2, payload.q3, payload.q4);
                            UiDispatcher.RunOnUIThread(() =>
                            {
                            ModelViewer.ModelAttitude = initialAttitude * q;
                        });
                        }
                        break;
                    }

                case MAVLink.MAVLINK_MSG_ID.ATTITUDE:
                    {
                        lastAttitudeMessage = Environment.TickCount;
                        var payload = (MAVLink.mavlink_attitude_t)e.TypedPayload;
                        Quaternion y = new Quaternion(new Vector3D(0, 0, 1), -payload.yaw * 180 / Math.PI);
                        Quaternion x = new Quaternion(new Vector3D(1, 0, 0), payload.pitch * 180 / Math.PI);
                        Quaternion z = new Quaternion(new Vector3D(0, 1, 0), payload.roll * 180 / Math.PI);
                        UiDispatcher.RunOnUIThread(() =>
                        {
                            ModelViewer.ModelAttitude = initialAttitude * (y * x * z);
                        });
                        break;
                    }

                case MAVLink.MAVLINK_MSG_ID.HIL_STATE_QUATERNION:
                    {
                        var payload = (MAVLink.mavlink_hil_state_quaternion_t)e.TypedPayload;
                        Quaternion q = new Quaternion(payload.attitude_quaternion[0],
                            payload.attitude_quaternion[1],
                            payload.attitude_quaternion[2],
                            payload.attitude_quaternion[3]);
                        UiDispatcher.RunOnUIThread(() =>
                        {
                            ModelViewer.ModelAttitude = initialAttitude * q;
                        });
                        break;
                    }
                case MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT:
                    {
                        var payload = (MAVLink.mavlink_global_position_int_t)e.TypedPayload;
                        UiDispatcher.RunOnUIThread(() =>
                        {
                            MapLocation((double)payload.lat / 1e7, (double)payload.lon / 1e7);
                        });
                        break;
                    }
                case MAVLink.MAVLINK_MSG_ID.SERIAL_CONTROL:
                    {
                        var ctrl = (MAVLink.mavlink_serial_control_t)e.TypedPayload;
                        if (ctrl.count > 0)
                        {
                            string text = System.Text.Encoding.ASCII.GetString(ctrl.data, 0, ctrl.count);
                            UiDispatcher.RunOnUIThread(() =>
                            {
                                SystemConsole.Write(text);
                            });
                        }
                        break;
                    }
                case MAVLink.MAVLINK_MSG_ID.DATA_TRANSMISSION_HANDSHAKE:
                    if (showImageStream)
                    {
                        var p = (MAVLink.mavlink_data_transmission_handshake_t)e.TypedPayload;
                        incoming_image.size = p.size;
                        incoming_image.packets = p.packets;
                        incoming_image.payload = p.payload;
                        incoming_image.quality = p.jpg_quality;
                        incoming_image.type = p.type;
                        incoming_image.width = p.width;
                        incoming_image.height = p.height;
                        incoming_image.start = Environment.TickCount;
                        incoming_image.packetsArrived = 0;
                        incoming_image.data = new byte[incoming_image.size];
                    }
                    break;
                case MAVLink.MAVLINK_MSG_ID.ENCAPSULATED_DATA:
                    if (showImageStream)
                    {
                        var img = (MAVLink.mavlink_encapsulated_data_t)e.TypedPayload;

                        int seq = img.seqnr;
                        uint pos = (uint)seq * (uint)incoming_image.payload;

                        // Check if we have a valid transaction
                        if (incoming_image.packets == 0 || incoming_image.size == 0)
                        {
                            // not expecting an image?
                            incoming_image.packetsArrived = 0;
                            break;
                        }

                        uint available = (uint)incoming_image.payload;
                        if (pos + available > incoming_image.size)
                        {
                            available = incoming_image.size - pos;
                        }
                        Array.Copy(img.data, 0, incoming_image.data, pos, available);

                        progress.ShowProgress(0, incoming_image.size, pos + available);

                        ++incoming_image.packetsArrived;
                        //Debug.WriteLine("packet {0} of {1}, position {2} of {3}", incoming_image.packetsArrived, incoming_image.packets,
                        //    pos + available, incoming_image.size);

                        // emit signal if all packets arrived
                        if (pos + available >= incoming_image.size)
                        {
                            // Restart state machine
                            incoming_image.packets = 0;
                            incoming_image.packetsArrived = 0;
                            byte[] saved = incoming_image.data;
                            incoming_image.data = null;

                            UiDispatcher.RunOnUIThread(() =>
                            {
                                progress.ShowProgress(0, 0, 0);
                                ShowImage(saved);
                            });
                        }
                    }
                    break;
            }
        }

        private void ShowImage(byte[] image)
        {
            try
            {
                MemoryStream ms = new MemoryStream(image);
                BitmapDecoder decoder = null;
                MAVLink.MAVLINK_DATA_STREAM_TYPE type = (MAVLink.MAVLINK_DATA_STREAM_TYPE)incoming_image.type;
                switch (type)
                {
                    case MAVLink.MAVLINK_DATA_STREAM_TYPE.MAVLINK_DATA_STREAM_IMG_JPEG:
                        decoder = JpegBitmapDecoder.Create(ms, BitmapCreateOptions.IgnoreImageCache, BitmapCacheOption.None);
                        break;
                    case MAVLink.MAVLINK_DATA_STREAM_TYPE.MAVLINK_DATA_STREAM_IMG_BMP:
                        decoder = BitmapDecoder.Create(ms, BitmapCreateOptions.IgnoreImageCache, BitmapCacheOption.None);
                        break;
                    case MAVLink.MAVLINK_DATA_STREAM_TYPE.MAVLINK_DATA_STREAM_IMG_RAW8U:
                        var raw = Raw8UBitmapDecoder.Create(image, incoming_image.width, incoming_image.height);
                        BitmapFrame frame = raw.Frames[0];
                        ImageViewer.Source = frame;
                        break;
                    case MAVLink.MAVLINK_DATA_STREAM_TYPE.MAVLINK_DATA_STREAM_IMG_RAW32U:
                        //decoder = Raw8UBitmapDecoder.Create(ms, incoming_image.width, incoming_image.height);
                        break;
                    case MAVLink.MAVLINK_DATA_STREAM_TYPE.MAVLINK_DATA_STREAM_IMG_PGM:
                        //decoder = PgmBitmapDecoder.Create(ms, BitmapCreateOptions.IgnoreImageCache, BitmapCacheOption.None);
                        break;
                    case MAVLink.MAVLINK_DATA_STREAM_TYPE.MAVLINK_DATA_STREAM_IMG_PNG:
                        decoder = PngBitmapDecoder.Create(ms, BitmapCreateOptions.IgnoreImageCache, BitmapCacheOption.None);
                        break;
                }
                if (decoder != null && decoder.Frames.Count > 0)
                {
                    BitmapFrame frame = decoder.Frames[0];
                    ImageViewer.Source = frame;
                }
            }
            catch
            {

            }
        }

        private void OnChannelDisconnected(object sender, EventArgs e)
        {
            ConnectorControl.Connected = false;
            SystemConsole.Channel = null;
        }

        private void OnLogDownloadComplete(object sender, string fileName)
        {
            ConnectionPanel.Visibility = Visibility.Collapsed;
            Task.Run(async () => { await LoadBinaryFile(fileName); });
        }


        private async void OnOpenFile(object sender, RoutedEventArgs e)
        {
            OpenButton.IsEnabled = false;

            Microsoft.Win32.OpenFileDialog fo = new Microsoft.Win32.OpenFileDialog();
            fo.Filter = "PX4 Log Files (*.px4log)|*.px4log|PX4 ulog Files (*.ulg)|*.ulg|CSV Files (*.csv)|*.csv|bin files (*.bin)|*.bin|mavlink files (*.mavlink)|*.mavlink|JSON files (*.json)|*.json|KML files (*.kml)|*.kml";
            fo.CheckFileExists = true;
            fo.Multiselect = true;
            if (fo.ShowDialog() == true)
            {
                SystemConsole.Show();
                foreach (var file in fo.FileNames)
                {
                    switch (System.IO.Path.GetExtension(file).ToLowerInvariant())
                    {
                        case ".csv":
                            await Task.Run(async () => { await LoadCsvFile(file); });
                            break;
                        case ".bin":
                        case ".px4log":
                            await Task.Run(async () => { await LoadBinaryFile(file); });
                            break;
                        case ".ulg":
                        case ".ulog":
                            await Task.Run(async () => { await LoadULogFile(file); });
                            break;
                        case ".mavlink":
                            await Task.Run(async () => { await LoadMavlinkFile(file); });
                            break;
                        case ".json":
                            await Task.Run(async () => { await LoadJSonFile(file); });
                            break;
                        case ".kml":
                            await Task.Run(async () => { await LoadKmlFile(file); });
                            break;
                        default:
                            MessageBox.Show("Do not know how to read files of type : " + System.IO.Path.GetExtension(file),
                                "Unsupported file extension", MessageBoxButton.OK, MessageBoxImage.Exclamation);
                            break;
                    }
                    UpdateTitle(System.IO.Path.GetFileName(file));
                }
                ShowTotalFlightTime();
            }
            OpenButton.IsEnabled = true;
        }

        string originalTitle;

        private void UpdateTitle(string caption)
        {
            if (originalTitle == null)
            {
                originalTitle = this.Title;
            }
            if (string.IsNullOrEmpty(caption))
            {
                this.Title = originalTitle;
            }
            else
            {
                this.Title = originalTitle + " - " + caption;
            }
        }

        private async Task LoadJSonFile(string file)
        {
            try
            {
                UiDispatcher.RunOnUIThread(() =>
                {
                    SystemConsole.Show();
                });
                AppendMessage("Loading " + file);
                ShowStatus("Loading " + file);

                JSonDataLog data = new JSonDataLog();
                await data.Load(file, progress);

                //logs.Add(data);
                ShowSchema();

                LoadFlights(data);

            }
            catch (Exception ex)
            {
                AppendMessage("### Error loading json file: " + ex.Message);
            }
            ShowStatus("Done Loading " + file);
            UpdateButtons();
        }

        private async Task LoadKmlFile(string file)
        {
            try
            {
                await Dispatcher.BeginInvoke(new Action(() =>
                {
                    SystemConsole.Show();
                })).Task;

                AppendMessage("Loading " + file);
                ShowStatus("Loading " + file);

                XDocument doc = XDocument.Load(file);
                KmlDataLog data = new Model.KmlDataLog();
                data.Load(doc);

                ShowSchema();

                LoadFlights(data);
                this.logs.Add(data);
            }
            catch (Exception ex)
            {
                AppendMessage("### Error loading KML file: " + ex.Message);
            }
            ShowStatus("Done Loading " + file);
            UpdateButtons();
        }

        private async Task LoadULogFile(string file)
        {
            try
            {
                UiDispatcher.RunOnUIThread(() =>
                {
                    SystemConsole.Show();
                });
                AppendMessage("Loading " + file);
                ShowStatus("Loading " + file);

                Px4ULog data = new Px4ULog();
                await data.Load(file, progress);

                logs.Add(data);
                ShowSchema();
                LoadFlights(data);

                // remember successfully loaded log file.
                Settings settings = await ((App)App.Current).LoadSettings();
                settings.LastLogFile = file;
                await settings.SaveAsync();
            }
            catch (Exception ex)
            {
                AppendMessage("### Error loading log: " + ex.Message);
            }
            ShowStatus("Done Loading " + file);
            UpdateButtons();
        }

        private async Task LoadBinaryFile(string file)
        {
            try
            {
                UiDispatcher.RunOnUIThread(() =>
                {
                    SystemConsole.Show();
                });
                AppendMessage("Loading " + file);
                ShowStatus("Loading " + file);

                Px4DataLog data = new Px4DataLog();
                await data.Load(file, progress);

                logs.Add(data);
                ShowSchema();
                LoadFlights(data);

                // remember successfully loaded log file.
                Settings settings = await ((App)App.Current).LoadSettings();
                settings.LastLogFile = file;
                await settings.SaveAsync();
            }
            catch (Exception ex)
            {
                AppendMessage("### Error loading log: " + ex.Message);
            }
            ShowStatus("Done Loading " + file);
            UpdateButtons();
        }

        private void LoadFlights(IDataLog data)
        {
            UiDispatcher.RunOnUIThread(() =>
            {
                // add flights 
                Flight entireLog = new Flight()
                {
                    Name = "Log " + logs.Count,
                    StartTime = DateTime.MinValue,
                    Duration = TimeSpan.MaxValue,
                    Log = data
                };
                allFlights.Add(entireLog);

                foreach (var flight in data.GetFlights())
                {
                    flight.Name = "Flight " + allFlights.Count;
                    allFlights.Add(flight);
                    AppendMessage("Motor started at {0} and ran for {1} ", flight.StartTime, flight.Duration);
                }

                if (myMap.Visibility == Visibility.Visible)
                {
                    ShowMap();
                }
                
            });

        }

        private void ShowTotalFlightTime()
        {
            TimeSpan total = new TimeSpan();
            foreach (Flight f in allFlights)
            {
                if (f.Name.StartsWith("Flight"))
                {
                    total += f.Duration;
                }
            }

            AppendMessage("Total flight time {0} ", total);
        }

        private void AppendMessage(string message, params object[] args)
        {
            UiDispatcher.RunOnUIThread(() =>
            {
                if (args == null || args.Length == 0)
                {
                    SystemConsole.Write(message + "\n");
                }
                else
                {
                    SystemConsole.Write(string.Format(message, args) + "\n");
                }
            });
        }

        private async Task LoadMavlinkFile(string file)
        {
            try
            {
                UiDispatcher.RunOnUIThread(() =>
                {
                    SystemConsole.Show();
                });
                AppendMessage("Loading " + file);
                ShowStatus("Loading " + file);

                MavlinkLog data = new MavlinkLog();
                await data.Load(file, progress);

                logs.Add(data);
                ShowSchema();

                Debug.WriteLine(data.StartTime.ToString());
                LoadFlights(data);

                // remember successfully loaded log file.
                Settings settings = await ((App)App.Current).LoadSettings();
                settings.LastLogFile = file;
                await settings.SaveAsync();
            }
            catch (Exception ex)
            {
                AppendMessage("### Error loading log: " + ex.Message);
            }
            ShowStatus("Done Loading " + file);
            UpdateButtons();
        }
        
        private async Task LoadCsvFile(string file)
        {
            try
            {
                ShowStatus("Loading " + file);

                CsvDataLog log = new CsvDataLog();
                await log.Load(file, progress);

                UiDispatcher.RunOnUIThread(() =>
                {
                    FlightView.ItemsSource = new List<Flight>();
                    logs.Add(log);
                    ShowSchema();
                });

            }
            catch (Exception ex)
            {
                ShowStatus(ex.Message);
            }
            ShowStatus("Done");
        }

        private void ShowSchema()
        {
            UiDispatcher.RunOnUIThread(() =>
            {
                LogItemSchema schema = null;

                // todo: compute combined schema for selected logs, but for now just show the first one.
                if (this.currentFlightLog != null)
                {
                    schema = currentFlightLog.Schema;
                }
                foreach (var log in this.logs)
                {
                    var s = log.Schema;
                    if (schema == null)
                    {
                        schema = s;
                    }
                    else
                    {
                        schema.Combine(s);
                    }
                }
                if (schema == null || schema.ChildItems == null || schema.ChildItems.Count == 0)
                {
                    CategoryList.ItemsSource = null;
                }
                else 
                {
                    List<LogItemSchema> list = new List<Model.LogItemSchema>(schema.ChildItems);
                    list.Sort((a, b) => { return string.Compare(a.Name, b.Name, StringComparison.OrdinalIgnoreCase); });
                    CategoryList.ItemsSource = list;
                }
            });
        }

        private void ShowStatus(string message)
        {
            UiDispatcher.RunOnUIThread(() =>
            {
                StatusText.Text = message;
            });
        }

        private void OnListItemSelected(object sender, SelectionChangedEventArgs e)
        {
            if (e.AddedItems != null && e.AddedItems.Count > 0)
            {
                LogItemSchema item = (LogItemSchema)e.AddedItems[0];
                if (sender == CategoryList && item.ChildItems != null)
                {
                    ListViewItem listItem = CategoryList.ItemContainerGenerator.ContainerFromItem(item) as ListViewItem;
                    if (listItem != null)
                    {
                        // make sure expander is toggled.
                        Expander expander = listItem.FindDescendantsOfType<Expander>().FirstOrDefault();
                        expander.IsExpanded = true;
                    }
                }
                else if (item.ChildItems == null || item.ChildItems.Count == 0)
                {
                    if (item.Parent == null)
                    {
                        GraphItem(item);
                    }
                }
            }

            if (e.RemovedItems != null && e.RemovedItems.Count > 0)
            {
                LogItemSchema item = (LogItemSchema)e.RemovedItems[0];
                if (sender == CategoryList && item.ChildItems != null)
                {
                    ListViewItem listItem = CategoryList.ItemContainerGenerator.ContainerFromItem(item) as ListViewItem;
                    if (listItem != null)
                    {
                        Expander expander = listItem.FindDescendantsOfType<Expander>().FirstOrDefault();
                        expander.IsExpanded = false;
                    }
                }

            }
        }

        private void OnShowMap(object sender, RoutedEventArgs e)
        {
            ShowMap();
        }

        private void OnHideMap(object sender, RoutedEventArgs e)
        {
            myMap.Visibility = Visibility.Collapsed;
            ChartStack.Visibility = Visibility.Visible;
        }

        List<Flight> GetSelectedFlights()
        {
            List<Flight> selected = new List<Model.Flight>();
            foreach (Flight f in FlightView.SelectedItems)
            {
                selected.Add(f);
            }
            return selected;
        }

        double lastLat = 0;
        double lastLon = 0;

        private void MapLocation(double latitude, double longitude)
        {
            if (Math.Round(lastLat, 5) == Math.Round(latitude) && Math.Round(lastLon, 5) == Math.Round(longitude, 5))
            {
                // hasn't moved far enough yet...
                return;
            }
            lastLat = latitude;
            lastLon = longitude;
            bool first = false;
            if (currentFlight == null)
            {
                first = true;
                currentFlight = new MapPolyline();
                currentFlight.StrokeThickness = 4;
                currentFlight.Stroke = new SolidColorBrush(Colors.Magenta);
                currentFlight.Locations = new LocationCollection();
                myMap.Children.Add(currentFlight);
            }
            currentFlight.Locations.Add(new Location() { Latitude = latitude, Longitude = longitude });
            // make sure it's on top.
            if (myMap.Children.IndexOf(currentFlight) < 0)
            {
                myMap.Children.Add(currentFlight);
            }
            else if (myMap.Children.IndexOf(currentFlight) != myMap.Children.Count - 1)
            {
                myMap.Children.Remove(currentFlight);
                myMap.Children.Add(currentFlight);
            }
            if (currentFlight.Locations.Count > 1000)
            {
                // remove the older points.
                currentFlight.Locations.RemoveAt(0);
            }
            if (first && myMap.Visibility == Visibility.Visible)
            {
                myMap.SetView(currentFlight.Locations, new Thickness(20.0), 0);
            }
        }

        List<LogEntryGPS> mapData = null;

        Pushpin GetOrCreateMapMarker(Location loc)
        {
            Pushpin mapMarker = null;
            foreach (var child in myMap.Children)
            {
                if (child is Pushpin)
                {
                    mapMarker = (Pushpin)child;
                    break;
                }
            }
            if (mapMarker == null)
            {
                mapMarker = new Pushpin();
                myMap.Children.Add(mapMarker);
            }
            mapMarker.Location = loc;
            return mapMarker;
        }

        void ShowMap()
        {
            myMap.Children.Clear();

            List<Flight> selected = GetSelectedFlights();
            if (selected.Count == 0)
            {
                // show everything.
                selected.Add(new Flight() { StartTime = DateTime.MinValue, Duration = TimeSpan.MaxValue });
            }
            mapData = new List<Utilities.LogEntryGPS>();
            var glitchIcon = XamlExtensions.LoadImageResource("Assets.GpsGlitchIcon.png");
            var imageLayer = new MapLayer();
            myMap.Children.Add(imageLayer);
            MapPolyline last = currentFlight;
            foreach (IDataLog log in this.logs)
            {
                if (log != null)
                {
                    bool gpsIsBad = false;
                    foreach (var flight in selected)
                    {
                        if (flight.Log == null || flight.Log == log)
                        {
                            MapPolyline line = new MapPolyline();
                            line.StrokeLineJoin = PenLineJoin.Round;
                            line.StrokeThickness = 4;
                            line.Stroke = new SolidColorBrush(GetRandomColor());
                            LocationCollection points = new LocationCollection();
                            mappedLogEntries = new List<Model.LogEntry>();

                            //Debug.WriteLine("time,\t\tlat,\t\tlong,\t\t\tnsat,\talt,\thdop,\tfix");
                            foreach (var row in log.GetRows("GPS", flight.StartTime, flight.Duration))
                            {
                                LogEntryGPS gps = new LogEntryGPS(row);
                                //Debug.WriteLine("{0},\t{1},\t{2},\t{3},\t\t{4:F2},\t{5},\t{6}", gps.GPSTime,  gps.Lat, gps.Lon, gps.nSat, gps.Alt, gps.EPH, gps.Fix);
                                if (!(Math.Floor(gps.Lat) == 0 && Math.Floor(gps.Lon) == 0))
                                {
                                    // map doesn't like negative altitudes.
                                    double alt = gps.Alt;
                                    if (alt < 0)
                                    {
                                        alt = 0;
                                    }
                                    mapData.Add(gps);
                                    mappedLogEntries.Add(row);
                                    var pos = new Location() { Altitude = alt, Latitude = gps.Lat, Longitude = gps.Lon };
                                    points.Add(pos);
                                    ulong time = (ulong)gps.GPSTime;
                                    if (time != 0)
                                    {
                                        if ((gps.nSat < 5 || gps.EPH > 20))
                                        {
                                            if (!gpsIsBad)
                                            {
                                                gpsIsBad = true;
                                                //Debug.WriteLine("{0},\t{1},\t{2},\t{3},\t\t{4:F2},\t{5},\t{6}", gps.GPSTime, gps.Lat, gps.Lon, gps.nSat, gps.Alt, gps.EPH, gps.Fix);
                                                Image img = new Image();
                                                img.Width = 30;
                                                img.Height = 30;
                                                img.Source = glitchIcon;
                                                img.Stretch = Stretch.None;
                                                img.ToolTip = "GPS Glitch!";
                                                imageLayer.AddChild(img, pos, PositionOrigin.Center);
                                            }
                                        }
                                        else
                                        {
                                            gpsIsBad = false;
                                        }
                                    }
                                }
                            }

                            if (points.Count > 0)
                            {
                                line.Locations = points;
                                myMap.Children.Add(line);
                                last = line;
                            }
                        }
                    }
                }
            }

            // hide the stuff on top...
            QuadButton.IsChecked = false;
            ConsoleButton.IsChecked = false;
            SystemConsole.Hide();
            ChartStack.Visibility = Visibility.Collapsed;
            myMap.Visibility = Visibility.Visible;
            myMap.UpdateLayout();

            if (last != null)
            {
                try
                {
                    myMap.SetView(last.Locations, new Thickness(20.0), 0);
                }
                catch (Exception ex)
                {
                    ShowStatus(ex.Message);
                }
            }

        }

        private LocationCollection GetBoundingBox(LocationCollection locations)
        {
            if (locations.Count == 0)
            {
                throw new Exception("Must provide at least one location");
            }
            Location first = locations.First();
            double minLat = first.Latitude;
            double maxLat = first.Latitude;
            double minlong = first.Longitude;
            double maxLong = first.Longitude;
            foreach (Location i in locations)
            {
                minLat = Math.Min(minLat, i.Latitude);
                maxLat = Math.Max(maxLat, i.Latitude);
                minlong = Math.Min(minlong, i.Longitude);
                maxLong = Math.Max(maxLong, i.Longitude);
            }
            var corners = new LocationCollection();
            corners.Add(new Location(minLat, minlong));
            corners.Add(new Location(minLat, minlong ));
            corners.Add(new Location(minLat, minlong ));
            corners.Add(new Location(minLat, minlong ));
            return corners;
        }

        private void UpdateButtons()
        {
            UiDispatcher.RunOnUIThread(() =>
            {
                ShowMapButton.Visibility = Visibility.Visible;
            });
        }
        
        HashSet<ListView> childLists = new HashSet<ListView>();

        private void OnChildListItemSelected(object sender, SelectionChangedEventArgs e)
        {
            if (e.AddedItems != null && e.AddedItems.Count > 0 && e.OriginalSource == sender)
            {
                childLists.Add((ListView)sender);
                LogItemSchema item = (LogItemSchema)e.AddedItems[0];
                GraphItem(item);
            }
        }

        IEnumerable<DataValue> GetSelectedDataValues(LogItemSchema schema)
        {
            List<Flight> selected = GetSelectedFlights();
            if (selected.Count == 0)
            {
                // show everything.
                selected.Add(new Flight() { StartTime = DateTime.MinValue, Duration = TimeSpan.MaxValue });
            }

            foreach (IDataLog log in this.logs)
            {
                if (log != null)
                {
                    foreach (var flight in selected)
                    {
                        if (flight.Log == null || flight.Log == log)
                        {
                            foreach (var dv in log.GetDataValues(schema, flight.StartTime, flight.Duration))
                            {
                                yield return dv;
                            }
                        }
                    }
                }
            }            
        }

        Thickness defaultChartMargin = new Thickness(0, 10, 0, 10);

        Color GetRandomColor()
        {
            return new HlsColor((float)(rand.NextDouble() * 360), 0.7f, 0.95f).Color;
        }

        Random rand = new Random();

        SimpleLineChart AddChart(LogItemSchema schema, IEnumerable<DataValue> values)
        {
            SimpleLineChart chart = new SimpleLineChart();
            chart.ChartGenerated += OnNewChartGenerated;
            chart.ClearAllAdornments += OnClearAllAdornments;
            chart.DisplayMessage += OnShowMessage;
            chart.PointerMoved += OnPointerMoved;
            chart.Margin = defaultChartMargin;
            chart.Focusable = false;
            chart.Closed += OnChartClosed;
            chart.LineColor = GetRandomColor();
            chart.StrokeThickness = 1;
            chart.Tag = schema;
            InitializeChartData(schema, chart, values);
            if (chart != null)
            {
                if (chartGroup != null)
                {
                    chartGroup.AddChart(chart);
                    if (chartGroup.Parent == null)
                    {
                        ChartStack.AddChartGroup(chartGroup);
                    }
                }
                else
                {
                    ChartStack.AddChart(chart);
                }
                LayoutCharts();
            }
            return chart;
        }

        private void OnPointerMoved(object sender, DataValue data)
        {
            if (data != null && mapData != null && myMap.Visibility == Visibility.Visible)
            {
                double time = data.X;
                double dist = double.MaxValue; 
                int closest = 0;
                int i = 0;
                LogEntryGPS gps = null;
                // find matching gps location in time.
                foreach (var item in mapData)
                {
                    double t = item.Timestamp;
                    double d = Math.Abs((double)(t - time));
                    if (dist == ulong.MaxValue || d < dist)
                    {
                        gps = item;
                        dist = d;
                        closest = i;
                    }
                    i++;
                }
                GetOrCreateMapMarker(new Location(gps.Lat, gps.Lon, gps.Alt));
            }
        }

        private void OnShowMessage(object sender, string message)
        {
            SystemConsole.Write(message);
            ConsoleButton.IsChecked = true;
            SystemConsole.Show();
        }

        private void GraphItem(LogItemSchema schema)
        {
            if (schema.IsNumeric)
            {
                ChartStack.Visibility = Visibility.Visible;
                ChartStack.UpdateLayout();

                SimpleLineChart chart = null;

                if (currentFlightLog != null && schema.Root == currentFlightLog.Schema)
                {
                    List<DataValue> values = new List<DataValue>(currentFlightLog.GetDataValues(schema, DateTime.MinValue, TimeSpan.MaxValue));
                    chart = AddChart(schema, values);

                    if (!pauseRecording)
                    {
                        // now turn on live scrolling if we are recording...
                        chart.LiveScrolling = true;
                        // the X values are in microseconds (s0 the numerator is the speed of scrolling).
                        chart.LiveScrollingXScale = 50.0 / 1000000.0;
                    }
                    liveScrolling.Add(chart);

                    // now start watching the live update for new values that need to be added to this chart.
                    Task.Run(() =>
                    {
                        LiveUpdate(chart, currentFlightLog, schema);
                    });

                }
                else
                {
                    var data = GetSelectedDataValues(schema);
                    if (data.Count() > 0)
                    {
                        chart = AddChart(schema, data);
                    }                                      
                    ShowStatus(string.Format("Found {0} data values", data.Count()));
                }

                ConsoleButton.IsChecked = false;
                SystemConsole.Hide();
            }
            else
            {
                StringBuilder sb = new StringBuilder();
                string previous = null;
                List<DataValue> unique = new List<Model.DataValue>();
                foreach (var value in GetSelectedDataValues(schema))
                {
                    if (!string.IsNullOrEmpty(value.Label))
                    {
                        if (previous != value.Label)
                        {
                            unique.Add(value);
                            sb.Append(((ulong)value.X).ToString());
                            sb.Append(": ");
                            sb.AppendLine(value.Label);
                            previous = value.Label;
                        }
                    }
                }
                SystemConsole.Write(sb.ToString());
                ConsoleButton.IsChecked = true;
                SystemConsole.Show();
            }
        }

        private void AnnotateMap(LogItemSchema schema)
        {
            List<DataValue> unique = new List<Model.DataValue>();
            if (schema.IsNumeric)
            {
                var data = GetSelectedDataValues(schema);
                ShowStatus(string.Format("Found {0} data values", data.Count()));
                if (data.Count() > 0)
                {
                    double previous = 0;
                    {
                        // uniquify it.
                        foreach (var value in data)
                        {
                            if (value.Y != previous)
                            {
                                unique.Add(value);
                                previous = value.Y;
                            }
                        }
                    }
                }
            }
            else
            {
                StringBuilder sb = new StringBuilder();
                string previous = null;
                foreach (var value in GetSelectedDataValues(schema))
                {
                    if (!string.IsNullOrEmpty(value.Label))
                    {
                        if (previous != value.Label)
                        {
                            unique.Add(value);
                            sb.Append(value.X.ToString());
                            sb.Append(": ");
                            sb.AppendLine(value.Label);
                            previous = value.Label;
                        }
                    }
                }
            }

            // if there are too many values, then limit it to an even spread of 100 items.
            if (unique.Count > 100)
            {
                var summary = new List<Model.DataValue>();
                double skip = (unique.Count / 100);
                for (int i = 0, n = unique.Count; i < n; i++)
                {
                    var value = unique[i];
                    if (i >= summary.Count * unique.Count / 100)
                    {
                        summary.Add(value);
                    }
                }
                unique = summary;
            }
            AnnotateMap(unique);
        }

        private void AnnotateMap(List<DataValue> unique)
        {
            if (this.mappedLogEntries == null || this.mappedLogEntries.Count == 0)
            {
                ShowMap();
            }

            if (this.mappedLogEntries == null || this.mappedLogEntries.Count == 0)
            {
                MessageBox.Show("Sorry, could not find GPS map info, so cannot annotate data on the map",
                    "GPS info is missing", MessageBoxButton.OK, MessageBoxImage.Exclamation);
                return;
            }

            if (annotationLayer != null)
            {
                myMap.Children.Remove(annotationLayer);
            }
            annotationLayer = new MapLayer();
            
            SolidColorBrush annotationBrush = new SolidColorBrush(Color.FromArgb(0x80, 0xff, 0xff, 0xB0));

            foreach (var dv in unique)
            {
                LogEntry closest = null;
                LogField field = dv.UserData as LogField;
                if (field != null)
                {
                    // csv log
                    LogEntry e = field.Parent;
                    closest = FindNearestMappedItem(e.Timestamp);
                }
                else
                {
                    // px4 log?
                    Message msg = dv.UserData as Message;
                    if (msg != null)
                    {
                        closest = FindNearestMappedItem(msg.GetTimestamp());
                    }
                    else
                    {
                        // mavlink
                        MavlinkLog.Message mavmsg = dv.UserData as MavlinkLog.Message;
                        if (mavmsg != null)
                        {
                            closest = FindNearestMappedItem(mavmsg.Timestamp.Ticks / 10);
                        }
                    }
                }

                if (closest != null)
                {
                    LogEntryGPS gps = new LogEntryGPS(closest);
                    // map doesn't like negative altitudes.
                    double alt = gps.Alt;
                    if (alt < 0)
                    {
                        alt = 0;
                    }
                    var pos = new Location() { Altitude = alt, Latitude = gps.Lat, Longitude = gps.Lon };
                    string label = dv.Label;
                    if (string.IsNullOrEmpty(label))
                    {
                        label = dv.Y.ToString();
                    }
                    annotationLayer.AddChild(new TextBlock(new Run(label) { Background = annotationBrush }), pos, PositionOrigin.BottomLeft);
                }
                
            }
            myMap.Children.Add(annotationLayer);

            SystemConsole.Hide();
            ChartStack.Visibility = Visibility.Collapsed;
            myMap.Visibility = Visibility.Visible;
            myMap.UpdateLayout();
        }

        private LogEntry FindNearestMappedItem(double t)
        {
            LogEntry closest = null;
            double bestDiff = 0;
            // find nearest mapped location (nearest in time).
            foreach (var mapped in this.mappedLogEntries)
            {
                var time = mapped.Timestamp;
                var diff = Math.Abs((double)time - t);

                if (closest == null || diff < bestDiff)
                {
                    closest = mapped;
                    bestDiff = diff;
                }
            }
            return closest;
        }

        private void OnNewChartGenerated(object sender, List<DataValue> e)
        {
            SimpleLineChart chart = (SimpleLineChart)sender;
            AddChart((LogItemSchema)chart.Tag, e);
        }
        
        private void OnClearAllAdornments(object sender, EventArgs e)
        {
            foreach (var chart in ChartStack.FindCharts())
            {
                chart.ClearAdornments();
            }
        }
        
        private void InitializeChartData(LogItemSchema schema, SimpleLineChart chart, IEnumerable<DataValue> values)
        {
            chart.SetData(new Model.DataSeries()
            {
                Name = schema.Name,
                Values = new List<DataValue>(values)
            });
        }

        private void LiveUpdate(SimpleLineChart chart, MavlinkLog currentFlightLog, LogItemSchema schema)
        {
            // this method is running on a background task and it's job is to read an infinite stream of
            // data values from the log and show them in the live scrolling chart.

            CancellationTokenSource canceller = new CancellationTokenSource();

            chart.Closed += (s, e) =>
            {
                canceller.Cancel();
            };

            var query = currentFlightLog.LiveQuery(schema, canceller.Token);
            foreach (DataValue item in query)
            {
                if (item == null)
                {
                    return;
                }
                chart.SetCurrentValue(item);
            }

            chart.Closed += (sender, e) =>
            {
                canceller.Cancel();
            };

            Debug.WriteLine("LiveUpdate terminating on schema item " + schema.Name);
        }

        private void LayoutCharts()
        {
            // layout charts to fill the space available.
            ChartStack.UpdateLayout();
            double height = ChartStack.ActualHeight;
            double count = ChartStack.ChartCount;
            height -= (count * (defaultChartMargin.Top + defaultChartMargin.Bottom)); // remove margins
            if (height < 0)
            {
                height = 0;
            }
            double chartHeight = Math.Min(MaxChartHeight, height / count);
            bool found = false;
            foreach (FrameworkElement c in ChartStack.Charts)
            {
                found = true;
                c.Height = chartHeight;
            }

            // give all the charts the same min/max on the X dimension so that the charts are in sync (even when they are not grouped).
            //ChartScaleInfo combined = null;
            //foreach (SimpleLineChart chart in ChartStack.FindCharts())
            //{
            //    var info = chart.ComputeScaleSelf();
            //    if (combined == null)
            //    {
            //        combined = info;
            //    }
            //    else
            //    {
            //        combined.Combine(info);
            //    }
            //}

            //// now set the min/max on each chart.
            //foreach (SimpleLineChart chart in ChartStack.FindCharts())
            //{
            //    chart.FixMinimumX = combined.minX;
            //    chart.FixMaximumX = combined.maxX;
            //    chart.InvalidateArrange();
            //}

            if (!found)
            {
                ChartStack.Visibility = Visibility.Collapsed;
            }
        }

        private void OnChartClosed(object sender, EventArgs e)
        {
            SimpleLineChart chart = sender as SimpleLineChart;
            ChartStack.RemoveChart(chart);
            chart.LiveScrolling = false;
            liveScrolling.Remove(chart);
            LayoutCharts();

            LogItemSchema item = (chart.Tag as LogItemSchema);
            if (item != null)
            {
                UnselectCategory(item);
            }

        }

        private void UnselectCategory(LogItemSchema item)
        {
            if (CategoryList.SelectedItems.Contains(item))
            {
                CategoryList.SelectedItems.Remove(item);
            }
            else
            {
                // might be a child category item...
                foreach (var childList in childLists)
                {
                    if (childList.SelectedItems.Contains(item))
                    {
                        childList.SelectedItems.Remove(item);
                    }
                }
            }
        }


        private void OnClear(object sender, RoutedEventArgs e)
        {
            ChartStack.ClearCharts();
            liveScrolling.Clear();
            logs.Clear();
            CategoryList.SelectedItem = null;
            allFlights.Clear();
            if (currentFlightLog != null)
            {
                currentFlightLog.Clear();
                logs.Add(currentFlightLog);
            }
            if (currentFlight != null)
            {
                currentFlight.Locations = new LocationCollection();
            }
            SystemConsole.Clear();
            myMap.Children.Clear();
            ImageViewer.Source = null;
            ShowSchema();
            UpdateTitle("");
        }

        private void OnFlightSelected(object sender, SelectionChangedEventArgs e)
        {
            UiDispatcher.RunOnUIThread(() =>
            {
                if (myMap.Visibility == Visibility.Visible)
                {
                    ShowMap();
                }
                else
                {
                    // todo: show sensor data pruned to this flight time...
                    foreach (LogItemSchema item in CategoryList.SelectedItems)
                    {
                        if (item.ChildItems == null || item.ChildItems.Count == 0)
                        {
                            GraphItem(item);
                        }
                    }
                }
            });
        }
        
        private void OnFlightViewKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Delete)
            {
                // remove this flight.
            }
        }

        private void OnItemExpanded(object sender, RoutedEventArgs e)
        {
            Expander expander = (Expander)sender;
            ListView childView = expander.Content as ListView;
            if (childView != null)
            {
                LogItemSchema item = (LogItemSchema)expander.DataContext;
                childView.ItemsSource = item.ChildItems;
            }
        }

        private void OnItemCollapsed(object sender, RoutedEventArgs e)
        {
            Expander expander = (Expander)sender;
            Expander source = e.OriginalSource as Expander;
            if (source != null && source != expander)
            {
                // bugbug: for some reason WPF also sends collapse event to all the parents
                // but we want to ignore those.
                return;
            }
            ListView childView = expander.Content as ListView;
            if (childView != null)
            {
                childView.ItemsSource = null;
            }
            // todo: remove the graphs...
        }

        ChartGroup chartGroup;

        private void OnGroupChecked(object sender, RoutedEventArgs e)
        {
            chartGroup = new ChartGroup() { HorizontalAlignment = HorizontalAlignment.Stretch, VerticalAlignment = VerticalAlignment.Stretch };
        }

        private void OnGroupUnchecked(object sender, RoutedEventArgs e)
        {
            chartGroup = null;
        }

        private void OnClearZoom(object sender, RoutedEventArgs e)
        {
            ChartStack.ResetZoom();
        }

        private void OnConnectorClick(object sender, MouseButtonEventArgs e)
        {
            ConnectionPanel.Start();
            XamlExtensions.Flyout(ConnectionPanel);
        }

        private void OnOpenFileCommand(object sender, ExecutedRoutedEventArgs e)
        {
            OnOpenFile(sender, e);
        }

        private void OnShowQuad(object sender, RoutedEventArgs e)
        {
            ModelViewer.Visibility = Visibility.Visible;
            ConsoleButton.IsChecked = false;

            SendMessage(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
                new MAVLink.mavlink_command_long_t()
                {
                    command = (ushort)MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL,
                    param1 = (float)MAVLink.MAVLINK_MSG_ID.ATTITUDE_QUATERNION,
                    param2 = 50000
                }
            );
        }

        private void OnHideQuad(object sender, RoutedEventArgs e)
        {
            ModelViewer.Visibility = Visibility.Collapsed;

            SendMessage(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG,
                new MAVLink.mavlink_command_long_t()
                {
                    command = (ushort)MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL,
                    param1 = (float)MAVLink.MAVLINK_MSG_ID.ATTITUDE_QUATERNION,
                    param2 = 500000
                }
            );
        }

        private void OnRecord(object sender, RoutedEventArgs e)
        {
            Button button = (Button)sender;
            if (button.Tag != null)
            {
                // paused
                button.Content = button.Tag;
                button.Tag = null;
                StopRecording();
            }
            else
            {
                // start recording.
                button.Tag = button.Content;
                button.Content = "\ue15b";
                StartRecording();
            }
        }

        void StartRecording()
        {
            if (currentFlightLog == null)
            {
                currentFlightLog = new MavlinkLog();
                currentFlightLog.SchemaChanged += (s, args) =>
                {
                    ShowSchema();
                };
            }
            pauseRecording = false;
            foreach (var chart in liveScrolling)
            {
                chart.LiveScrolling = true;
                // the X values are in milliseconds (s0 the numerator is the speed of scrolling).
                chart.LiveScrollingXScale = 5.0 / 1000.0;
            }
        }

        bool pauseRecording;
        List<SimpleLineChart> liveScrolling = new List<SimpleLineChart>();

        void StopRecording()
        {
            pauseRecording = true;
            foreach (var chart in liveScrolling)
            {                
                chart.LiveScrolling = false;
            }
        }

        private void OnShowCamera(object sender, RoutedEventArgs e)
        {
            CameraPanel.Visibility = Visibility.Visible;
            showImageStream = true;
        }

        private void OnHideCamera(object sender, RoutedEventArgs e)
        {
            showImageStream = false;
            CameraPanel.Visibility = Visibility.Collapsed;
        }
        private void ShowImageStream()
        {
            // show encapsulated data
        }

        bool showImageStream;

        struct IncomingImage
        {
            public uint size;              // Image size being transmitted (bytes)
            public int packets;           // Number of data packets being sent for this image
            public int packetsArrived;    // Number of data packets received
            public int payload;           // Payload size per transmitted packet (bytes). Standard is 254, and decreases when image resolution increases.
            public int quality;           // Quality of the transmitted image (percentage)
            public int type;              // Type of the transmitted image (BMP, PNG, JPEG, RAW 8 bit, RAW 32 bit)
            public int width;             // Width of the image stream
            public int height;            // Width of the image stream
            public byte[] data;           // Buffer for the incoming bytestream
            public long start;            // Time when we started receiving data
        };

        IncomingImage incoming_image = new IncomingImage();

        public LogItemSchema rightClickedItem { get; private set; }

        private void OnShowConsole(object sender, RoutedEventArgs e)
        {
            SystemConsole.Show();
        }

        #region System Console
        private void OnHideConsole(object sender, RoutedEventArgs e)
        {
            SystemConsole.Hide();
        }

        void SendMessage(MAVLink.MAVLINK_MSG_ID id, object mavlinkPayload)
        {
            var channel = ConnectionPanel.Channel;
            if (channel != null)
            {
                MavLinkMessage message = new MavLinkMessage();
                message.ComponentId = (byte)MAVLink.MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
                message.SystemId = 255;
                message.MsgId = id;
                message.TypedPayload = mavlinkPayload;
                channel.SendMessage(message);
            }
        }

        #endregion

        private void OnSettings(object sender, RoutedEventArgs e)
        {
            XamlExtensions.Flyout(AppSettingsPanel);
        }

        private void OnMapTest(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.OpenFileDialog fo = new Microsoft.Win32.OpenFileDialog();
            fo.Filter = "CSV Files (*.csv)|*.csv";
            fo.CheckFileExists = true;
            fo.Multiselect = false;
            if (fo.ShowDialog() == true)
            {
                LoadMapData(fo.FileName);
            }
        }

        private void LoadMapData(string fileName)
        {
            // turn data into two lookup tables for easy access.
            double[,] xmag  = new double[180,360];
            double[,] ymag = new double[180, 360];

            using (StreamReader reader = new StreamReader(fileName))
            {
                string line = reader.ReadLine();
                while (line != null)
                {
                    string[] parts = line.Split('\t');
                    if (parts.Length == 5)
                    {
                        double lat, lon, x, y;
                        if (double.TryParse(parts[0], out lat))
                        {
                            lon = double.Parse(parts[1]);
                            x = double.Parse(parts[2]);
                            y = double.Parse(parts[3]);
                            lat += 90;
                            lon += 180;
                            xmag[(int)lat, (int)lon] = x;
                            ymag[(int)lat, (int)lon] = y;
                        }
                    }

                    line = reader.ReadLine();
                }
            }

            DrawVectors(xmag, ymag);
        }

        class LocationComparer : IEqualityComparer<Location>
        {
            public bool Equals(Location x, Location y)
            {
                return x.Altitude == y.Altitude && x.Latitude == y.Latitude && x.Longitude == y.Longitude;
            }

            public int GetHashCode(Location obj)
            {
                return (int)(obj.Altitude + obj.Latitude + obj.Longitude);
            }
        }

        private void DrawVectors(double[,] xmag, double[,] ymag)
        {
            // find guassian lines in the map and draw them so it looks like this:
            // https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_D_MERC.pdf

            for (int i = 0; i < 180; i++)
            {
                for (int j = 0; j < 360; j++)
                {
                    double x = xmag[i, j];
                    double y = ymag[i, j];

                    double latitude = i - 90;
                    double longitude = j - 180;

                    MapPolyline line = new MapPolyline();
                    line.StrokeThickness = 1;
                    line.Stroke = new SolidColorBrush(Colors.Red);
                    LocationCollection points = new LocationCollection();
                    Location pos = new Location() { Altitude = 0, Latitude = latitude, Longitude = longitude };
                    points.Add(pos);

                    // ok, we have a winner, pick this one and continue.
                    pos = new Location() { Latitude = latitude + (x*2), Longitude = longitude + (y*2) };
                    points.Add(pos);
                    line.Locations = points;
                    myMap.Children.Add(line);
                }
            }
        }

        private void OnPaste(object sender, ExecutedRoutedEventArgs e)
        {
            if (Clipboard.ContainsImage())
            {
                var image = Clipboard.GetImage();
                ImageViewer.Source = image;
                CameraPanel.Visibility = Visibility.Visible;
            }
        }

        private void OnAnnotateItem(object sender, RoutedEventArgs e)
        {
            LogItemSchema item = this.rightClickedItem;
            if (item != null)
            {
                AnnotateMap(item);
            }
        }

        private void OnRightClickCategoryList(object sender, MouseButtonEventArgs e)
        {
            this.rightClickedItem = null;
            Point pos = e.GetPosition(CategoryList);
            DependencyObject dep = (DependencyObject)e.OriginalSource;
            while ((dep != null) && !(dep is ListViewItem))
            {
                dep = VisualTreeHelper.GetParent(dep);
            }
            if (dep == null)
                return;
            ListViewItem listitem = (ListViewItem)dep;
            LogItemSchema item = listitem.DataContext as LogItemSchema;
            this.rightClickedItem = item;
        }
    }
}

