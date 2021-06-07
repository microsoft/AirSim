using LogViewer.Utilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using System.Reflection;
using Microsoft.Networking.Mavlink;
using LogViewer.Controls;
using System.Collections;
using System.Threading;

namespace LogViewer.Model
{

    class MavlinkLog : IDataLog
    {
        string file;
        ulong startTicks = 0;
        DateTime startTime;
        TimeSpan duration;
        List<Message> data = new List<Message>();
        LogItemSchema schema = new LogItemSchema() { Name = "MavlinkLog", Type = "Root" };
        Dictionary<Type, LogItemSchema> schemaCache = new Dictionary<Type, LogItemSchema>();
        Dictionary<string, MAVLink.mavlink_param_value_t> parameters = new Dictionary<string, MAVLink.mavlink_param_value_t>();

        internal class Message
        {
            public MavLinkMessage Msg;
            public ulong Ticks;
            public DateTime Timestamp;
            public object TypedValue;
        }

        public MavlinkLog()
        {
        }

        public LogItemSchema Schema
        {
            get { return this.schema; }
        }

        public DateTime StartTime
        {
            get { return startTime; }
        }
        public TimeSpan Duration
        {
            get { return duration; }
        }

        public ulong StartTicks { get; private set; }

        public IEnumerable<DataValue> GetDataValues(LogItemSchema schema, DateTime startTime, TimeSpan duration)
        {
            if (schema != null)
            {
                bool first = true;
                ulong startTicks = 0;
                ulong previousTicks = 0;
                foreach (var msg in GetMessages(startTime, duration))
                {
                    if (first)
                    {
                        startTicks = previousTicks = msg.Ticks;
                        first = false;
                    }

                    previousTicks = msg.Ticks;
                    DataValue value = GetDataValue(schema, msg, startTicks);
                    if (value != null)
                    {
                        yield return value;
                    }
                }
            }
        }

        internal DataValue GetDataValue(LogItemSchema schema, Message msg, ulong startTicks)
        {
            // we support 3 levels of nesting, so schema could be the row, the column or an array item.
            LogItemSchema rowSchema = schema;
            LogItemSchema columnSchema = schema;
            LogItemSchema arraySchema = schema;
            if (rowSchema.Parent != this.schema)
            {
                rowSchema = rowSchema.Parent;
            }
            if (rowSchema.Parent != this.schema)
            {
                columnSchema = rowSchema;
                rowSchema = rowSchema.Parent;
            }

            StringBuilder textBuilder = new StringBuilder();
            var row = msg.TypedValue;
            if (row != null && row.GetType().Name == rowSchema.Type)
            {
                // get a time value for this message.
                FieldInfo fi = row.GetType().GetField(columnSchema.Name, BindingFlags.Public | BindingFlags.Instance);
                if (fi != null)
                {
                    object value = fi.GetValue(row);
                    double x = (double)(msg.Ticks - startTicks);
                    DataValue data = new DataValue() { X = x, UserData = msg }; // microseconds.
                    
                    // byte array is special (we treat this like text).
                    if (value is byte[])
                    {
                        textBuilder.Length = 0;
                        byte[] text = (byte[])value;
                        bool binary = false;
                        // see if this is binary or text.
                        for (int i = 0, n = text.Length; i < n; i++)
                        {
                            byte b = text[i];
                            if (b != 0 && b < 0x20 || b > 0x80)
                            {
                                binary = true;
                            }
                        }

                        for (int i = 0, n = text.Length; i < n; i++)
                        {
                            byte b = text[i];
                            if (b != 0)
                            {
                                if (binary)
                                {
                                    textBuilder.Append(b.ToString("x2"));
                                    textBuilder.Append(" ");
                                }
                                else
                                {
                                    char ch = Convert.ToChar(b);
                                    textBuilder.Append(ch);
                                }
                            }
                        }

                        data.Label = textBuilder.ToString();
                    }
                    else if (value is string)
                    {
                        data.Label = (string)value;
                    }
                    else
                    {
                        if (fi.FieldType.IsArray)
                        {
                            // then we are expecting an array selector as well...
                            Array a = (Array)value;
                            int index = 0;
                            int.TryParse(arraySchema.Name, out index);
                            if (a.Length > index)
                            {
                                value = a.GetValue(index);
                            }
                        }

                        data.Y = ConvertToNumeric(value);
                        data.Label = GetLabel(row, columnSchema.Name);
                    };
                    return data;
                }
            }
            return null;
        }

        private string GetLabel(object row, string fieldName)
        {
            if (row is MAVLink.mavlink_heartbeat_t)
            {
                MAVLink.mavlink_heartbeat_t heartbeat = (MAVLink.mavlink_heartbeat_t)row;
                if (fieldName == "base_mode")
                {
                    return GetModeFlags(heartbeat.base_mode);
                }
                else if (fieldName == "system_status")
                {
                    return GetSystemStateName(heartbeat.system_status);
                }
                else if (fieldName == "custom_mode")
                {
                    return GetCustomModeFlags(heartbeat.custom_mode);
                }
            }
            else if (row is MAVLink.mavlink_extended_sys_state_t)
            {
                MAVLink.mavlink_extended_sys_state_t state = (MAVLink.mavlink_extended_sys_state_t)row;
                if (fieldName == "landed_state")
                {
                    return GetLandedStateName(state.landed_state);
                }
                else if (fieldName == "vtol_state")
                {
                    return GetVTolStateName(state.vtol_state);
                }
            }
            return null;
        }
        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            return new MavlinkQuery(this, schema, token);
        }

        internal void AddQuery(MavlinkQueryEnumerator q)
        {
            lock (queryEnumerators)
            {
                queryEnumerators.Add(q);
            }
        }
        internal void RemoveQuery(MavlinkQueryEnumerator q)
        {
            lock (queryEnumerators)
            {
                queryEnumerators.Remove(q);
            }
        }

        /// <summary>
        /// This implements a nice little reactive stream over the DataValues.
        /// </summary>
        internal class MavlinkQuery : IEnumerable<DataValue>
        {
            private MavlinkLog log;
            private LogItemSchema schema;
            private CancellationToken token;

            internal MavlinkQuery(MavlinkLog log, LogItemSchema schema, CancellationToken token)
            {
                this.log = log;
                this.schema = schema;
                this.token = token;
            }

            public IEnumerator<DataValue> GetEnumerator()
            {
                return new MavlinkQueryEnumerator(this.log, this.schema, this.token);
            }

            IEnumerator IEnumerable.GetEnumerator()
            {
                return new MavlinkQueryEnumerator(this.log, this.schema, this.token);
            }

        }

        internal class MavlinkQueryEnumerator : IEnumerator<DataValue>
        {
            private MavlinkLog log;
            private LogItemSchema schema;
            private DataValue current;
            private DataValue next;
            private CancellationToken token;
            private bool disposed;
            private bool waiting;
            private ulong startTicks;
            private System.Threading.Semaphore available = new System.Threading.Semaphore(0, 1);

            internal MavlinkQueryEnumerator(MavlinkLog log, LogItemSchema schema, CancellationToken token)
            {
                this.log = log;
                this.schema = schema;
                this.token = token;
                this.startTicks = log.StartTicks;
                this.log.AddQuery(this);
            }

            internal void Add(Message msg)
            {
                DataValue d = this.log.GetDataValue(schema, msg, startTicks);
                if (d != null)
                {
                    next = d;
                    if (waiting)
                    {
                        available.Release();
                    }
                }
            }

            public DataValue Current
            {
                get { return current; }
            }

            object IEnumerator.Current
            {
                get { return current; }
            }

            public void Dispose()
            {
                disposed = true;
                this.log.RemoveQuery(this);
            }

            public bool MoveNext()
            {
                if (disposed || token.IsCancellationRequested)
                {
                    return false;
                }

                if (next == null)
                {
                    waiting = true;
                    WaitHandle.WaitAny(new[] { available, token.WaitHandle });
                    waiting = false;
                }
                current = next;

                return true;
            }

            public void Reset()
            {
                // this is a forward only infinite stream.
                throw new NotSupportedException();
            }
        }



        IEnumerable<Message> GetMessages(DateTime startTime, TimeSpan duration)
        {
            bool allValues = (startTime == DateTime.MinValue && duration == TimeSpan.MaxValue);
            DateTime endTime = allValues ? DateTime.MaxValue : startTime + duration;

            lock (data)
            {
                foreach (Message message in data)
                {
                    var timestamp = message.Timestamp;
                    bool include = allValues;
                    if (!include)
                    {
                        include = (timestamp >= startTime && timestamp <= endTime);
                    }
                    if (include)
                    {
                        yield return message;
                    }
                }
            }
        }

        public IEnumerable<LogEntry> GetRows(string typeName, DateTime startTime, TimeSpan duration)
        {
            foreach (var msg in GetMessages(startTime, duration))
            {
                object typedValue = msg.TypedValue;

                if (typedValue != null)
                {
                    if (typeName == "GPS" )
                    {
                        // do auto-conversion from MAVLink.mavlink_global_position_int_t
                        if (typedValue is MAVLink.mavlink_global_position_int_t)
                        {
                            MAVLink.mavlink_global_position_int_t gps = (MAVLink.mavlink_global_position_int_t)typedValue;
                            LogEntry e = new LogEntry();
                            e.SetField("GPSTime", (ulong)gps.time_boot_ms * 1000); // must be in microseconds.
                            e.SetField("Lat", (double)gps.lat / 1E7);
                            e.SetField("Lon", (double)gps.lon / 1E7);
                            e.SetField("Alt", (float)((double)gps.alt / 1000));
                            e.SetField("nSat", 9); // fake values so the map works
                            e.SetField("EPH", 1);
                            yield return e;
                        }
                    }
                }
            }
        }

        List<MavlinkQueryEnumerator> queryEnumerators = new List<MavlinkQueryEnumerator>();

        internal Message AddMessage(MavLinkMessage e)
        {
            if (this.data == null)
            {
                this.data = new List<Message>();
            }

            DateTime time = epoch.AddMilliseconds((double)e.Time / (double)1000);

            Message msg = new Model.MavlinkLog.Message()
            {
                Msg = e,
                Timestamp = time,
                Ticks = e.Time
            };

            if (e.TypedPayload == null) {
                Type msgType = MavLinkMessage.GetMavlinkType((uint)e.MsgId);
                if (msgType != null)
                {
                    byte[] msgBuf = new byte[256];
                    GCHandle handle = GCHandle.Alloc(msgBuf, GCHandleType.Pinned);
                    IntPtr ptr = handle.AddrOfPinnedObject();

                    // convert to proper mavlink structure.
                    msg.TypedValue = Marshal.PtrToStructure(ptr, msgType);

                    handle.Free();
                }
            }
            else
            {
                msg.TypedValue = e.TypedPayload;
            }

            lock (data)
            {
                this.data.Add(msg);
                
                if (msg.TypedValue is MAVLink.mavlink_param_value_t)
                {
                    MAVLink.mavlink_param_value_t param = (MAVLink.mavlink_param_value_t)msg.TypedValue;
                    string name = ConvertToString(param.param_id);
                    parameters[name] = param;
                    Debug.WriteLine("{0}={1}", name, param.param_value);
                }
            }
            if (msg.TypedValue != null)
            {
                CreateSchema(msg.TypedValue);
            }

            lock (queryEnumerators)
            {
                foreach (var q in queryEnumerators)
                {
                    q.Add(msg);
                }
            }
            return msg;
        }

        public DateTime GetTime(ulong timeMs)
        {
            return startTime.AddMilliseconds(timeMs);
        }


        private double ConvertToNumeric(object value)
        {
            if (value == null) return 0;
            if (value is Array)
            {
                Array a = (Array)value;
                if (a.Length > 0)
                {
                    // todo: need a way for user to retrieve specific indexes from this array...
                    object x = a.GetValue(0);
                    return Convert.ToDouble(x);
                }
                return 0;
            }
            return Convert.ToDouble(value);
        }

        private string ConvertToString(byte[] a)
        {
            StringBuilder sb = new StringBuilder();
            for (int i = 0, n = a.Length; i < n; i++)
            {
                byte b = a[i];
                if (b >= 0x20)
                {
                    sb.Append(Convert.ToChar(b));
                }
            }
            return sb.ToString();
        }

        public IEnumerable<string> GetStatusMessages()
        { 
            // compute the min/max servo settings.
            foreach (Message msg in this.data)
            {
                if (msg.TypedValue is MAVLink.mavlink_statustext_t)
                {
                    mavlink_statustext_t2 status = new mavlink_statustext_t2((MAVLink.mavlink_statustext_t)msg.TypedValue);
                    if (!string.IsNullOrEmpty(status.text))
                    {
                        yield return status.text;
                    }
                }
            }
        }
        

        List<Flight> flights;

        public IEnumerable<Flight> GetFlights()
        {
            if (flights == null)
            {
                flights = new List<Model.Flight>();
                int min = int.MaxValue;
                int max = int.MinValue;

                // mavlink_servo_output_raw_t is an actual PWM signal, so it toggles between RC0_TRIM (usually 1500) and the
                // values it is sending to the motor, so we have to weed out the trim values in order to see the actual
                // values.
                int trim = 1500; // should get this from the parameter values.

                // compute the min/max servo settings.
                foreach (var msg in this.data)
                {
                    if (msg.TypedValue is MAVLink.mavlink_servo_output_raw_t)
                    {
                        MAVLink.mavlink_servo_output_raw_t servo = (MAVLink.mavlink_servo_output_raw_t)msg.TypedValue;
                        if (servo.servo1_raw != 0 && servo.servo1_raw != trim)
                        {
                            min = Math.Min(min, servo.servo1_raw);
                            max = Math.Max(max, servo.servo1_raw);
                        }
                    }
                }

                // find flights
                DateTime start = this.startTime;
                DateTime endTime = start;
                Flight current = null;
                int offCount = 0;
                // compute the min/max servo settings.
                foreach (var msg in this.data)
                {
                    if (msg.TypedValue is MAVLink.mavlink_servo_output_raw_t)
                    {
                        MAVLink.mavlink_servo_output_raw_t servo = (MAVLink.mavlink_servo_output_raw_t)msg.TypedValue;
                        endTime = start.AddMilliseconds(servo.time_usec / 1000);
                        if (servo.servo1_raw != trim)
                        {
                            if (servo.servo1_raw > min)
                            {
                                if (current == null)
                                {
                                    current = new Flight()
                                    {
                                        Log = this,
                                        StartTime = start.AddMilliseconds(servo.time_usec / 1000)
                                    };
                                    flights.Add(current);
                                    offCount = 0;
                                }
                            }
                            else if (servo.servo1_raw == min && offCount++ > 10)
                            {
                                if (current != null)
                                {
                                    current.Duration = endTime - current.StartTime;
                                    current = null;
                                }
                            }
                        }
                    }
                }
                if (current != null)
                {
                    current.Duration = endTime - current.StartTime;
                }

            }

            return flights;
        }

        static DateTime epoch = new DateTime(1970, 1, 1);

        public async Task Load(string file, ProgressUtility progress)
        {
            this.file = file;
            this.startTime = DateTime.MinValue;
            this.duration = TimeSpan.Zero;
            bool first = true;
            // QT time is milliseconds from the following epoch.
            const int BUFFER_SIZE = 8000;
            byte[] msgBuf = new byte[BUFFER_SIZE];
            GCHandle handle = GCHandle.Alloc(msgBuf, GCHandleType.Pinned);
            IntPtr ptr = handle.AddrOfPinnedObject();
            
            DateTime lastTime = DateTime.MinValue;

            await Task.Run(() =>
            {
                long length = 0;
                // MAVLINK_MSG_ID
                using (Stream s = File.OpenRead(file))
                {
                    length = s.Length;
                    BinaryReader reader = new BinaryReader(s);
                    while (s.Position < length)
                    {
                        progress.ShowProgress(0, length, s.Position);
                        try
                        {
                            MavLinkMessage header = new MavLinkMessage();
                            header.ReadHeader(reader);

                            Array.Clear(msgBuf, 0, BUFFER_SIZE);
                            int read = s.Read(msgBuf, 0, header.Length);

                            if (read == header.Length)
                            {
                                int id = (int)header.MsgId;
                                header.Crc = reader.ReadUInt16();

                                bool checkCrc = true;
                                if (id == MAVLink.mavlink_telemetry.MessageId)
                                {
                                    if (header.Crc == 0)  // telemetry has no crc.
                                    {
                                        checkCrc = false;
                                        continue;
                                    }                                     
                                }

                                if (checkCrc && !header.IsValidCrc(msgBuf, read))
                                {
                                    continue;
                                }

                                Type msgType = MavLinkMessage.GetMavlinkType((uint)header.MsgId);
                                if (msgType != null)
                                {
                                    // convert to proper mavlink structure.
                                    header.TypedPayload = Marshal.PtrToStructure(ptr, msgType);
                                }

                                Message message = AddMessage(header);
                                if (first)
                                {
                                    startTime = message.Timestamp;
                                    startTicks = message.Ticks;
                                    first = false;
                                }
                            }
                        }
                        catch
                        {
                            // try and continue...
                        }
                    }
                }
                progress.ShowProgress(0, length, length);
                handle.Free();
            });
            this.duration = lastTime - startTime;
        }

        private void CreateSchema(object message)
        {
            Type t = message.GetType();
            if (schemaCache.ContainsKey(t))
            {
                return;
            }

            string name = t.Name;
            if (name.StartsWith("mavlink_"))
            {
                name = name.Substring(8);
            }
            else if (name.StartsWith("mavlink"))
            {
                name = name.Substring(7);
            }

            LogItemSchema item = new LogItemSchema() { Name = name, Type = t.Name };

            foreach (FieldInfo fi in t.GetFields(BindingFlags.Public | BindingFlags.Instance))
            {
                var field = new LogItemSchema() { Name = fi.Name, Type = fi.FieldType.Name };
                item.AddChild(field);

                object value = fi.GetValue(message);
                // byte[] array is special, we return that as binhex encoded binary data.
                if (fi.FieldType.IsArray && fi.FieldType != typeof(byte[]))
                {
                    field.IsArray = true;
                    Type itemType = fi.FieldType.GetElementType();
                    Array a = (Array)value;
                    for (int i = 0, n = a.Length; i < n; i++)
                    {
                        field.AddChild(new LogItemSchema() { Name = i.ToString(), Type = itemType.Name });
                    }
                }
            }
            schemaCache[t] = item;

            schema.AddChild(item);

            if (SchemaChanged != null)
            {
                SchemaChanged(this, EventArgs.Empty);
            }
        }

        public event EventHandler SchemaChanged;

        internal void Clear()
        {
            this.data = new List<Message>();
            schema = new LogItemSchema() { Name = "MavlinkLog", Type = "Root" };
            schemaCache = new Dictionary<Type, LogItemSchema>();
        }

        // text strings well known states

        static string GetSystemStateName(int value)
        {
            if (value >= 0 && value < MavSystemStateNames.Length)
            {
                return MavSystemStateNames[value];
            }
            return null;
        }

        static string[] MavSystemStateNames = {
            "MAV_STATE_UNINIT",
            "MAV_STATE_BOOT",
            "MAV_STATE_CALIBRATING",
            "MAV_STATE_STANDBY",
            "MAV_STATE_ACTIVE",
            "MAV_STATE_CRITICAL",
            "MAV_STATE_EMERGENCY",
            "MAV_STATE_POWEROFF"
        };

        enum PX4_CUSTOM_MAIN_MODE
        {
            PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
            PX4_CUSTOM_MAIN_MODE_ALTCTL,
            PX4_CUSTOM_MAIN_MODE_POSCTL,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_MAIN_MODE_ACRO,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            PX4_CUSTOM_MAIN_MODE_STABILIZED,
            PX4_CUSTOM_MAIN_MODE_RATTITUDE,
            LAST_VALUE
        };

        enum PX4_CUSTOM_SUB_MODE_AUTO
        {
            PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
            PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
            PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
            PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
            PX4_CUSTOM_SUB_MODE_AUTO_RTL,
            PX4_CUSTOM_SUB_MODE_AUTO_LAND,
            PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
            PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
            LAST_VALUE
        };

        // Enumeration of landed detector states
        enum MAV_LANDED_STATE
        {
            // MAV landed state is unknown
            MAV_LANDED_STATE_UNDEFINED = 0,
            // MAV is landed (on ground)
            MAV_LANDED_STATE_ON_GROUND = 1,
            // MAV is in air
            MAV_LANDED_STATE_IN_AIR = 2
        };

        static string[] MavLandedStateNames = {
            "MAV_LANDED_STATE_UNDEFINED",
            "MAV_LANDED_STATE_ON_GROUND",
            "MAV_LANDED_STATE_IN_AIR"
        };

        static string GetLandedStateName(int value)
        {
            if (value >= 0 && value < MavLandedStateNames.Length)
            {
                return MavLandedStateNames[value];
            }
            return null;
        }

        // Enumeration of VTOL states
        static string[] MavVTOLStateNames =
        {
            "MAV_VTOL_STATE_UNDEFINED",
            "MAV_VTOL_STATE_TRANSITION_TO_FW",
            "MAV_VTOL_STATE_TRANSITION_TO_MC",
            "MAV_VTOL_STATE_MC",
            "MAV_VTOL_STATE_FW"
        };

        static string GetVTolStateName(int value)
        {
            if (value >= 0 && value < MavVTOLStateNames.Length)
            {
                return MavVTOLStateNames[value];
            }
            return null;
        }

        struct FlagName
        {
            public int Flag;
            public string Name;
        };

        static string GetCustomModeFlags(uint value)
        {
            uint custom = (value >> 16);
            uint mode = (custom & 0xff);
            uint submode = (custom >> 8);

            StringBuilder sb = new StringBuilder();
            if (mode >= 0 && mode <= (uint)PX4_CUSTOM_MAIN_MODE.LAST_VALUE)
            {
                PX4_CUSTOM_MAIN_MODE mm = (PX4_CUSTOM_MAIN_MODE)mode;
                sb.AppendLine(mm.ToString());
            }
            if (mode >= 0 && mode <= (uint)PX4_CUSTOM_SUB_MODE_AUTO.LAST_VALUE)
            {
                PX4_CUSTOM_SUB_MODE_AUTO sm = (PX4_CUSTOM_SUB_MODE_AUTO)submode;
                sb.AppendLine(sm.ToString());
            }
            return sb.ToString();
        }


        static string GetFlags(FlagName[] flags, uint value)
        {
            StringBuilder sb = new StringBuilder();

            foreach (FlagName f in flags)
            {
                if ((value & f.Flag) != 0)
                {
                    sb.AppendLine(f.Name);
                }
            }

            return sb.ToString();
        }

        static string GetModeFlags(uint value)
        {
            return GetFlags(ModeFlagNames, value);
        }

        static FlagName[] ModeFlagNames = {
	        new FlagName() { Flag = (int)MAVLink.MAV_MODE_FLAG.TEST_ENABLED,          Name = "MAV_MODE_FLAG_TEST_ENABLED" },
	        new FlagName() { Flag = (int)MAVLink.MAV_MODE_FLAG.AUTO_ENABLED,          Name = "MAV_MODE_FLAG_AUTO_ENABLED" },
	        new FlagName() { Flag = (int)MAVLink.MAV_MODE_FLAG.GUIDED_ENABLED,        Name = "MAV_MODE_FLAG_GUIDED_ENABLED" },
	        new FlagName() { Flag = (int)MAVLink.MAV_MODE_FLAG.STABILIZE_ENABLED,     Name = "MAV_MODE_FLAG_STABILIZE_ENABLED" },
	        new FlagName() { Flag = (int)MAVLink.MAV_MODE_FLAG.HIL_ENABLED,           Name = "MAV_MODE_FLAG_HIL_ENABLED" },
	        new FlagName() { Flag = (int)MAVLink.MAV_MODE_FLAG.MANUAL_INPUT_ENABLED,  Name = "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED" },
	        new FlagName() { Flag = (int)MAVLink.MAV_MODE_FLAG.SAFETY_ARMED,          Name = "MAV_MODE_FLAG_SAFETY_ARMED" }
        };

    }
}
