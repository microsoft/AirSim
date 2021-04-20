using LogViewer.Controls;
using LogViewer.Utilities;
using MissionPlanner.Log;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace LogViewer.Model
{
    public class LogField
    {
        public string Name { get; set; }
        public object Value { get; set; }
        public LogEntry Parent { get; internal set; }
    }

    public class LogEntry : LogField
    {
        public ulong Timestamp { get; set; }
                
        /// <summary>
        /// Return a data value, and try and timestamp in the x-coordinate
        /// </summary>
        /// <param name="field"></param>
        /// <returns></returns>
        public virtual DataValue GetDataValue(string field)
        {
            LogField result = null;
            if (cache == null)
            {
                cache = new Dictionary<string, Model.LogField>();
                // crack the blob
                ParseFields();
            }
            if (!cache.TryGetValue(field, out result))
            {
                return null;
            }
            if (result != null)
            {
                DataValue dv = new DataValue() { X = Timestamp, Y = 0, UserData = result };
                try
                {
                    if (result.Value is double)
                    {
                        dv.Y = (double)result.Value;
                    }
                    if (result.Value is string)
                    {
                        double y = 0;
                        double.TryParse((string)result.Value, out y);
                        dv.Y = y;
                    }
                }
                catch { }

                if (result.Value is string)
                {
                    dv.Label = (string)result.Value;
                }
                else
                {
                    dv.Label = GetLabel(result);
                }
                return dv;
            }
            return null;
        }

        private string GetLabel(LogField result)
        {
            // auto-populate some well known PX4 enums.
            if (this.Name == "STAT")
            {
                if (result.Name == "MainState")
                {
                    uint v = Convert.ToUInt32(result.Value);
                    if (v < (uint)PX4_MAIN_STATE.MAIN_STATE_MAX)
                    {
                        return string.Format("{0} ({1})", ((PX4_MAIN_STATE)v).ToString(), v);
                    }
                }
                else if (result.Name == "NavState")
                {
                    uint v = Convert.ToUInt32(result.Value);
                    if (v < (uint)PX4_NAV_STATE.NAVIGATION_STATE_MAX)
                    {
                        return string.Format("{0} ({1})", ((PX4_NAV_STATE)v).ToString(), v);
                    }
                }
                else if (result.Name == "ArmS")
                {
                    uint v = Convert.ToUInt32(result.Value);
                    if (v < (uint)PX4_ARM_STATE.ARMING_STATE_MAX)
                    {
                        return string.Format("{0} ({1})", ((PX4_ARM_STATE)v).ToString(), v);
                    }
                }
            }
            return null;
        }

        enum PX4_ARM_STATE
        {
            ARMING_STATE_INIT = 0,
            ARMING_STATE_STANDBY = 1,
            ARMING_STATE_ARMED = 2,
            ARMING_STATE_ARMED_ERROR = 3,
            ARMING_STATE_STANDBY_ERROR = 4,
            ARMING_STATE_REBOOT = 5,
            ARMING_STATE_IN_AIR_RESTORE = 6,
            ARMING_STATE_MAX = 7
        }

        enum PX4_MAIN_STATE
        {
            MAIN_STATE_MANUAL = 0,
            MAIN_STATE_ALTCTL = 1,
            MAIN_STATE_POSCTL = 2,
            MAIN_STATE_AUTO_MISSION = 3,
            MAIN_STATE_AUTO_LOITER = 4,
            MAIN_STATE_AUTO_RTL = 5,
            MAIN_STATE_ACRO = 6,
            MAIN_STATE_OFFBOARD = 7,
            MAIN_STATE_STAB = 8,
            MAIN_STATE_RATTITUDE = 9,
            MAIN_STATE_AUTO_TAKEOFF = 10,
            MAIN_STATE_AUTO_LAND = 11,
            MAIN_STATE_AUTO_FOLLOW_TARGET = 12,
            MAIN_STATE_MAX = 13
        }

        enum PX4_NAV_STATE
        {
            NAVIGATION_STATE_MANUAL = 0,
            NAVIGATION_STATE_ALTCTL = 1,
            NAVIGATION_STATE_POSCTL = 2,
            NAVIGATION_STATE_AUTO_MISSION = 3,
            NAVIGATION_STATE_AUTO_LOITER = 4,
            NAVIGATION_STATE_AUTO_RTL = 5,
            NAVIGATION_STATE_AUTO_RCRECOVER = 6,
            NAVIGATION_STATE_AUTO_RTGS = 7,
            NAVIGATION_STATE_AUTO_LANDENGFAIL = 8,
            NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9,
            NAVIGATION_STATE_ACRO = 10,
            NAVIGATION_STATE_UNUSED = 11,
            NAVIGATION_STATE_DESCEND = 12,
            NAVIGATION_STATE_TERMINATION = 13,
            NAVIGATION_STATE_OFFBOARD = 14,
            NAVIGATION_STATE_STAB = 15,
            NAVIGATION_STATE_RATTITUDE = 16,
            NAVIGATION_STATE_AUTO_TAKEOFF = 17,
            NAVIGATION_STATE_AUTO_LAND = 18,
            NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19,
            NAVIGATION_STATE_MAX = 20
        }

        public bool HasFields
        {
            get
            {
                if (cache == null)
                {
                    cache = new Dictionary<string, Model.LogField>();
                    // crack the blob
                    ParseFields();
                }
                return this.cache != null;
            }
        }

        public IEnumerable<LogField> GetFields()
        {
            if (cache == null)
            {
                cache = new Dictionary<string, Model.LogField>();
                // crack the blob
                ParseFields();
            }
            if (this.cache != null)
            {
                return this.cache.Values;
            }
            return null;
        }

        public bool HasField(string name)
        {
            if (cache == null)
            {
                cache = new Dictionary<string, Model.LogField>();
                // crack the blob
                ParseFields();
            }
            return cache.ContainsKey(name);
        }

        public T GetField<T>(string name)
        {
            LogField result = null;
            if (cache == null)
            {
                cache = new Dictionary<string, Model.LogField>();
                // crack the blob
                ParseFields();
            }
            if (cache.TryGetValue(name, out result))
            {
                Type t = typeof(T);
                if (t == typeof(sbyte)) // b
                {
                    object d = Convert.ToSByte(result.Value);
                    return (T)d;
                }
                else if (t == typeof(byte)) // B
                {
                    object d = Convert.ToByte(result.Value);
                    return (T)d;
                }
                else if (t == typeof(Int16)) // h
                {
                    object d = Convert.ToInt16(result.Value);
                    return (T)d;
                }
                else if (t == typeof(UInt16)) // H
                {
                    object d = Convert.ToUInt16(result.Value);
                    return (T)d;
                }
                else if (t == typeof(Int32)) // i
                {
                    object d = Convert.ToInt32(result.Value);
                    return (T)d;
                }
                else if (t == typeof(UInt32)) // I
                {
                    object d = Convert.ToUInt32(result.Value);
                    return (T)d;
                }
                else if (t == typeof(Int64)) // q
                {
                    object d = Convert.ToInt64(result.Value);
                    return (T)d;
                }
                else if (t == typeof(UInt64)) // Q
                {
                    try
                    {
                        object d = Convert.ToUInt64(result.Value);
                        return (T)d;
                    }
                    catch
                    {
                    }
                }
                else if (t == typeof(float)) // f
                {
                    object d = Convert.ToSingle(result.Value);
                    return (T)d;
                }
                else if (t == typeof(double)) // d
                {
                    object d = Convert.ToDouble(result.Value);
                    return (T)d;
                }
                else if (t == typeof(string)) // n, N, Z
                {
                    object d = Convert.ToString(result.Value);
                    return (T)d;
                }
            }
            return default(T);
        }

        public LogEntryFMT Format { get; set; }

        public byte[] Blob { get; set; }

        public static string ReadAsciiString(BinaryReader reader, int len)
        {
            byte[] buffer = reader.ReadBytes(len);
            return System.Text.ASCIIEncoding.ASCII.GetString(buffer, 0, buffer.Length).Trim('\0');
        }

        // Useful for conversion from mavlink types to our log format.
        internal void SetField(string name, object value)
        {
            if (cache == null)
            {
                cache = new Dictionary<string, LogField>();
            }
            cache[name] = new LogField()
            {
                Name = name,
                Value = value,
                Parent = this
            };
        }

        private void ParseFields()
        {
            if (Format == null)
            {
                return;
            }
            int offset = 0;
            byte[] message = this.Blob;
            string format = Format.FormatString;
            for (int k = 0, n = format.Length; k < n; k++)
            {
                char ch = format[k];
                object value = null;
                switch (ch)
                {
                    case 'b':
                        value = (sbyte)message[offset];
                        offset++;
                        break;
                    case 'B':
                        value = message[offset];
                        offset++;
                        break;
                    case 'h':
                        value = BitConverter.ToInt16(message, offset);
                        offset += 2;
                        break;
                    case 'H':
                        value = BitConverter.ToUInt16(message, offset);
                        offset += 2;
                        break;
                    case 'i':
                        value = BitConverter.ToInt32(message, offset);
                        offset += 4;
                        break;
                    case 'I':
                        value = BitConverter.ToUInt32(message, offset);
                        offset += 4;
                        break;
                    case 'q':
                        value = BitConverter.ToInt64(message, offset);
                        offset += 8;
                        break;
                    case 'Q':
                        value = BitConverter.ToUInt64(message, offset);
                        offset += 8;
                        break;
                    case 'f':
                        value = BitConverter.ToSingle(message, offset);
                        offset += 4;
                        break;
                    case 'd':
                        value = BitConverter.ToDouble(message, offset);
                        offset += 8;
                        break;
                    case 'c':
                        value = (BitConverter.ToInt16(message, offset) / 100.0);
                        offset += 2;
                        break;
                    case 'C':
                        value = (BitConverter.ToUInt16(message, offset) / 100.0);
                        offset += 2;
                        break;
                    case 'e':
                        value = (BitConverter.ToInt32(message, offset) / 100.0);
                        offset += 4;
                        break;
                    case 'E':
                        value = (BitConverter.ToUInt32(message, offset) / 100.0);
                        offset += 4;
                        break;
                    case 'L':
                        value = ((double)BitConverter.ToInt32(message, offset) / 10000000.0);
                        offset += 4;
                        break;
                    case 'n':
                        value = ASCIIEncoding.ASCII.GetString(message, offset, 4).Trim(NullTerminator);
                        offset += 4;
                        break;
                    case 'N':
                        value = ASCIIEncoding.ASCII.GetString(message, offset, 16).Trim(NullTerminator);
                        offset += 16;
                        break;
                    case 'M':
                        int modeno = message[offset];
                        value = modeno;
                        offset++;
                        break;
                    case 'Z':
                        value = ASCIIEncoding.ASCII.GetString(message, offset, 64).Trim(NullTerminator);
                        offset += 64;
                        break;
                    default:
                        throw new Exception(string.Format("Unexpected format specifier '{0}'", ch));
                }
                if (k < Format.Columns.Length)
                {
                    cache[Format.Columns[k]] = new LogField()
                    {
                        Name = Format.Columns[k],
                        Value = value,
                        Parent = this
                    };
                }
            }
        }

        static char[] NullTerminator = new char[] { '\0' };


        private Dictionary<String, LogField> cache;
    }

    class Px4DataLog : IDataLog
    {
        string file;
        LogType logType;
        DateTime startTime;
        TimeSpan duration = TimeSpan.Zero;
        List<LogEntry> data;
        LogItemSchema schema = new LogItemSchema() { Name = "Px4DataLog", Type = "Root" };

        public DateTime StartTime { get { return this.startTime; } }

        public TimeSpan Duration
        {
            get { return duration; }
        }

        public LogItemSchema Schema { get { return schema; } }

        public IEnumerable<DataValue> GetDataValues(LogItemSchema schema, DateTime startTime, TimeSpan duration)
        {
            if (data != null && schema.Parent != null)
            {
                foreach (LogEntry entry in GetRows(schema.Parent.Name, startTime, duration))
                {
                    var dv = entry.GetDataValue(schema.Name);
                    if (dv != null)
                    {
                        yield return dv;
                    }
                }
            }
        }


        public async Task Load(string file, ProgressUtility progress)
        {
            this.file = file;
            string ext = System.IO.Path.GetExtension(file).ToLowerInvariant();
            if (ext == ".bin")
            {
                this.logType = LogType.binFile;
            }
            else if (ext == ".px4log")
            {
                this.logType = LogType.px4logFile;
            }

            bool hasStartTime = false;
            this.startTime = DateTime.MinValue;

            List<LogEntry> rows = new List<LogEntry>();

            await Task.Run(() =>
            {
                using (Stream s = File.OpenRead(file))
                {
                    BinaryReader reader = new BinaryReader(s);
                    PX4BinaryLog log = new PX4BinaryLog(this.logType);
                    log.GenerateParser = false;
                    DateTime? gpsStartTime = null;
                    ulong gpsAbsoluteOffset = 0;
                    ulong logStartTime = 0;

                    while (s.Position < s.Length)
                    {
                        progress.ShowProgress(0, s.Length, s.Position);

                        LogEntry row = log.ReadMessageObjects(s) as LogEntry;
                        if (row == null)
                        {
                            // has no system clock value yet, so skip it.
                            continue;
                        }

                        if (!hasStartTime && log.CurrentTime != 0)
                        {
                            hasStartTime = true;
                            logStartTime = log.CurrentTime;
                            this.startTime = GetTime(log.CurrentTime);
                        }
                        if (!gpsStartTime.HasValue && row.Name == "GPS")
                        {
                            ulong time = row.GetField<ulong>("GPSTime");
                            if (time > 0)
                            {
                                // ok, we got a real GPS time
                                gpsStartTime = GetTime(time);
                                // backtrack and fix up initial rows.
                                if (hasStartTime)
                                {
                                    // compute offset from PX4 boot time (which is the log.CurrentTime) and the absolute GPS real time clock
                                    // so we can add this offset to PX4 log.CurrentTime so all our times from here on out are in real time.
                                    gpsStartTime = gpsStartTime.Value.AddMilliseconds((double)logStartTime - (double)log.CurrentTime);
                                }
                                ulong linuxEpochMicroseconds = ((ulong)(gpsStartTime.Value.ToUniversalTime() - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000);
                                gpsAbsoluteOffset = linuxEpochMicroseconds - logStartTime;
                                this.startTime = gpsStartTime.Value;
                                // and fix existing log entries
                                foreach (LogEntry e in rows)
                                {
                                    // add GPS absolute offset to the timestamp.
                                    e.Timestamp += gpsAbsoluteOffset;
                                }
                                hasStartTime = true;
                            }
                        }

                        if (gpsStartTime.HasValue)
                        {
                            // add GPS absolute offset to the timestamp.
                            row.Timestamp += gpsAbsoluteOffset;
                        }

                        rows.Add(row);

                        if (row.Format.Name == "PARM")
                        {
                            string name = row.GetField<string>("Name");
                            float value = row.GetField<float>("Value");
                            Debug.WriteLine("{0}={1}", name, value);
                        }
                    }
                    if (log.CurrentTime != 0)
                    {
                        DateTime endTime = GetTime(log.CurrentTime + gpsAbsoluteOffset);
                        this.duration = endTime - startTime;
                        Debug.WriteLine("StartTime={0}, EndTime={1}, Duration={2}", startTime.ToString(), endTime.ToString(), duration.ToString());
                    }

                    CreateSchema(log);
                }
            });

            this.data = rows;
        }

        private void CreateSchema(PX4BinaryLog log)
        {
            LogItemSchema schema = new LogItemSchema() { Name = "Px4DataLog", Type = "Root" };

            foreach (var fmt in log.GetFormats())
            {
                LogItemSchema element = new LogItemSchema() { Name = fmt.Name };

                int i = 0;
                foreach (var c in fmt.Columns)
                {
                    LogItemSchema column = new LogItemSchema() { Name = c };
                    if (i < fmt.FormatString.Length)
                    {
                        column.Type = GetTypeName(fmt.FormatString[i]);
                    }
                    i++;
                    element.AddChild(column);
                }
                schema.AddChild(element);
            }

            this.schema = schema;
        }

        private string GetTypeName(char ch)
        {
            switch (ch)
            {
                case 'b':
                    return "sbyte";
                case 'B':
                    return "byte";
                case 'h':
                    return "Int16";
                case 'H':
                    return "UInt16";
                case 'i':
                    return "Int32";
                case 'I':
                    return "UInt32";
                case 'q':
                    return "Int64";
                case 'Q':
                    return "UInt64";
                case 'f':
                    return "Single";
                case 'd':
                    return "Double";
                case 'c':
                    return "Double";
                case 'C':
                    return "Double";
                case 'e':
                    return "Double";
                case 'E':
                    return "Double";
                case 'L':
                    return "Double";
                case 'n':
                    return "string";
                case 'N':
                    return "string";
                case 'M':
                    return "string";
                case 'Z':
                    return "string";
                default:
                    return "null";
            }
        }

        public IEnumerable<Flight> GetFlights()
        {
            List<ulong> times = new List<ulong>();
            List<float> motor1History = new List<float>();
            float min = float.MaxValue;
            float max = float.MinValue;

            foreach (var row in this.data)
            {
                if (row.Name == "OUT0")
                {
                    DataValue f = row.GetDataValue("Out0");
                    if (f != null)
                    {
                        ulong timeMs = row.Timestamp;
                        float motor1 = (float)f.Y;
                        if (motor1 < min)
                        {
                            min = motor1;
                        }
                        if (motor1 > max)
                        {
                            max = motor1;
                        }
                        times.Add(timeMs);
                        motor1History.Add(motor1);
                    }
                }
            }
            ulong clock = 0;
            ulong start = 0;
            for (int i = 0, n = times.Count; i < n; i++)
            {
                clock = times[i];
                float m = motor1History[i];
                if (m > min)
                {
                    if (start == 0)
                    {
                        start = clock;
                    }
                }
                else if (start > 0)
                {
                    DateTime st = GetTime((ulong)start);
                    DateTime et = GetTime((ulong)clock);
                    yield return new Flight()
                    {
                        Log = this,
                        StartTime = st,
                        Duration = et - st
                    };
                    start = 0;
                }
            }
            if (start > 0)
            {
                DateTime st = GetTime((ulong)start);
                DateTime et = GetTime((ulong)clock);
                yield return new Flight()
                {
                    Log = this,
                    StartTime = st,
                    Duration = et - st
                };
                start = 0;
            }

        }

        public DateTime GetTime(ulong timeUs)
        {
            return new DateTime(1970, 1, 1).AddMilliseconds(timeUs / 1000).ToLocalTime();
        }

        public DateTime GetGpsTime(int week, ulong timeUs)
        {
            // as of 2016, GPS time is ahead by 17 leap seconds.
            // see http://www.leapsecond.com/java/gpsclock.htm
            uint leapMs = 17 * 1000;
            // not correct for leap seconds  day   days  weeks  seconds
            var basetime = new DateTime(1980, 1, 6, 0, 0, 0, DateTimeKind.Utc);
            basetime = basetime.AddDays(week * 7);
            var timeMs = timeUs / 1000;
            if (timeMs < leapMs)
            {
                // hmmm, odd value then...
                basetime = basetime.AddMilliseconds(timeMs);
            }
            else
            {
                basetime = basetime.AddMilliseconds(timeMs - leapMs);
            }
            return basetime.ToLocalTime();
        }


        public IEnumerable<LogEntry> GetRows(string typeName, DateTime startTime, TimeSpan duration)
        {
            if (data != null)
            {
                bool allValues = (startTime == DateTime.MinValue && duration == TimeSpan.MaxValue);
                DateTime endTime = allValues ? DateTime.MaxValue : startTime + duration;
                bool include = allValues;

                foreach (var row in data)
                {
                    if (!allValues)
                    {
                        // check if we are in the specified time range according to LogEntryRCOU values.
                        // (since LogEntryRCOU is what we used to create Flight time information)
                        if (row.Timestamp != 0)
                        {
                            DateTime date = GetTime(row.Timestamp);
                            include = (date >= startTime && date <= endTime);
                        }
                    }
                    if (row.Name == typeName && include)
                    {
                        yield return row;
                    }
                }
            }
        }

        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            throw new NotImplementedException("LiveQuery");
        }

    }

}
