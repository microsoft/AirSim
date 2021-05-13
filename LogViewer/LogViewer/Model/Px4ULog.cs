using LogViewer.Utilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace LogViewer.Model.ULog
{
    public abstract class Message
    {
        public abstract ulong GetTimestamp();
    }

    public enum FieldType
    {
        None,
        Float,
        Double,
        Int8,
        Bool,
        UInt8,
        Int16,
        UInt16,
        Int32,
        UInt32,
        Int64,
        UInt64,
        Char,
        Struct
    };


    public class MessageField
    {
        public FieldType type;
        public string name;
        public int arraySize;
        public string typeName;
        public MessageFormat structType;

        public override string ToString()
        {
            return typeName + (arraySize > 0 ? "[" + arraySize + "] " : " ") + name;
        }

        public static string GetFieldTypeName(FieldType f)
        {
            switch (f)
            {
                case FieldType.Float:
                    return "float";
                case FieldType.Double:
                    return "double";
                case FieldType.Int8:
                    return "int8_t";
                case FieldType.Bool:
                    return "bool";
                case FieldType.UInt8:
                    return "uint8_t";
                case FieldType.Int16:
                    return "int16_t";
                case FieldType.UInt16:
                    return "uint16_t";
                case FieldType.Int32:
                    return "int32_t";
                case FieldType.UInt32:
                    return "uint32_t";
                case FieldType.Int64:
                    return "int64_t";
                case FieldType.UInt64:
                    return "uint64_t";
                case FieldType.Char:
                    return "char";
                case FieldType.Struct:
                    break;
            }
            throw new Exception("Unexpected FieldType");
        }

        public MessageField(string definition)
        {
            string[] parts = definition.Split(' ');
            typeName = parts[0];

            int i = typeName.IndexOf('[');
            if (i > 0)
            {
                // an array
                i++;
                int j = typeName.IndexOf(']', i);
                if (j > 0)
                {
                    string s = typeName.Substring(i, j - i);
                    arraySize = int.Parse(s);
                    typeName = typeName.Substring(0, i - 1);
                }
                else
                {
                    throw new FormatException("Badly formatted array syntax in type definition: " + definition);
                }
            }
            switch (typeName)
            {
                case "float":
                    type = FieldType.Float;
                    break;
                case "double":
                    type = FieldType.Double;
                    break;
                case "int8_t":
                    type = FieldType.Int8;
                    break;
                case "bool":
                    type = FieldType.Bool;
                    break;
                case "uint8_t":
                    type = FieldType.UInt8;
                    break;
                case "int16_t":
                    type = FieldType.Int16;
                    break;
                case "uint16_t":
                    type = FieldType.UInt16;
                    break;
                case "int32_t":
                    type = FieldType.Int32;
                    break;
                case "uint32_t":
                    type = FieldType.UInt32;
                    break;
                case "int64_t":
                    type = FieldType.Int64;
                    break;
                case "uint64_t":
                    type = FieldType.UInt64;
                    break;
                case "char":
                    type = FieldType.Char;
                    break;
                default:
                    type = FieldType.Struct;
                    break;
            }
            if (type != FieldType.Struct)
            {
                typeName = type.ToString(); // normalize to match LogItemSchema names.
            }
            if (parts.Length > 1)
            {
                name = parts[1];
            }
        }

        internal void Resolve(Dictionary<string, MessageFormat> formats)
        {
            if (type == FieldType.Struct)
            {
                if (!formats.TryGetValue(typeName, out structType))
                {
                    throw new FormatException("Undefined struct name: " + typeName);
                }
            }
        }
    };

    public class MessageFormat : Message
    {
        public string fmt;
        public string name;
        public int id;
        public List<MessageField> fields = new List<MessageField>();

        public override ulong GetTimestamp()
        {
            return 0;
        }

        public MessageFormat(string name, List<MessageField> fields)
        {
            this.fields = fields;
            this.name = name;
        }

        public MessageFormat(string fmt)
        {
            this.fmt = fmt;
            string[] parts = fmt.Split(':');
            name = parts[0];
            if (parts.Length > 1)
            {
                string[] fieldStrings = parts[1].Split(';');
                foreach (string fs in fieldStrings)
                {
                    if (!string.IsNullOrWhiteSpace(fs))
                    {
                        var field = new MessageField(fs);
                        if (!field.name.StartsWith("_padding"))
                        {
                            fields.Add(field);
                        }
                    }
                }
            }
        }

        internal void Resolve(Dictionary<string, MessageFormat> formats)
        {
            foreach (var field in fields)
            {
                field.Resolve(formats);
            }
        }
    };

    class MessageLogging : Message
    {
        byte logLevel;
        long timestamp;
        string msg;
        ushort tag;

        public byte LogLevel => logLevel;
        public long Timestamp => timestamp;
        public string Contents => msg;
        public ushort Tag => tag;

        public MessageLogging(byte logLevel, long timestamp, string msg, ushort tag = 0)
        {
            this.logLevel = logLevel;
            this.timestamp = timestamp;
            this.msg = msg;
            this.tag = tag;
        }

        public override ulong GetTimestamp()
        {
            return (ulong)timestamp;
        }
    }
    
    class MessageData : Message
    {
        internal MessageSubscription subscription;
        internal MessageFormat format;
        UInt16 msgId;
        byte[] value;
        Dictionary<string, object> values;
        Dictionary<string, MessageFormat> nestedFormats;

        public MessageData(UInt16 msgId, byte[] value, MessageSubscription s)
        {
            this.msgId = msgId;
            this.value = value;
            this.subscription = s;
            this.format = s.format;
        }

        public override ulong GetTimestamp()
        {
            if (values == null)
            {
                ParseValues();
            }

            object ts = null;
            if (values.TryGetValue("timestamp", out ts))
            {
                return Convert.ToUInt64(ts);
            }
            return 0;
        }

        internal DataValue GetValue(MessageField field)
        {
            if (values == null)
            {
                ParseValues();
            }

            ulong x = GetTimestamp();
            double y = 0;

            object o = null;
            if (values.TryGetValue(field.name, out o))
            {
                y = Convert.ToDouble(o);
            }
            return new DataValue()
            {
                X = (double)x,
                Y = y,
                UserData = this,
                Label = GetLabel(field, y)
            };
        }


        public enum NavStates {
            NAVIGATION_STATE_MANUAL = 0, //		# Manual mode
            NAVIGATION_STATE_ALTCTL = 1, //		# Altitude control mode
            NAVIGATION_STATE_POSCTL = 2, //		# Position control mode
            NAVIGATION_STATE_AUTO_MISSION = 3, //		# Auto mission mode
            NAVIGATION_STATE_AUTO_LOITER = 4, //		# Auto loiter mode
            NAVIGATION_STATE_AUTO_RTL = 5, //		# Auto return to launch mode
            NAVIGATION_STATE_AUTO_LANDENGFAIL = 8, // 	# Auto land on engine failure
            NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9, //	# Auto land on gps failure (e.g. open loop loiter down)
            NAVIGATION_STATE_ACRO = 10, //		# Acro mode
            NAVIGATION_STATE_UNUSED = 11, //		# Free slot
            NAVIGATION_STATE_DESCEND = 12, //		# Descend mode (no position control)
            NAVIGATION_STATE_TERMINATION = 13, //		# Termination mode
            NAVIGATION_STATE_OFFBOARD = 14, //
            NAVIGATION_STATE_STAB = 15, //		# Stabilized mode
            NAVIGATION_STATE_UNUSED2 = 16, //		# Free slot
            NAVIGATION_STATE_AUTO_TAKEOFF = 17, //	# Takeoff
            NAVIGATION_STATE_AUTO_LAND = 18, //		# Land
            NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19, //	# Auto Follow
            NAVIGATION_STATE_AUTO_PRECLAND = 20, //	# Precision land with landing target
            NAVIGATION_STATE_ORBIT = 21, //       # Orbit in a circle
            NAVIGATION_STATE_MAX = 22, //
        };

        public enum ArmingStates {
            ARMING_STATE_INIT = 0,
            ARMING_STATE_STANDBY = 1,
            ARMING_STATE_ARMED = 2,
            ARMING_STATE_STANDBY_ERROR = 3,
            ARMING_STATE_SHUTDOWN = 4,
            ARMING_STATE_IN_AIR_RESTORE = 5,
            ARMING_STATE_MAX = 6
        }

        [Flags]
        public enum FailureDetectorFlags
        {
            FAILURE_NONE = 0, //
            FAILURE_ROLL = 1, // 	        # (1 << 0)
            FAILURE_PITCH = 2, //	        # (1 << 1)
            FAILURE_ALT = 4, // 	        # (1 << 2)
            FAILURE_EXT = 8, // 	        # (1 << 3)
            FAILURE_ARM_ESC = 16, //      # (1 << 4)
        }

        string GetLabel(MessageField field, double y)
        {
            if (this.format.name == "vehicle_status")
            {
                if (field.name == "nav_state")
                {
                    NavStates s = (NavStates)y;
                    return s.ToString();
                }
                else if (field.name == "arming_state")
                {
                    ArmingStates s = (ArmingStates)y;
                    return s.ToString();
                }
                else if (field.name == "failure_detector_status")
                {
                    FailureDetectorFlags s = (FailureDetectorFlags)y;
                    return s.ToString();
                }
            }
            return null;
        }

        private void ParseValues()
        {
            BinaryReader reader = new BinaryReader(new MemoryStream(value));
            ReadValues(reader);
        }

        private void ReadValues(BinaryReader reader)
        {
            values = new Dictionary<string, object>();
            foreach (var field in this.format.fields)
            {
                object value = null;
                if (field.arraySize > 0)
                {
                    object[] array = new object[field.arraySize];
                    for (int i = 0; i < field.arraySize; i++)
                    {
                        array[i] = ReadField(reader, field);
                    }
                    value = array;
                }
                else
                {
                    value = ReadField(reader, field);
                }
                values[field.name] = value;
            }
        }

        private object ReadField(BinaryReader reader, MessageField field)
        {
            object o = null;            
            switch (field.type)
            {
                case FieldType.Float:
                    o = reader.ReadSingle();
                    break;
                case FieldType.Double:
                    o = reader.ReadDouble();
                    break;
                case FieldType.Int8:
                    o = reader.ReadSByte();
                    break;
                case FieldType.Bool:
                    o = (reader.ReadByte() == 0) ? false : true;
                    break;
                case FieldType.UInt8:
                    o = reader.ReadByte();
                    break;
                case FieldType.Int16:
                    o = reader.ReadInt16();
                    break;
                case FieldType.UInt16:
                    o = reader.ReadUInt16();
                    break;
                case FieldType.Int32:
                    o = reader.ReadInt32();
                    break;
                case FieldType.UInt32:
                    o = reader.ReadUInt32();
                    break;
                case FieldType.Int64:
                    o = reader.ReadInt64();
                    break;
                case FieldType.UInt64:
                    o = reader.ReadUInt64();
                    break;
                case FieldType.Char:
                    o = (char)reader.ReadByte();
                    break;
                case FieldType.Struct:
                    {
                        MessageFormat f = field.structType;
                        if (f == null)
                        {
                            throw new Exception(string.Format("Unexpected FieldType.Struct in field {0}", field.name));
                        }
                        MessageData s = new MessageData(this.msgId, null, this.subscription);
                        s.format = f;
                        s.ReadValues(reader);
                        o = s;
                    }
                    break;
            }
            return o;
        }

        internal MessageData GetNestedData(string fieldName)
        {
            if (values == null)
            {
                ParseValues();
            }

            foreach (var field in format.fields)
            {
                if (field.name == fieldName)
                {
                    object value = values[fieldName];
                    if (field.arraySize > 0)
                    {
                        if (nestedFormats == null)
                        {
                            nestedFormats = new Dictionary<string, MessageFormat>();
                        }

                        MessageFormat format = null;
                        if (!nestedFormats.TryGetValue(fieldName, out format))
                        {
                            List<MessageField> fields = new List<MessageField>();

                            object tv;
                            // we need to copy the timestamp down so the data renders with time sync.
                            if (this.values.TryGetValue("timestamp", out tv))
                            {
                                fields.Add(new MessageField("double timestamp"));
                            }

                            for (int i = 0; i < field.arraySize; i++)
                            {
                                var name = i.ToString();
                                fields.Add(new MessageField(MessageField.GetFieldTypeName(field.type) + " " + name));
                            }

                            format = new MessageFormat(fieldName, fields);
                            nestedFormats[fieldName] = format;
                        }

                        MessageData array = new MessageData(this.msgId, null, this.subscription);
                        array.format = format;
                        var values = new Dictionary<string, object>();
                        object ts = null;
                        int offset = 0;
                        // we need to copy the timestamp down so the data renders with time sync.
                        if (this.values.TryGetValue("timestamp", out ts))
                        {
                            values["timestamp"] = ts;
                            offset++;
                        }

                        Array data = (Array)value;
                        for (int i = 0; i < field.arraySize; i++)
                        {
                            string name = format.fields[i + offset].name;
                            values[name] = data.GetValue(i);
                        }

                        array.values = values;
                        return array;
                    }
                    else if (value is MessageData)
                    {
                        return (MessageData)value;
                    }
                    else
                    {
                        throw new Exception(string.Format("Field {0} is not a struct", fieldName));
                    }
                }
            }
            return null;
        }

        internal T GetValue<T>(string fieldName) where T : IComparable
        {
            if (values == null)
            {
                ParseValues();
            }

            foreach (var field in format.fields)
            {
                if (field.name == fieldName)
                {
                    object o = null;
                    if (values.TryGetValue(field.name, out o))
                    {
                        if (typeof(T) == typeof(UInt64))
                        {
                            object i = Convert.ToUInt64(o);
                            return (T)i;
                        }
                        if (typeof(T) == typeof(float))
                        {
                            object i = Convert.ToSingle(o);
                            return (T)i;
                        }
                        if (typeof(T) == typeof(double))
                        {
                            object i = Convert.ToDouble(o);
                            return (T)i;
                        }
                        if (typeof(T) == typeof(int))
                        {
                            object i = Convert.ToInt32(o);
                            return (T)i;
                        }
                        if (typeof(T) == typeof(byte))
                        {
                            object i = Convert.ToByte(o);
                            return (T)i;
                        }
                    }
                }
            }
            return default(T);
        }
    }

    class MessageInfo : Message
    {
        string key;
        bool isContinued;
        string value;

        public MessageInfo(string key, bool isContinued, string value)
        {
            this.key = key;
            this.value = value;
            this.isContinued = isContinued;
        }
        public override ulong GetTimestamp()
        {
            return 0;
        }
        
    }

    class MessageParameter<T> : Message
    {
        string name;
        T value;

        public MessageParameter(string name, T value)
        {
            this.name = name;
            this.value = value;
        }

        public string Name => name;
        public T Value => value;

        public override ulong GetTimestamp()
        {
            return 0;
        }
    }

    [Flags]
    enum UlogParameterDefaultType
    {
        system = 1,
        current_setup = 2
    }

    class MessageParameterDefault<T> : Message
    {
        UlogParameterDefaultType type;
        string name;
        T value;

        public MessageParameterDefault(UlogParameterDefaultType type, string name, T value)
        {
            this.type = type;
            this.name = name;
            this.value = value;
        }
        public string Key => name;
        public T Value => value;


        public override ulong GetTimestamp()
        {
            return 0;
        }
    }

    class MessageSync : Message
    {
        byte[] magic;

        public MessageSync(byte[] magic)
        {
            this.magic = magic;
        }


        public override ulong GetTimestamp()
        {
            return 0;
        }
    }

    class MessageDropOut : Message
    {
        UInt16 duration;

        public MessageDropOut(UInt16 duration)
        {
            this.duration = duration;
        }


        public override ulong GetTimestamp()
        {
            return 0;
        }
    }

    class MessageSubscription
    {
        public MessageFormat format;
        public int multiId;
        public int id;

        public MessageSubscription(int id, MessageFormat fmt, int multiId)
        {
            this.id = id;
            this.format = fmt;
            this.multiId = multiId;
        }
    }

    class Px4ULog : IDataLog
    {

        enum ULogMessageType
        {
            FORMAT = 'F',
            DATA = 'D',
            INFO = 'I',
            INFO_MULTIPLE = 'M',
            PARAMETER = 'P',
            PARAMETER_DEFAULT = 'Q',
            ADD_LOGGED_MSG = 'A',
            REMOVE_LOGGED_MSG = 'R',
            SYNC = 'S',
            DROPOUT = 'O',
            LOGGING = 'L',
            LOGGING_TAGGED = 'C',
            FLAG_BITS = 'B',
        };

        BinaryReader reader;
        TimeSpan duration;
        DateTime startTime;
        long logStartTimestamp;
        string file;
        int version; // ulog version.
        Dictionary<string, MessageFormat> formats = new Dictionary<string, MessageFormat>();
        Dictionary<int, MessageSubscription> subscriptions = new Dictionary<int, MessageSubscription>();
        List<Message> msgs;
        bool resolveNestedTypes;
        LogItemSchema schema;

        public TimeSpan Duration
        {
            get { return this.duration; }
        }

        public LogItemSchema Schema
        {
            get { return schema; }
        }

        public DateTime StartTime
        {
            get { return this.startTime; }
        }

        public IEnumerable<DataValue> GetDataValues(LogItemSchema schema, DateTime startTime, TimeSpan duration)
        {
            List<LogItemSchema> path = new List<Model.LogItemSchema>();
            while (schema != null)
            {
                path.Insert(0, schema);
                schema = schema.Parent;
            }

            LogItemSchema root = path[0];
            foreach (var m in msgs)
            {
                if (m is MessageData)
                {
                    MessageData data = (MessageData)m;
                    //if (data.subscription.id == root.Id)
                    {
                        MessageFormat f = data.format;
                        // matching root schema, so drill down if necessary.
                        for (int i = 1, n = path.Count; i < n; i++)
                        {
                            bool found = false;
                            if (path[i].Name == f.name && i + 1 < n)
                            {
                                LogItemSchema child = path[i + 1];
                                foreach (var field in f.fields)
                                {
                                    if (field.name == child.Name)
                                    {
                                        found = true;
                                        if (i + 1 < n && child.HasChildren)
                                        {
                                            // still need to drill down, so we need a MessageData and MessageFormat for the child item.
                                            data = data.GetNestedData(field.name);
                                            f = data.format;
                                        }
                                        else
                                        {
                                            yield return data.GetValue(field);
                                            found = false; // done drilling down
                                        }
                                        break;
                                    }
                                }
                            }
                            if (!found)
                            {
                                break;
                            }
                        }
                    }
                }
            }
        }

        public IEnumerable<Flight> GetFlights()
        {
            MessageField field = new MessageField("uint8_t arming_state");
            List <Flight> actualFlights = new List<Flight>();
            Flight current = null;
            bool flying = false;
            Px4ULog log = null;
            int length = this.msgs.Count;
            for (int i = 0; i < length; i++)
            {
                var msg = this.msgs[i];
                bool lastMessage = (i == length - 1);
                if (msg is MessageData md)
                {
                    if (md.format.name == "vehicle_status")
                    {
                        var data = md.GetValue(field);
                        if (data != null)
                        {
                            var s = (MessageData.ArmingStates)data.Y;
                            if (s == MessageData.ArmingStates.ARMING_STATE_ARMED)
                            {
                                flying = true;
                            } 
                            else
                            {
                                flying = false;
                            }
                        }
                    }
                }
                if (flying)
                {
                    if (current == null)
                    {
                        log = new Px4ULog()
                        {
                            file = this.file,
                            formats = this.formats,
                            logStartTimestamp = (long)msg.GetTimestamp(),
                            msgs = new List<Message>(),
                            schema = this.schema,
                            startTime = GetTime(msg.GetTimestamp()),
                            subscriptions = this.subscriptions,
                            version = this.version
                        };
                        current = new Flight() { Name = string.Format("Flight {0}", actualFlights.Count + 1), StartTime = log.startTime, Log = log };
                    }
                    log.msgs.Add(msg);

                }

                if ((!flying || lastMessage) && current != null)
                {
                    var now = GetTime(msg.GetTimestamp());
                    current.Duration = now - current.StartTime;
                    actualFlights.Add(current);
                    log = null;
                    current = null;
                }
            }

            return actualFlights;
        }

        public IEnumerable<LogEntry> GetRows(string typeName, DateTime startTime, TimeSpan duration)
        {
            if (typeName == "GPS")
            {
                // convert the ulog "vehicleglobal_position" to a GPS entry.
                MessageFormat format = null;
                if (formats.TryGetValue("vehicle_global_position", out format))
                {
                    foreach (var m in msgs)
                    {
                        if (m is MessageData)
                        {
                            MessageData data = (MessageData)m;
                            if (data.format == format)
                            {
                                yield return new vehicleglobal_position(data);
                            }
                        }
                    }
                }
            }
        }

        public DateTime GetTime(ulong timeMs)
        {
            var delta = timeMs - (ulong)this.logStartTimestamp;
            return this.startTime + new TimeSpan((long)delta * 10); // microseconds to 100 nanosecond ticks.
        }

        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            throw new NotImplementedException();
        }

        public async Task Load(string file, ProgressUtility progress)
        {
            this.file = file;
            this.startTime = File.GetCreationTime(file);
            this.duration = TimeSpan.Zero;
            DateTime lastTime = DateTime.MinValue;

            await Task.Run(() =>
            {
                var msgs = new List<ULog.Message>();

                using (Stream s = File.OpenRead(file))
                {
                    BinaryReader reader = new BinaryReader(s);
                    this.reader = reader;
                    ReadFileHeader();

                    while (s.Position < s.Length)
                    {
                        progress.ShowProgress(0, s.Length, s.Position);

                        try
                        {
                            Message msg = ReadMessage();
                            if (msg != null)
                            {
                                msgs.Add(msg);
                            }
                        }
                        catch
                        {
                            // stop here then.
                            break;
                        }
                    }
                    this.reader = null;
                }
                this.msgs = msgs;
                CreateSchema();
            });
            this.duration = lastTime - startTime;
        }


        private void CreateSchema()
        {
            LogItemSchema schema = new LogItemSchema() { Name = "Px4ULog", Type = "Root" };
            // only need to show formats that we actually have subscriptions on.
            foreach (var sub in this.subscriptions.Values)
            {
                // we can have "multi_id" subscriptions on the same format.
                var element = CreateSchemaItem(sub, sub.format);
                schema.AddChild(element);
            }

            this.schema = schema;
        }

        LogItemSchema CreateSchemaItem(MessageSubscription sub, MessageFormat fmt)
        {
            LogItemSchema element = new LogItemSchema() { Name = fmt.name, Id = sub.id };
            foreach (var f in fmt.fields)
            {
                LogItemSchema column = new LogItemSchema() { Name = f.name, Type = f.typeName + (f.arraySize > 0 ? "[" + f.arraySize + "]" : ""), Id = sub.id };
                if (f.type == FieldType.Struct)
                {
                    // nested
                    var child = CreateSchemaItem(sub, f.structType);
                    foreach (var item in child.CopyChildren())
                    {
                        column.AddChild(item);
                    }
                }
                else if (f.arraySize > 0)
                {
                    column.IsArray = true;
                    // break out the elements of the array as separate items.
                    for (int i = 0; i < f.arraySize; i++)
                    {
                        column.AddChild(new LogItemSchema() { Name = i.ToString(), Type = f.typeName, Id = sub.id });
                    }
                }
                element.AddChild(column);
            }
            return element;
        }


        private void ReadFileHeader()
        {
            byte[] magic = reader.ReadBytes(8);
            var logStartTimestamp = reader.ReadInt64();
            if (magic[0] == 'U' &&
                magic[1] == 'L' &&
                magic[2] == 'o' &&
                magic[3] == 'g' &&
                magic[4] == 0x1 &&
                magic[5]== 0x12 &&
                magic[6] == 0x35)
            {
                // good header
            }
            else
            {
                throw new FormatException("ULog: unexpected file header");
            }
        }

        private Message ReadMessage()
        {
            UInt16 len = reader.ReadUInt16();
            byte type = reader.ReadByte();
            ULogMessageType msgType = (ULogMessageType)type;
            // Debug.WriteLine(msgType);
            switch (msgType)
            {
                case 0:
                    // not sure what these are about!
                    return null;
                case ULogMessageType.FORMAT:
                    return ReadMessageFormat(len);
                case ULogMessageType.DATA:
                    return ReadDataMessage(len);
                case ULogMessageType.INFO:
                    return ReadInfoMessage(len);
                case ULogMessageType.INFO_MULTIPLE:
                    return ReadInfoMessageMultiple(len);
                case ULogMessageType.PARAMETER:
                    return ReadParameter(len);
                case ULogMessageType.PARAMETER_DEFAULT:
                    return ReadParameterDefault(len);
                case ULogMessageType.ADD_LOGGED_MSG:
                    return ReadAddLoggedMessage(len);
                case ULogMessageType.REMOVE_LOGGED_MSG:
                    return ReadRemoveLoggedMessage(len);
                case ULogMessageType.SYNC:
                    return ReadSyncMessage(len);
                case ULogMessageType.DROPOUT:
                    return ReadDropOutMessage(len);
                case ULogMessageType.LOGGING:
                    return ReadLoggingMessage(len);
                case ULogMessageType.LOGGING_TAGGED:
                    return ReadLoggingMessageTagged(len);
                case ULogMessageType.FLAG_BITS:
                    return ReadFlagBits(len);
                default:
                    throw new FormatException("found unexpected ulog message type: " + msgType.ToString());
            }
        }

        private Message ReadFlagBits(ushort len)
        {
            const byte ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK = 1;
            const byte ULOG_COMPAT_FLAG0_DEFAULT_PARAMETERS_MASK = 1;

            byte[] msg = reader.ReadBytes(len);
            bool contains_appended_data = (msg[8] & ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK) != 0;
            bool has_unknown_incompat_bits = (msg[0] & ~0x1) != 0;
            return null;
        }

        internal Message ReadMessageFormat(ushort len)
        {
            string fmtstr = reader.ReadAsciiString(len);
            MessageFormat fmt = new MessageFormat(fmtstr);
            fmt.id = formats.Count;
            formats[fmt.name] = fmt;
            return null;
        }

        public MessageLogging ReadLoggingMessage(ushort len)
        {
            byte logLevel = reader.ReadByte();
            long timestamp = reader.ReadInt64();
            string msg = reader.ReadAsciiString(len - 9);
            return new MessageLogging(logLevel, timestamp, msg);
        }

        public MessageLogging ReadLoggingMessageTagged(ushort len)
        {
            byte logLevel = reader.ReadByte();
            ushort tag = reader.ReadUInt16();
            long timestamp = reader.ReadInt64();
            string msg = reader.ReadAsciiString(len - 9);
            return new MessageLogging(logLevel, timestamp, msg, tag);
        }

        public MessageData ReadDataMessage(ushort len)
        {
            if (!resolveNestedTypes)
            {
                ResolveNestedTypes();
            }

            UInt16 msgId = reader.ReadUInt16();
            int valueSize = len - 2;
            byte[] value = new byte[valueSize];
            reader.Read(value, 0, valueSize);

            MessageSubscription s = null;
            subscriptions.TryGetValue(msgId, out s);
            MessageData data = new MessageData(msgId, value, s);

            return data;
        }

        private void ResolveNestedTypes()
        {
            resolveNestedTypes = true;
            foreach (var fmt in this.formats.Values)
            {
                fmt.Resolve(this.formats);
            }
        }

        internal MessageInfo ReadInfoMessage(ushort len)
        {
            int keyLen = reader.ReadByte();
            string key = reader.ReadAsciiString(keyLen); 
            int space = key.IndexOf(' ');
            if (space < 0)
            {
                throw new Exception("Expecting space separated parameter type/value pair, but found " + key);
            }
            string type = key.Substring(0, space);
            string name = key.Substring(space + 1);
            int valueSize = len - keyLen - 1;

            if (type == "int32_t")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for int32_t parameter value but got " + valueSize);
                }
                int value = reader.ReadInt32();
                return new MessageInfo(name, false, value.ToString());
            }
            else if (type == "float")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for float parameter value but got " + valueSize);
                }
                float value = reader.ReadSingle();
                return new MessageInfo(name, false, value.ToString());
            }
            else
            {
                byte[] value = new byte[valueSize];
                reader.Read(value, 0, valueSize);
                return new MessageInfo(name, false, Encoding.UTF8.GetString(value));
            }
        }

        internal MessageInfo ReadInfoMessageMultiple(ushort len)
        {
            bool isContinued = reader.ReadByte() != 0;
            int keyLen = reader.ReadByte();
            string key = reader.ReadAsciiString(keyLen);
            int space = key.IndexOf(' ');
            if (space < 0)
            {
                throw new Exception("Expecting space separated parameter type/value pair, but found " + key);
            }
            string type = key.Substring(0, space);
            string name = key.Substring(space + 1);
            int valueSize = len - keyLen - 2;

            if (type == "int32_t")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for int32_t parameter value but got " + valueSize);
                }
                int value = reader.ReadInt32();
                return new MessageInfo(name, isContinued, value.ToString());
            }
            else if (type == "float")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for float parameter value but got " + valueSize);
                }
                float value = reader.ReadSingle();
                return new MessageInfo(name, isContinued, value.ToString());
            }
            else
            {
                byte[] value = new byte[valueSize];
                reader.Read(value, 0, valueSize);
                return new MessageInfo(name, isContinued, Encoding.UTF8.GetString(value));
            }
        }

        internal Message ReadParameter(ushort len)
        {
            int keyLen = reader.ReadByte();
            string key = reader.ReadAsciiString(keyLen);
            int valueSize = len - keyLen - 1;
            int space = key.IndexOf(' ');
            if (space < 0)
            {
                throw new Exception("Expecting space separated parameter type/value pair, but found " + key);
            }
            string type = key.Substring(0, space);
            string name = key.Substring(space + 1);

            if (type == "int32_t")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for int32_t parameter value but got " + valueSize);
                }
                int value = reader.ReadInt32();
                return new MessageParameter<int>(name, value);
            }
            else if (type == "float")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for float parameter value but got " + valueSize);
                }
                float value = reader.ReadSingle();
                return new MessageParameter<float>(name, value);
            }
            else
            {
                byte[] value = new byte[valueSize];
                reader.Read(value, 0, valueSize);
                return new MessageParameter<string>(name, Encoding.UTF8.GetString(value));
            }
        }

        internal Message ReadParameterDefault(ushort len)
        {
            var defaultType = (UlogParameterDefaultType)reader.ReadByte();
            int keyLen = reader.ReadByte();
            int valueSize = len - keyLen - 2;
            string key = reader.ReadAsciiString(keyLen);
            int space = key.IndexOf(' ');
            if (space < 0)
            {
                throw new Exception("Expecting space separated parameter type/value pair, but found " + key);
            }
            string type = key.Substring(0, space);
            string name = key.Substring(space + 1);

            if (type == "int32_t")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for int32_t parameter value but got " + valueSize);
                }
                int value = reader.ReadInt32();
                return new MessageParameterDefault<int>(defaultType, name, value);
            }
            else if (type == "float")
            {
                if (valueSize != 4)
                {
                    throw new Exception("Expecting 4 bytes for float parameter value but got " + valueSize);
                }
                float value = reader.ReadSingle();
                return new MessageParameterDefault<float>(defaultType, name, value);
            }
            else
            {
                byte[] value = new byte[valueSize];
                reader.Read(value, 0, valueSize);
                return new MessageParameter<string>(name, Encoding.UTF8.GetString(value));
            }
        }

        public Message ReadAddLoggedMessage(ushort len)
        {
            byte multi_id = reader.ReadByte();
            UInt16 msgId = reader.ReadUInt16();
            string msgName = reader.ReadAsciiString(len - 3);
            MessageFormat fmt = formats[msgName];
            if (!subscriptions.ContainsKey(msgId))
            {
                subscriptions[msgId] = new MessageSubscription(msgId, fmt, multi_id);
            }
            return null;
        }

        public Message ReadRemoveLoggedMessage(ushort len)
        {
            Debug.Assert(len == 3);
            byte multi_id = reader.ReadByte();
            UInt16 msgId = reader.ReadUInt16();
            return null;
        }

        public MessageSync ReadSyncMessage(ushort len)
        {
            Debug.Assert(len == 8);
            byte[] magic = new byte[8];
            reader.Read(magic, 0, 8);
            return new MessageSync(magic);
        }

        public MessageDropOut ReadDropOutMessage(ushort len)
        {
            Debug.Assert(len == 2);
            UInt16 duration = reader.ReadUInt16();
            return new MessageDropOut(duration);
        }

        private class vehicleglobal_position : LogEntry
        {
            public vehicleglobal_position(MessageData m)
            {
                this.Timestamp = m.GetValue<UInt64>("timestamp");
                SetField("GPSTime", m.GetValue<UInt64>("timestamp"));
                SetField("EPH", m.GetValue<float>("eph"));
                SetField("EPV", m.GetValue<float>("epv"));
                SetField("Lat", m.GetValue<double>("lat"));
                SetField("Lon", m.GetValue<double>("lon"));
                SetField("Alt", m.GetValue<float>("alt"));
                SetField("VelN", m.GetValue<float>("vel_n"));
                SetField("VelE", m.GetValue<float>("vel_e"));
                SetField("VelD", m.GetValue<float>("vel_d"));
            }
        }
    }

    static class BinaryReaderExtensions
    {
        public static string ReadAsciiString(this BinaryReader reader, int len)
        {
            byte[] bytes = new byte[len];
            int read = reader.Read(bytes, 0, len);
            if (read != len)
            {
                throw new FormatException("unexpected end of file");
            }
            return Encoding.ASCII.GetString(bytes);
        }
    }
}
