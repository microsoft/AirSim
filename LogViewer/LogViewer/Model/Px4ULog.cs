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
        public abstract double GetTimestamp();
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

        public override double GetTimestamp()
        {
            return 0;
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

        public MessageLogging(byte logLevel, long timestamp, string msg)
        {
            this.logLevel = logLevel;
            this.timestamp = timestamp;
            this.msg = msg;
        }

        public override double GetTimestamp()
        {
            return timestamp;
        }


    }

    class MessageData : Message
    {
        internal MessageSubscription subscription;
        internal MessageFormat format;
        UInt16 msgId;
        byte[] value;
        Dictionary<string, object> values;

        public MessageData(UInt16 msgId, byte[] value, MessageSubscription s)
        {
            this.msgId = msgId;
            this.value = value;
            this.subscription = s;
            this.format = s.format;
        }

        public override double GetTimestamp()
        {
            object ts = null;
            if (values.TryGetValue("timestamp", out ts))
            {
                return Convert.ToDouble(ts);
            }
            return 0;
        }

        internal DataValue GetValue(MessageField field)
        {
            if (values == null)
            {
                ParseValues();
            }

            double x = GetTimestamp();
            double y = 0;

            object o = null;
            if (values.TryGetValue(field.name, out o))
            {
                y = Convert.ToDouble(o);
            }
            return new DataValue()
            {
                X = x,
                Y = y,
                UserData = this,
                Label = GetLabel(field, y)
            };
        }


        static string[] NavStateNames = {
            "NAVIGATION_STATE_MANUAL",
            "NAVIGATION_STATE_ALTCTL",
            "NAVIGATION_STATE_POSCTL",
            "NAVIGATION_STATE_AUTO_MISSION",
            "NAVIGATION_STATE_AUTO_LOITER",
            "NAVIGATION_STATE_AUTO_RTL",
            "NAVIGATION_STATE_AUTO_RCRECOVER",
            "NAVIGATION_STATE_AUTO_RTGS",
            "NAVIGATION_STATE_AUTO_LANDENGFAIL",
            "NAVIGATION_STATE_AUTO_LANDGPSFAIL",
            "NAVIGATION_STATE_ACRO",
            "NAVIGATION_STATE_UNUSED",
            "NAVIGATION_STATE_DESCEND",
            "NAVIGATION_STATE_TERMINATION",
            "NAVIGATION_STATE_OFFBOARD",
            "NAVIGATION_STATE_STAB",
            "NAVIGATION_STATE_RATTITUDE",
            "NAVIGATION_STATE_AUTO_TAKEOFF",
            "NAVIGATION_STATE_AUTO_LAND",
            "NAVIGATION_STATE_AUTO_FOLLOW_TARGET",
            "NAVIGATION_STATE_MAX"
        };

        string GetLabel(MessageField field, double y)
        {
            if (field.name == "nav_state" && this.format.name == "vehicle_status")
            {
                int i = (int)y;
                if (i < NavStateNames.Length)
                {
                    return NavStateNames[i];
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
                        // cook up a MessageData just for the array.
                        MessageData array = new MessageData(this.msgId, null, this.subscription);
                        StringBuilder fieldTypes = new StringBuilder();
                        fieldTypes.Append(fieldName);
                        fieldTypes.Append(':');

                        var values = new Dictionary<string, object>();
                        bool first = true;
                        object ts = null;
                        // we need to copy the timestamp down so the data renders with time sync.
                        if (this.values.TryGetValue("timestamp", out ts))
                        {
                            fieldTypes.Append("double timestamp");
                            values["timestamp"] = ts;
                            first = false;
                        }

                        Array data = (Array)value;
                        for (int i = 0; i < field.arraySize; i++)
                        {
                            if (!first)
                            {
                                fieldTypes.Append(';');
                            }
                            fieldTypes.Append(MessageField.GetFieldTypeName(field.type));
                            fieldTypes.Append(' ');
                            fieldTypes.Append(i.ToString());
                            values[i.ToString()] = data.GetValue(i);
                            first = false;
                        }
                        array.format = new MessageFormat(fieldTypes.ToString());
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
        byte[] value;

        public MessageInfo(string key, byte[] value)
        {
            this.key = key;
            this.value = value;
        }
        public override double GetTimestamp()
        {
            return 0;
        }
        
    }

    class MessageParameter : Message
    {
        string key;
        byte[] value;

        public MessageParameter(string key, byte[] value)
        {
            this.key = key;
            this.value = value;
        }


        public override double GetTimestamp()
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


        public override double GetTimestamp()
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


        public override double GetTimestamp()
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
            PARAMETER = 'P',
            ADD_LOGGED_MSG = 'A',
            REMOVE_LOGGED_MSG = 'R',
            SYNC = 'S',
            DROPOUT = 'O',
            LOGGING = 'L',
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
                    if (data.subscription.id == root.Id)
                    {
                        MessageFormat f = data.format;
                        // matching root schema, so drill down if necessary.
                        for (int i = 1, n = path.Count; i < n; i++)
                        {
                            bool found = false;
                            LogItemSchema child = path[i];
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
            return new Flight[0];
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
            throw new NotImplementedException();
        }

        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            throw new NotImplementedException();
        }

        public async Task Load(string file, ProgressUtility progress)
        {
            this.file = file;
            this.startTime = DateTime.MinValue;
            this.duration = TimeSpan.Zero;
            DateTime lastTime = DateTime.MinValue;

            await Task.Run(() =>
            {
                this.msgs = new List<ULog.Message>();

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

                CreateSchema();
            });
            this.duration = lastTime - startTime;
        }


        private void CreateSchema()
        {
            LogItemSchema schema = new LogItemSchema() { Name = "Px4ULog", Type = "Root", ChildItems = new List<Model.LogItemSchema>() };
            // only need to show formats that we actually have subscriptions on.
            foreach (var sub in this.subscriptions.Values)
            {
                // we can have "multi_id" subscriptions on the same format.
                var element = CreateSchemaItem(sub, sub.format);
                schema.ChildItems.Add(element);
            }

            this.schema = schema;
        }

        LogItemSchema CreateSchemaItem(MessageSubscription sub, MessageFormat fmt)
        {
            LogItemSchema element = new LogItemSchema() { Name = fmt.name, Parent = schema, Id = sub.id };
            foreach (var f in fmt.fields)
            {
                LogItemSchema column = new LogItemSchema() { Name = f.name, Parent = element, Type = f.typeName + (f.arraySize > 0 ? "[" + f.arraySize + "]" : ""), Id = sub.id };
                if (f.type == FieldType.Struct)
                {
                    // nested
                    var child = CreateSchemaItem(sub, f.structType);
                    column.ChildItems = child.ChildItems;
                }
                else if (f.arraySize > 0)
                {
                    List<LogItemSchema> arrayItems = new List<LogItemSchema>();
                    // break out the elements of the array as separate items.
                    for (int i = 0; i < f.arraySize; i++)
                    {
                        arrayItems.Add(new LogItemSchema() { Name = i.ToString(), Parent = column, Type = f.typeName, Id = sub.id });
                    }
                    column.ChildItems = arrayItems;
                }
                if (element.ChildItems == null)
                {
                    element.ChildItems = new List<Model.LogItemSchema>();
                }
                element.ChildItems.Add(column);
            }
            return element;
        }


        private void ReadFileHeader()
        {
            if (reader.ReadByte() == 'U' &&
                reader.ReadByte() == 'L' &&
                reader.ReadByte() == 'o' &&
                reader.ReadByte() == 'g' &&
                reader.ReadByte() == 0x1 &&
                reader.ReadByte() == 0x12 &&
                reader.ReadByte() == 0x35)
            {
                version = reader.ReadByte();
                logStartTimestamp = reader.ReadInt64();
            }
            else
            {
                throw new FormatException("ULog: unexpected file header");
            }
        }

        private Message ReadMessage()
        {
            UInt16 len = reader.ReadUInt16();
            ULogMessageType msgType = (ULogMessageType)reader.ReadByte();
            switch (msgType)
            {
                case ULogMessageType.FORMAT:
                    return ReadMessageFormat(len);
                case ULogMessageType.DATA:
                    return ReadDataMessage(len);
                case ULogMessageType.INFO:
                    return ReadInfoMessage(len);
                case ULogMessageType.PARAMETER:
                    return ReadParameter(len);
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
                default:
                    throw new FormatException("found unexpected ulog message type: " + msgType.ToString());
            }
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
            int valueSize = len - keyLen - 1;
            byte[] value = new byte[valueSize];
            reader.Read(value, 0, valueSize);
            return new MessageInfo(key, value);
        }

        internal MessageParameter ReadParameter(ushort len)
        {
            int keyLen = reader.ReadByte();
            string key = reader.ReadAsciiString(keyLen);
            int valueSize = len - keyLen - 1;
            byte[] value = new byte[valueSize];
            reader.Read(value, 0, valueSize);
            return new MessageParameter(key, value);
        }

        public Message ReadAddLoggedMessage(ushort len)
        {
            byte multi_id = reader.ReadByte();
            UInt16 msgId = reader.ReadUInt16();
            string msgName = reader.ReadAsciiString(len - 3);
            MessageFormat fmt = formats[msgName];
            subscriptions[msgId] = new MessageSubscription(msgId, fmt, multi_id);
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
