using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Runtime.InteropServices;
using uint8_t = System.Byte;
using System.Diagnostics;
using LogViewer.Utilities;
using LogViewer.Model;

namespace MissionPlanner.Log
{
    public enum LogType { binFile, px4logFile };

    /// <summary>
    /// Convert a binary log to an ascii log
    /// </summary>
    public class PX4BinaryLog
    {
        private LogType logType;
        private IntPtr _logFormatPtr;
        private byte[] _logFormatBuffer;
        public const byte HEAD_BYTE1 = 0xA3; // Decimal 163  
        public const byte HEAD_BYTE2 = 0x95; // Decimal 149  
        ulong currentTime;

        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct log_Format
        {
            public uint8_t type;
            public uint8_t length;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public byte[] name;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] format;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
            public byte[] labels;
        }

        public ulong CurrentTime {  get { return currentTime; } }
        
        public bool GenerateParser { get; set; }

        public PX4BinaryLog(LogType logType)
        {
            this.logType = logType;
        }

        ~PX4BinaryLog()
        {
            if (_logFormatPtr != IntPtr.Zero)
            {
                Marshal.FreeCoTaskMem(_logFormatPtr);
                _logFormatPtr = IntPtr.Zero;
            }
        }

        void ConvertBinaryToText(string binaryFileName, string outputFileName)
        {
            using (var stream = File.Open(outputFileName, FileMode.Create))
            {
                using (StreamWriter writer = new StreamWriter(stream, Encoding.UTF8))
                {
                    using (BinaryReader br = new BinaryReader(File.OpenRead(binaryFileName)))
                    {
                        while (br.BaseStream.Position < br.BaseStream.Length)
                        {
                            string line = ReadMessage(br.BaseStream);
                            writer.WriteLine(line);
                        }
                    }
                }
            }
        }

        public string ReadMessage(Stream br)
        {
            int log_step = 0;

            long length = br.Length;

            while (br.Position < length)
            {
                byte data = (byte)br.ReadByte();

                switch (log_step)
                {
                    case 0:
                        if (data == HEAD_BYTE1)
                        {
                            log_step++;
                        }
                        break;

                    case 1:
                        if (data == HEAD_BYTE2)
                        {
                            log_step++;
                        }
                        else
                        {
                            log_step = 0;
                        }
                        break;

                    case 2:
                        log_step = 0;
                        try
                        {
                            string line = logEntry(data, br);
                            return line;
                        }
                        catch
                        {
                            Debug.WriteLine("Bad Binary log line {0}", data);
                        }
                        break;
                }
            }

            return "";
        }


        public Tuple<byte, long> ReadMessageTypeOffset(Stream br)
        {
            int log_step = 0;
            long length = br.Length;

            while (br.Position < length)
            {
                byte data = (byte)br.ReadByte();

                switch (log_step)
                {
                    case 0:
                        if (data == HEAD_BYTE1)
                        {
                            log_step++;
                        }
                        break;

                    case 1:
                        if (data == HEAD_BYTE2)
                        {
                            log_step++;
                        }
                        else
                        {
                            log_step = 0;
                        }
                        break;

                    case 2:
                        log_step = 0;
                        try
                        {
                            long pos = br.Position - 3;
                            logEntryFMT(data, br);

                            return new Tuple<byte, long>(data, pos);
                        }
                        catch
                        {
                            Debug.WriteLine("Bad Binary log line {0}", data);
                        }
                        break;
                }
            }

            return null;
        }

        static char[] NullTerminator = new char[] { '\0' };

        LogEntryFMT ReadLogFormat(Stream br)
        {
            int len = Marshal.SizeOf<log_Format>();
            if (_logFormatPtr == IntPtr.Zero)
            {
                _logFormatPtr = Marshal.AllocCoTaskMem(len);
                _logFormatBuffer = new byte[len];
            }

            br.Read(_logFormatBuffer, 0, _logFormatBuffer.Length);

            // copy byte array to ptr
            Marshal.Copy(_logFormatBuffer, 0, _logFormatPtr, len);

            log_Format logfmt = Marshal.PtrToStructure<log_Format>(_logFormatPtr);

            string lgname = ASCIIEncoding.ASCII.GetString(logfmt.name).Trim(NullTerminator);


            var result = new LogEntryFMT()
            {
                Length = logfmt.length,
                Type = logfmt.type,
                Name = ASCIIEncoding.ASCII.GetString(logfmt.name).Trim(NullTerminator),
                FormatString = ASCIIEncoding.ASCII.GetString(logfmt.format).Trim(NullTerminator),
                Columns = ASCIIEncoding.ASCII.GetString(logfmt.labels).Trim(NullTerminator).Split(',')
            };
            if (result.Columns.Length == 1 && string.IsNullOrEmpty(result.Columns[0]))
            {
                result.Columns = new string[0];
            }
            packettypecache[logfmt.type] = result;
            Debug.WriteLine("// FMT {0}, {1}, {2}, {3}", result.Name, result.Length, result.FormatString, string.Join(",", result.Columns));

            if (GenerateParser)
            {
                StringWriter sw = new StringWriter();
                GenerateLogEntryClass(result, sw);
                Debug.WriteLine(sw.ToString());
            }
            return result;
        }

        public IEnumerable<LogEntryFMT> GetFormats()
        {
            return this.packettypecache.Values.ToArray();
        }

        void GenerateLogEntryParser(TextWriter writer)
        {
            writer.WriteLine("class LogEntryParser");
            writer.WriteLine("{");

            writer.WriteLine("    public static object ParseLogEntry(byte type, BinaryReader reader) {");

            writer.WriteLine("        switch(type) {");
            foreach (var pair in packettypecache)
            {
                LogEntryFMT fmt = pair.Value;
                string className = LogEntryPrefix + fmt.Name;

                writer.WriteLine("        case " + pair.Key + ":");
                writer.WriteLine("            return " + className + ".Read(reader);");
            }
            writer.WriteLine("        }");
            writer.WriteLine("        return null;");
            writer.WriteLine("    }");
            writer.WriteLine("}");
        }

        const string LogEntryPrefix = "LogEntry";

        void GenerateLogEntryClass(LogEntryFMT fmt, TextWriter writer)
        {
            const string LogEntryPrefix = "LogEntry";
            string className = LogEntryPrefix + fmt.Name;
            writer.WriteLine("class " + className + " : LogEntry {");


            writer.WriteLine("    public override string GetName() { return \"" + fmt.Name + "\"; }");
            writer.WriteLine("    public override DataValue GetDataValue(string field) {");
            writer.WriteLine("        switch (field) {");
            bool hasTime = fmt.Columns.Contains("TimeMS");
            string x = hasTime ? "this.TimeMS" : "0";

            int i = 0;
            foreach (var c in fmt.Columns)
            {
                writer.WriteLine("            case \"" + c + "\":");
                switch (fmt.FormatString[i])
                {
                    case 'n':
                    case 'N':
                    case 'Z':
                        writer.WriteLine("                return new DataValue() { X = " + x + ", Y = 0 };");
                        break;
                    default:
                        writer.WriteLine("                return new DataValue() { X = " + x + ", Y = this." + c + " };");
                        break;
                }
                i++;
            }
            writer.WriteLine("        }");
            writer.WriteLine("        return null;");
            writer.WriteLine("    }");

            int k = 0;
            int n = 0;
            string format = fmt.FormatString;
            for (k = 0, n = format.Length; k < n; k++)
            {
                char ch = format[k];
                switch (ch)
                {
                    case 'b':
                        writer.WriteLine("    public sbyte " + fmt.Columns[k] + ";");
                        break;
                    case 'B':
                        writer.WriteLine("    public byte " + fmt.Columns[k] + ";");
                        break;
                    case 'h':
                        writer.WriteLine("    public Int16 " + fmt.Columns[k] + ";");
                        break;
                    case 'H':
                        writer.WriteLine("    public UInt16 " + fmt.Columns[k] + ";");
                        break;
                    case 'i':
                        writer.WriteLine("    public Int32 " + fmt.Columns[k] + ";");
                        break;
                    case 'I':
                        writer.WriteLine("    public UInt32 " + fmt.Columns[k] + ";");
                        break;
                    case 'q':
                        writer.WriteLine("    public Int64 " + fmt.Columns[k] + ";");
                        break;
                    case 'Q':
                        writer.WriteLine("    public UInt64 " + fmt.Columns[k] + ";");
                        break;
                    case 'f':
                        writer.WriteLine("    public float " + fmt.Columns[k] + ";");
                        break;
                    case 'd':
                        writer.WriteLine("    public double " + fmt.Columns[k] + ";");
                        break;
                    case 'c':
                        writer.WriteLine("    public double " + fmt.Columns[k] + ";"); // divide by 100.0
                        break;
                    case 'C':
                        writer.WriteLine("    public double " + fmt.Columns[k] + ";"); // divide by 100.0
                        break;
                    case 'e':
                        writer.WriteLine("    public double " + fmt.Columns[k] + ";"); // divide by 100.0
                        break;
                    case 'E':
                        writer.WriteLine("    public double " + fmt.Columns[k] + ";"); // divide by 100.0
                        break;
                    case 'L':
                        writer.WriteLine("    public double " + fmt.Columns[k] + ";"); // divide by 10000000.0
                        break;
                    case 'n':
                        writer.WriteLine("    public string " + fmt.Columns[k] + ";");
                        break;
                    case 'N':
                        writer.WriteLine("    public string " + fmt.Columns[k] + ";");
                        break;
                    case 'M':
                        writer.WriteLine("    public byte " + fmt.Columns[k] + ";");
                        break;
                    case 'Z':
                        writer.WriteLine("    public string " + fmt.Columns[k] + ";");
                        break;
                    default:
                        writer.WriteLine("    // Unexpected format specifier '{0}'", ch);
                        break;
                }
            }

            writer.WriteLine("");
            writer.WriteLine("    public static " + className + " Read(BinaryReader reader) {");
            writer.WriteLine("        " + className + " result = new " + className + "();");
            for (k = 0, n = format.Length; k < n; k++)
            {
                char ch = format[k];
                writer.Write("        result." + fmt.Columns[k] + " = ");
                switch (ch)
                {
                    case 'b':
                        writer.WriteLine("(sbyte)reader.ReadByte();");
                        break;
                    case 'B':
                        writer.WriteLine("reader.ReadByte();");
                        break;
                    case 'h':
                        writer.WriteLine("reader.ReadInt16();");
                        break;
                    case 'H':
                        writer.WriteLine("reader.ReadUInt16();");
                        break;
                    case 'i':
                        writer.WriteLine("reader.ReadInt32();");
                        break;
                    case 'I':
                        writer.WriteLine("reader.ReadUInt32();");
                        break;
                    case 'q':
                        writer.WriteLine("reader.ReadInt64();");
                        break;
                    case 'Q':
                        writer.WriteLine("reader.ReadUInt64();");
                        break;
                    case 'f':
                        writer.WriteLine("reader.ReadSingle();");
                        break;
                    case 'd':
                        writer.WriteLine("reader.ReadDouble();");
                        break;
                    case 'c':
                        writer.WriteLine("reader.ReadInt16() / 100.0;");
                        break;
                    case 'C':
                        writer.WriteLine("reader.ReadUInt16() / 100.0;");
                        break;
                    case 'e':
                        writer.WriteLine("reader.ReadInt32() / 100.0;");
                        break;
                    case 'E':
                        writer.WriteLine("reader.ReadUInt32() / 100.0;");
                        break;
                    case 'L':
                        writer.WriteLine("reader.ReadInt32() / 10000000.0;");
                        break;
                    case 'n':
                        writer.WriteLine("ReadAsciiString(reader, 4);");
                        break;
                    case 'N':
                        writer.WriteLine("ReadAsciiString(reader, 16);");
                        break;
                    case 'M':
                        writer.WriteLine("reader.ReadByte();");
                        break;
                    case 'Z':
                        writer.WriteLine("ReadAsciiString(reader, 64);");
                        break;
                    default:
                        break;
                }

            }

            writer.WriteLine("        return result;");
            writer.WriteLine("    }");
            writer.WriteLine("}");

        }

        bool generatedParser;

        void logEntryFMT(byte packettype, Stream br)
        {
            switch (packettype)
            {
                case 0x80: // FMT
                    ReadLogFormat(br);
                    return;

                default:
                    string format = "";
                    string name = "";
                    int size = 0;
                    if (packettypecache.ContainsKey(packettype))
                    {
                        var fmt = packettypecache[packettype];
                        name = fmt.Name;
                        format = fmt.FormatString;
                        size = fmt.Length;
                    }

                    // didn't find a match, return unknown packet type
                    if (size == 0)
                        return;

                    br.Seek(size - 3, SeekOrigin.Current);
                    break;
            }
        }

        public object ReadMessageObjects(Stream br)
        {
            int log_step = 0;

            while (br.Position < br.Length)
            {
                byte data = (byte)br.ReadByte();

                switch (log_step)
                {
                    case 0:
                        if (data == HEAD_BYTE1)
                        {
                            log_step++;
                        }
                        break;

                    case 1:
                        if (data == HEAD_BYTE2)
                        {
                            log_step++;
                        }
                        else
                        {
                            log_step = 0;
                        }
                        break;

                    case 2:
                        log_step = 0;
                        return ReadRow(data, br);
                }
            }

            return null;
        }

        object ReadRow(byte packettype, Stream br)
        {
            switch (packettype)
            {
                case 0x80: // FMT
                    ReadLogFormat(br);
                    return null;

                default:
                    string format = "";
                    string name = "";
                    int size = 0;

                    if (!generatedParser && GenerateParser)
                    {
                        StringWriter writer = new StringWriter();
                        GenerateLogEntryParser(writer);
                        Debug.WriteLine(writer.ToString());
                        generatedParser = true;
                    }

                    LogEntryFMT fmt = null;
                    if (packettypecache.ContainsKey(packettype))
                    {
                        fmt = packettypecache[packettype];
                        name = fmt.Name;
                        format = fmt.FormatString;
                        size = fmt.Length;
                    }

                    // didn't find a match, return unknown packet type
                    if (fmt == null)
                        return null;

                    int len = size - 3; // size - 3 = message - messagetype - (header *2)

                    if (buffer == null || buffer.Length < len)
                    {
                        buffer = new byte[len];
                    }
                    br.Read(buffer, 0, len);

                    MemoryStream ms = new MemoryStream(buffer, 0, len);
                    BinaryReader reader = new BinaryReader(ms);

                    LogEntry entry = new LogEntry()
                    {
                        Format = fmt,
                        Blob = ms.ToArray(),
                        Name = fmt.Name
                    };
                    if (name == "TIME")
                    {
                        ulong time = entry.GetField<ulong>("StartTime");
                        if (time > 0)
                        {
                            this.currentTime = time;
                        }
                    }
                    if (this.logType == LogType.binFile)
                    {
                        ulong time = entry.GetField<ulong>("TimeMS");
                        if (time > 0)
                        {
                            this.currentTime = time;
                        }
                        else if (time == 0)
                        {
                            entry.SetField("TimeMS", this.currentTime);
                        }
                    }
                    entry.Timestamp = currentTime;

                    return entry;
            }
        }

        private byte[] buffer;

        /// <summary>
        /// Process each log entry
        /// </summary>
        /// <param name="packettype">packet type</param>
        /// <param name="br">input file</param>
        /// <returns>string of converted data</returns>
        string logEntry(byte packettype, Stream br)
        {
            switch (packettype)
            {
                case 0x80: // FMT

                    LogEntryFMT logfmt = ReadLogFormat(br);
                    string line = String.Format("FMT, {0}, {1}, {2}, {3}, {4}\r\n", logfmt.Type, logfmt.Length, logfmt.Name,
                        logfmt.FormatString, string.Join(",", logfmt.Columns));
                    return line;

                default:
                    string format = "";
                    string name = "";
                    int size = 0;

                    if (packettypecache.ContainsKey(packettype))
                    {
                        var fmt = packettypecache[packettype];
                        name = fmt.Name;
                        format = fmt.FormatString;
                        size = fmt.Length;
                    }

                    // didn't find a match, return unknown packet type
                    if (size == 0)
                        return "UNKW, " + packettype;

                    byte[] data = new byte[size - 3]; // size - 3 = message - messagetype - (header *2)

                    br.Read(data, 0, data.Length);

                    return ProcessMessage(data, name, format);
            }
        }

        Dictionary<int, LogEntryFMT> packettypecache = new Dictionary<int, LogEntryFMT>();

        /*  
    105    +Format characters in the format string for binary log messages  
    106    +  b   : int8_t  
    107    +  B   : uint8_t  
    108    +  h   : int16_t  
    109    +  H   : uint16_t  
    110    +  i   : int32_t  
    111    +  I   : uint32_t  
    112    +  f   : float  
         *     d   : double
    113    +  N   : char[16]  
    114    +  c   : int16_t * 100  
    115    +  C   : uint16_t * 100  
    116    +  e   : int32_t * 100  
    117    +  E   : uint32_t * 100  
    118    +  L   : uint32_t latitude/longitude  
    119    + */


        /// <summary>
        /// Convert to ascii based on the existing format message
        /// </summary>
        /// <param name="message">raw binary message</param>
        /// <param name="name">Message type name</param>
        /// <param name="format">format string containing packet structure</param>
        /// <returns>formatted ascii string</returns>
        string ProcessMessage(byte[] message, string name, string format)
        {
            char[] form = format.ToCharArray();

            int offset = 0;

            StringBuilder line = new StringBuilder(name, 1024);

            foreach (char ch in form)
            {
                switch (ch)
                {
                    case 'b':
                        line.Append(", " + (sbyte)message[offset]);
                        offset++;
                        break;
                    case 'B':
                        line.Append(", " + message[offset]);
                        offset++;
                        break;
                    case 'h':
                        line.Append(", " +
                                    BitConverter.ToInt16(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 2;
                        break;
                    case 'H':
                        line.Append(", " +
                                    BitConverter.ToUInt16(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 2;
                        break;
                    case 'i':
                        line.Append(", " +
                                    BitConverter.ToInt32(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 4;
                        break;
                    case 'I':
                        line.Append(", " +
                                    BitConverter.ToUInt32(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 4;
                        break;
                    case 'q':
                        line.Append(", " +
                                    BitConverter.ToInt64(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 8;
                        break;
                    case 'Q':
                        line.Append(", " +
                                    BitConverter.ToUInt64(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 8;
                        break;
                    case 'f':
                        line.Append(", " +
                                    BitConverter.ToSingle(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 4;
                        break;
                    case 'd':
                        line.Append(", " +
                                    BitConverter.ToDouble(message, offset)
                                        .ToString(System.Globalization.CultureInfo.InvariantCulture));
                        offset += 8;
                        break;
                    case 'c':
                        line.Append(", " +
                                    (BitConverter.ToInt16(message, offset) / 100.0).ToString("0.00",
                                        System.Globalization.CultureInfo.InvariantCulture));
                        offset += 2;
                        break;
                    case 'C':
                        line.Append(", " +
                                    (BitConverter.ToUInt16(message, offset) / 100.0).ToString("0.00",
                                        System.Globalization.CultureInfo.InvariantCulture));
                        offset += 2;
                        break;
                    case 'e':
                        line.Append(", " +
                                    (BitConverter.ToInt32(message, offset) / 100.0).ToString("0.00",
                                        System.Globalization.CultureInfo.InvariantCulture));
                        offset += 4;
                        break;
                    case 'E':
                        line.Append(", " +
                                    (BitConverter.ToUInt32(message, offset) / 100.0).ToString("0.00",
                                        System.Globalization.CultureInfo.InvariantCulture));
                        offset += 4;
                        break;
                    case 'L':
                        line.Append(", " +
                                    ((double)BitConverter.ToInt32(message, offset) / 10000000.0).ToString(
                                        System.Globalization.CultureInfo.InvariantCulture));
                        offset += 4;
                        break;
                    case 'n':
                        line.Append(", " + ASCIIEncoding.ASCII.GetString(message, offset, 4).Trim(NullTerminator));
                        offset += 4;
                        break;
                    case 'N':
                        line.Append(", " + ASCIIEncoding.ASCII.GetString(message, offset, 16).Trim(NullTerminator));
                        offset += 16;
                        break;
                    case 'M':
                        int modeno = message[offset];
                        var modes = GetModesList();
                        string currentmode = "";

                        foreach (var mode in modes)
                        {
                            if (mode.Key == modeno)
                            {
                                currentmode = mode.Value;
                                break;
                            }
                        }

                        line.Append(", " + currentmode);
                        offset++;
                        break;
                    case 'Z':
                        line.Append(", " + ASCIIEncoding.ASCII.GetString(message, offset, 64).Trim(NullTerminator));
                        offset += 64;
                        break;
                    default:
                        return "Bad Conversion";
                }
            }

            line.Append("\r\n");
            return line.ToString();
        }

        List<KeyValuePair<int, string>> modeList;

        private List<KeyValuePair<int, string>> GetModesList()
        {
            if (modeList == null)
            {
                modeList = new List<KeyValuePair<int, string>>()
                {
                    new KeyValuePair<int, string>((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_MANUAL << 16, "Manual"),
                    new KeyValuePair<int, string>((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_ACRO << 16, "Acro"),
                    new KeyValuePair<int, string>((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_STABILIZED << 16, "Stabalized"),
                    new KeyValuePair<int, string>((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_RATTITUDE << 16, "Rattitude"),
                    new KeyValuePair<int, string>((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_ALTCTL << 16, "Altitude Control"),
                    new KeyValuePair<int, string>((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_POSCTL << 16, "Position Control"),
                    new KeyValuePair<int, string>((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16, "Offboard Control"),
                    new KeyValuePair<int, string>(
                        ((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO << 16) +
                        (int) PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_READY << 24, "Auto: Ready"),
                    new KeyValuePair<int, string>(
                        ((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO << 16) +
                        (int) PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF << 24, "Auto: Takeoff"),
                    new KeyValuePair<int, string>(
                        ((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO << 16) +
                        (int) PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LOITER << 24, "Loiter"),
                    new KeyValuePair<int, string>(
                        ((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO << 16) +
                        (int) PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_MISSION << 24, "Auto"),
                    new KeyValuePair<int, string>(
                        ((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO << 16) +
                        (int) PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_RTL << 24, "RTL"),
                    new KeyValuePair<int, string>(
                        ((int) PX4_CUSTOM_MAIN_MODE.PX4_CUSTOM_MAIN_MODE_AUTO << 16) +
                        (int) PX4_CUSTOM_SUB_MODE_AUTO.PX4_CUSTOM_SUB_MODE_AUTO_LAND << 24, "Auto: Landing")
                };
            }
            return modeList;
        }

        enum PX4_CUSTOM_MAIN_MODE
        {
            PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
            PX4_CUSTOM_MAIN_MODE_ALTCTL,
            PX4_CUSTOM_MAIN_MODE_POSCTL,
            PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_MAIN_MODE_ACRO,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            PX4_CUSTOM_MAIN_MODE_STABILIZED,
            PX4_CUSTOM_MAIN_MODE_RATTITUDE
        }

        enum PX4_CUSTOM_SUB_MODE_AUTO
        {
            PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
            PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
            PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
            PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
            PX4_CUSTOM_SUB_MODE_AUTO_RTL,
            PX4_CUSTOM_SUB_MODE_AUTO_LAND,
            PX4_CUSTOM_SUB_MODE_AUTO_RTGS
        }
    }
}