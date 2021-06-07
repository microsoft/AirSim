using LogViewer.Controls;
using LogViewer.Utilities;
using Microsoft.Xml;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Xml;
using System.Xml.Linq;

namespace LogViewer.Model
{
    class CsvDataLog : IDataLog
    {
        DateTime startTime = DateTime.MinValue;
        TimeSpan duration = TimeSpan.Zero;
        LogItemSchema schema;
        List<Flight> flights = new List<Flight>();
        string timeElementName;
        List<LogEntry> log = new List<LogEntry>();


        public DateTime StartTime { get { return startTime; } }
        public TimeSpan Duration { get { return duration; } }

        public LogItemSchema Schema { get { return this.schema; } }

        public IEnumerable<DataValue> GetDataValues(LogItemSchema schema, DateTime startTime, TimeSpan duration)
        {
            if (this.log != null && schema.Parent != null)
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

        public DateTime GetTime(ulong timeMs)
        {
            return this.startTime + TimeSpan.FromMilliseconds(timeMs);
        }

        public async Task Load(string fileName, ProgressUtility progress)
        {
            flights.Clear();
            // CSV doesn't have realtime clock, so go with the file date instead.
            this.startTime = File.GetLastWriteTime(fileName);

            // time (ms)
            long min = long.MaxValue;
            long max = long.MinValue;

            await Task.Run(() =>
            {
                timeElementName = null;
                using (Stream s = File.OpenRead(fileName))
                {
                    Dictionary<string, LogItemSchema> map = new Dictionary<string, LogItemSchema>();
                    XmlNameTable nametable = new NameTable();
                    using (XmlCsvReader reader = new XmlCsvReader(s, System.Text.Encoding.UTF8, new Uri(fileName), nametable))
                    {
                        reader.FirstRowHasColumnNames = true;
                        reader.ColumnsAsAttributes = true;
                        while (reader.Read())
                        {
                            progress.ShowProgress(0, s.Length, s.Position);
                            if (this.schema == null)
                            {
                                // create the schema
                                this.schema = new LogItemSchema() { Name = "CsvLog", Type = "Root" };
                                LogItemSchema row = null;

                                foreach (String name in reader.ColumnNames)
                                {
                                    if (timeElementName == null && (name.ToLower().Contains("time") || name.ToLower().Contains("ticks")))
                                    {
                                        timeElementName = name;
                                    }

                                    if (name.Contains(":"))
                                    {
                                        // then we have sub-parts.
                                        int pos = name.IndexOf(":");
                                        string key = name.Substring(0, pos);
                                        string field = name.Substring(pos + 1);
                                        LogItemSchema group = null;
                                        if (!map.ContainsKey(key))
                                        {
                                            group = new LogItemSchema() { Name = key, Type = key };
                                            this.schema.AddChild(group);
                                            map[key] = group;
                                        }
                                        else
                                        {
                                            group = map[key];
                                        }
                                        var leaf = new LogItemSchema() { Name = field, Type = "Double" };
                                        group.AddChild(leaf);
                                        map[name] = leaf;
                                    }
                                    else
                                    {
                                        if (row == null)
                                        {
                                            row = new LogItemSchema() { Name = "Other", Type = "Other" };
                                            this.schema.AddChild(row);
                                        }
                                        var leaf = new LogItemSchema() { Name = name, Type = "Double" };
                                        row.AddChild(leaf);
                                        map[name] = leaf;
                                    }
                                }
                            }

                            if (reader.NodeType == XmlNodeType.Element && reader.LocalName == "row")
                            {
                                // read a row
                                long time = GetTicks(reader);
                                min = Math.Min(min, time);
                                max = Math.Max(max, time);
                                LogEntry row = new Model.LogEntry() { Name = "Other", Timestamp = (ulong)time };
                                log.Add(row);                                
                                Dictionary<string, LogEntry> groups = new Dictionary<string, LogEntry>();

                                if (reader.MoveToFirstAttribute())
                                {
                                    do
                                    {
                                        string name = XmlConvert.DecodeName(reader.LocalName);
                                        LogItemSchema itemSchema = map[name];
                                        LogEntry e = row;
                                        if (name.Contains(":"))
                                        {
                                            // then we have sub-parts.
                                            int pos = name.IndexOf(":");
                                            string key = name.Substring(0, pos);
                                            string field = name.Substring(pos + 1);
                                            if (!groups.ContainsKey(key))
                                            {
                                                e = new LogEntry() { Name = key, Timestamp = (ulong)time };
                                                groups[key] = e;
                                                log.Add(e);
                                            }
                                            else
                                            {
                                                e = groups[key];
                                            }
                                            name = field;
                                        }

                                        string value = reader.Value;
                                        double d = 0;
                                        if (double.TryParse(value, out d))
                                        {
                                            e.SetField(name, d);
                                        }
                                        else
                                        {
                                            if (!string.IsNullOrEmpty(value))
                                            {
                                                // not a number.
                                                itemSchema.Type = "String";
                                                e.SetField(name, value);
                                            }
                                        }
                                    }
                                    while (reader.MoveToNextAttribute());
                                    reader.MoveToElement();
                                }
                            }
                        }
                    }
                }
            });

            // this log has no absolute UTC time, only ticks since board was booted, so we make up a start time.
            DateTime end = this.startTime.AddMilliseconds((max - min) / 1000);
            var flight = new Flight() { Log = this, StartTime = this.startTime, Duration = end - this.startTime };
            this.duration = end - this.startTime;
            this.flights.Add(flight);

        }

        long GetTicks(XmlReader e)
        {
            string time = e.GetAttribute(timeElementName);
            if (!string.IsNullOrEmpty(time))
            {
                double i = 0;
                if (double.TryParse(time, out i))
                {
                    return (long)i;
                }
            }
            return 0;
        }

        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            throw new NotImplementedException("LiveQuery");
        }

        public IEnumerable<LogEntry> GetRows(string typeName, DateTime startTime, TimeSpan duration)
        {
            foreach (var row in this.log)
            {
                if (string.Compare(row.Name, typeName, StringComparison.OrdinalIgnoreCase) == 0 &&
                    (duration == TimeSpan.MaxValue ||
                    row.Timestamp >= (ulong)startTime.Ticks && row.Timestamp < (ulong)(startTime + duration).Ticks))
                {
                    yield return row;
                }
            }
        }

        /// <summary>
        /// Extract actual flight times from the log (based on when the motors were actually on).
        /// </summary>
        public IEnumerable<Flight> GetFlights()
        {
            return flights;
        }
    }
}
