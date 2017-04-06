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
        XDocument data;
        LogItemSchema schema = new LogItemSchema() { Name = "CsvDataLog", Type = "Root" };
        List<Flight> flights = new List<Flight>();
        string timeElementName;

        public DateTime StartTime { get { return startTime; } }
        public TimeSpan Duration { get { return duration; } }

        public LogItemSchema Schema { get { return this.schema; } }

        public IEnumerable<DataValue> GetDataValues(LogItemSchema schema, DateTime startTime, TimeSpan duration)
        {
            if (data != null)
            {
                bool allValues = (startTime == DateTime.MinValue && duration == TimeSpan.MaxValue);
                DateTime endTime = allValues ? DateTime.MaxValue : startTime + duration;
                string xname = XmlConvert.EncodeLocalName(schema.Name);
                long defaultX = 0;
                foreach (var row in data.Root.Elements("row"))
                {
                    foreach (var e in row.Elements(xname))
                    {
                        long? us = GetTimeMicroseconds(row);
                        long x = us.HasValue ? us.Value : defaultX++;
                        DateTime time = this.startTime.AddMilliseconds(x / 1000);
                        if (allValues || (us.HasValue && time > startTime && time < endTime))
                        {
                            string value = e.Value;
                            double d = 0;
                            if (double.TryParse(value, out d))
                            {
                                yield return new DataValue() { X = x, Y = d };
                            }
                        }
                    }
                }
            }
        }

        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            throw new NotImplementedException("LiveQuery");
        }

        public IEnumerable<LogEntry> GetRows(string typeName, DateTime startTime, TimeSpan duration)
        {
            throw new NotImplementedException("GetRows");
        }

        public DateTime GetTime(ulong timeMs)
        {
            throw new NotImplementedException("GetTime");
        }

        public async Task Load(string fileName, ProgressUtility progress)
        {
            flights.Clear();
            // CSV doesn't have realtime clock, so go with the file date instead.
            this.startTime = File.GetLastWriteTime(fileName);

            // time (us)
            long min = long.MaxValue;
            long max = long.MinValue;


            await Task.Run(() =>
            {
                timeElementName = null;
                using (Stream s = File.OpenRead(fileName))
                {
                    XmlNameTable nametable = new NameTable();
                    using (XmlCsvReader reader = new XmlCsvReader(s, System.Text.Encoding.UTF8, new Uri(fileName), nametable))
                    {
                        progress.ShowProgress(0, s.Length, s.Position);
                        reader.FirstRowHasColumnNames = true;
                        data = XDocument.Load(reader);

                        this.schema = new LogItemSchema() { Name = "CsvLog", Type = "Root" };
                        var item = new LogItemSchema() { Name = System.IO.Path.GetFileName(fileName), Type = "Item" };
                        
                        // create the schema
                        List<LogItemSchema> children = new List<Model.LogItemSchema>();
                        foreach (String name in reader.ColumnNames)
                        {
                            if (timeElementName == null && name.Contains("time"))
                            {
                                timeElementName = name;
                            }
                            children.Add(new LogItemSchema() { Name = name, Parent = this.schema, Type = "Double" });
                        }

                        item.ChildItems = children;
                        this.schema.ChildItems = new List<Model.LogItemSchema>(new LogItemSchema[] { item });

                        progress.ShowProgress(0, s.Length, s.Position);
                    }
                }

                foreach (var e in data.Root.Elements())
                {
                    long? i = GetTimeMicroseconds(e);
                    if (i.HasValue)
                    {
                        if (i.Value < min)
                        {
                            min = i.Value;
                        }
                        if (i > max)
                        {
                            max = i.Value;
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

        long? GetTimeMicroseconds(XElement e)
        {
            string time = (string)e.Attribute(timeElementName);
            if (time == null)
            {
                time = (string)e.Element(timeElementName);
            }
            double i = 0;
            if (double.TryParse(time, out i))
            {
                if (timeElementName == "time")
                {
                    return (long)(i * 1000000);
                }
                return (long)i;
            }
            return null;
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
