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
        string timeElementName = XmlConvert.EncodeLocalName("time (us)");

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
                int defaultX = 0;
                foreach (var e in data.Root.Elements(xname))
                {
                    int? us = GetTimeMicroseconds(e);
                    int x = us.HasValue ? us.Value : defaultX++;
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
            int min = int.MaxValue;
            int max = int.MinValue;


            await Task.Run(() =>
            {

                using (Stream s = File.OpenRead(fileName))
                {
                    XmlNameTable nametable = new NameTable();
                    using (XmlCsvReader reader = new XmlCsvReader(s, System.Text.Encoding.UTF8, new Uri(fileName), nametable))
                    {
                        progress.ShowProgress(0, s.Length, s.Position);
                        reader.FirstRowHasColumnNames = true;
                        data = XDocument.Load(reader);

                        this.schema = new LogItemSchema() { Name = "CsvDataLog", Type = "Root" };

                        // create the schema
                        List<LogItemSchema> children = new List<Model.LogItemSchema>();
                        foreach (String name in reader.ColumnNames)
                        {
                            children.Add(new LogItemSchema() { Name = name, Parent = this.schema });
                        }

                        this.schema.ChildItems = children;

                        progress.ShowProgress(0, s.Length, s.Position);
                    }
                }

                foreach (var e in data.Root.Elements())
                {
                    int? i = GetTimeMicroseconds(e);
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

        int? GetTimeMicroseconds(XElement e)
        {
            string time = (string)e.Attribute(timeElementName);
            int i = 0;
            if (int.TryParse(time, out i))
            {
                return i;
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
