using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace LogViewer.Model
{
    class KmlDataLog : IDataLog
    {
        DateTime startTime;
        DateTime endTime;
        XNamespace kmlns = XNamespace.Get("http://www.opengis.net/kml/2.2");
        XNamespace gxns = XNamespace.Get("http://www.google.com/kml/ext/2.2");
        LogItemSchema schema;
        List<Flight> flights = new List<Flight>();
        List<LogEntry> log = new List<LogEntry>();

        public KmlDataLog()
        {
            startTime = DateTime.Now;
            endTime = startTime;
        }

        public void Load(XDocument doc)
        {
            var root = doc.Root;
            if (root.Name.LocalName != "kml" || root.Name.Namespace != kmlns)
            {
                throw new Exception("Doesn't appear to be KML file, expecting XML namespace '" + kmlns + "'");
            }

            Dictionary<string, XElement> styles = new Dictionary<string, XElement>();

            var document = root.Element(kmlns + "Document");
            if (document != null) {
                foreach (XElement style in document.Elements(kmlns + "Style"))
                {
                    string id = (string)style.Attribute("id");
                    if (!string.IsNullOrEmpty(id))
                    {
                        styles[id] = style;
                    }
                }
                foreach (XElement placemark in document.Elements(kmlns + "Placemark"))
                {
                    string name = (string)document.Element(kmlns + "name");
                    string styleUrl = (string)document.Element(kmlns + "styleUrl");
                    bool started = false;
                    DateTime current = DateTime.Now;
                    DateTime startTime = current;
                    DateTime endTime = startTime;

                    // "Track" data is one style of KML that we can load GPS data from
                    foreach (XElement track in placemark.Elements(gxns + "Track"))
                    {
                        foreach (XElement child in track.Elements())
                        {
                            switch(child.Name.LocalName)
                            {
                                case "altitudeMode":
                                    // absolute?
                                    break;
                                case "interpolate":
                                    // 0?
                                    break;
                                case "when":
                                    var time = DateTime.Parse((string)child);
                                    if (!started)
                                    {
                                        startTime = time;
                                        started = true;
                                    }
                                    current = time;
                                    endTime = time;
                                    break;
                                case "coord":
                                    AddGpsData(current, startTime, (string)child, ' ', "OrderLatLong");
                                    break;
                            }
                        }
                    }

                    // "LineString" is another format.
                    foreach (XElement ls in placemark.Elements(kmlns + "LineString"))
                    {
                        if (!started)
                        {
                            started = true;
                        }

                        XElement coordinates = ls.Element(kmlns + "coordinates");
                        foreach (string line in ((string)coordinates).Split('\n'))
                        {
                            // fake time, since this format has no time.
                            current = current + TimeSpan.FromMilliseconds(1);
                            endTime = current;
                            AddGpsData(current, startTime, line, ',', "OrderLongLat");
                        }
                    }


                    if (started)
                    {
                        this.flights.Add(new Flight()
                        {
                            Duration = endTime - startTime,
                            Log = this,
                            Name = name,
                            StartTime = startTime
                        });
                    }
                }
            }


        }

        private void AddGpsData(DateTime current, DateTime startTime, string row, char separator, string order)
        {
            string[] parts = row.Split(separator);
            if (parts.Length == 3)
            {
                double lat, lng, alt;
                double.TryParse(parts[0], out lat);
                double.TryParse(parts[1], out lng);
                double.TryParse(parts[2], out alt);

                if (order == "OrderLongLat")
                {
                    // switch them.
                    double temp = lng;
                    lng = lat;
                    lat = temp;
                }

                var gps = new LogEntry()
                {
                    Name = "GPS",
                    Timestamp = (ulong)current.Ticks
                };
                gps.SetField("TimeMS", (UInt32)((current - startTime).Milliseconds));
                gps.SetField("Lat", lat);                
                gps.SetField("Lng", lng);
                gps.SetField("Alt", alt);
                log.Add(gps);
            }
        }

        public TimeSpan Duration
        {
            get { return endTime - startTime; }
        }

        private void Schematize(LogItemSchema schema, IEnumerable<LogField> items)
        {
            foreach (var item in items)
            {
                // is this a new item we haven't seen before?
                
                if (!schema.HasChild(item.Name))
                {
                    var typeName = item.Name;
                    if (item is LogEntry)
                    {
                        // recurrse down the structure
                        var s = new LogItemSchema() { Name = item.Name, Type = typeName };
                        schema.AddChild(s);
                        Schematize(s, ((LogEntry)item).GetFields());
                    }
                    else
                    {
                        // leaf node
                        typeName = item.Value.GetType().Name;
                        var leaf = new LogItemSchema() { Name = item.Name, Type = typeName };
                        schema.AddChild(leaf);
                    }
                }
            }
        }

        public LogItemSchema Schema
        {
            get {
                if (this.schema == null)
                {
                    this.schema = new LogItemSchema() { Name = "KmlLog", Type = "Root" };
                    Schematize(this.schema, this.log);
                }
                return this.schema;
            }
        }

        public DateTime StartTime
        {
            get { return this.startTime; }
        }

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

        public IEnumerable<Flight> GetFlights()
        {
            return this.flights;
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

        public DateTime GetTime(ulong timeMs)
        {
            TimeSpan ms = TimeSpan.FromMilliseconds(timeMs);
            return this.startTime + ms;
        }

        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            throw new NotImplementedException();
        }
    }
}
