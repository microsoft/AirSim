using LogViewer.Utilities;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static Microsoft.Networking.Mavlink.MAVLink;
using System.Threading;

namespace LogViewer.Model
{
    class rows
    {
        public object[] data;
    };

    class JSonDataLog : IDataLog
    {
        private DateTime startTime;
        private string file;
        private ProgressUtility progress;
        private Stream fileStream;

        public DateTime StartTime
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public TimeSpan Duration
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public LogItemSchema Schema
        {
            get
            {
                throw new NotImplementedException();
            }
        }

        public async Task Load(string file, ProgressUtility progress)
        {
            this.file = file;
            this.progress = progress;
            string ext = System.IO.Path.GetExtension(file).ToLowerInvariant();

            bool hasStartTime = false;
            this.startTime = DateTime.MinValue;
            //DateTime? gpsStartTime = null;
            //ulong gpsAbsoluteOffset = 0;
            //ulong logStartTime = 0;

            List<LogEntry> rows = new List<LogEntry>();

            rows r = new Model.rows();
            r.data = new object[1] {
                new mavlink_param_request_read_t()
                {
                     param_id = new byte[16]
                }
            };
            string result = JsonConvert.SerializeObject(r);

            Debug.WriteLine(result);

            await Task.Run(() =>
            {
                    using (Stream fileStream = File.OpenRead(file))
                    {
                        this.fileStream = fileStream;
                        using (TextReader textReader = new StreamReader(fileStream))
                        {
                            JsonTextReader reader = new JsonTextReader(textReader);
                            while (reader.Read())
                            {
                                ReportProgress();
                                //if (reader.TokenType == JsonToken.StartObject)
                                //{
                                //    ReadObject(reader);
                                //}
                                if (reader.Value != null)
                                {
                                    //Console.WriteLine("Token: {0}, Value: {1}", reader.TokenType, reader.Value);
                                }
                                else
                                {
                                   // Console.WriteLine("Token: {0}", reader.TokenType);
                                }
                            }

                            if (!hasStartTime)
                            {
                                hasStartTime = true;
                            }
                            //if (!gpsStartTime.HasValue && row.Name == "GPS")
                            //{
                            //    ulong time = row.GetField<ulong>("GPSTime");
                            //    if (time > 0)
                            //    {
                            //        // ok, we got a real GPS time
                            //        gpsStartTime = GetTime(time);
                            //        // backtrack and fix up initial rows.
                            //        if (hasStartTime)
                            //        {
                            //            // compute offset from PX4 boot time (which is the log.CurrentTime) and the absolute GPS real time clock
                            //            // so we can add this offset to PX4 log.CurrentTime so all our times from here on out are in real time.
                            //            gpsStartTime = gpsStartTime.Value.AddMilliseconds((double)logStartTime - (double)log.CurrentTime);
                            //        }
                            //        ulong linuxEpochMicroseconds = ((ulong)(gpsStartTime.Value.ToUniversalTime() - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000);
                            //        gpsAbsoluteOffset = linuxEpochMicroseconds - logStartTime;
                            //        this.startTime = gpsStartTime.Value;
                            //        // and fix existing log entries
                            //        foreach (LogEntry e in rows)
                            //        {
                            //            // add GPS absolute offset to the timestamp.
                            //            e.Timestamp += gpsAbsoluteOffset;
                            //        }
                            //        hasStartTime = true;
                            //    }
                        }

                        //if (gpsStartTime.HasValue)
                        //{
                        //    // add GPS absolute offset to the timestamp.
                        //    row.Timestamp += gpsAbsoluteOffset;
                        //}

                        //rows.Add(row);

                        //if (row.Format.Name == "PARM")
                        //{
                        //    string name = row.GetField<string>("Name");
                        //    float value = row.GetField<float>("Value");
                        //    Debug.WriteLine("{0}={1}", name, value);
                        //}
                    }
                    //if (log.CurrentTime != 0)
                    //{
                    //    DateTime endTime = GetTime(log.CurrentTime + gpsAbsoluteOffset);
                    //    this.duration = endTime - startTime;
                    //    Debug.WriteLine("StartTime={0}, EndTime={1}, Duration={2}", startTime.ToString(), endTime.ToString(), duration.ToString());
                    //}

                    //CreateSchema(log);                
            });

            //this.data = rows;
        }

        private void ReportProgress()
        {
            progress.ShowProgress(0, fileStream.Length, fileStream.Position);
        }

        private void ReadObject(JsonTextReader reader)
        {
            string propertyName = null;

            while (reader.Read() && reader.TokenType != JsonToken.EndObject)
            {
                reader.Read();
                if (reader.TokenType == JsonToken.PropertyName)
                {
                    propertyName = reader.ReadAsString();
                }
            }
        }

        public IEnumerable<DataValue> GetDataValues(LogItemSchema schema, DateTime startTime, TimeSpan duration)
        {
            throw new NotImplementedException();
        }

        public IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token)
        {
            throw new NotImplementedException();
        }

        public IEnumerable<LogEntry> GetRows(string typeName, DateTime startTime, TimeSpan duration)
        {
            throw new NotImplementedException();
        }

        public IEnumerable<Flight> GetFlights()
        {
            throw new NotImplementedException();
        }

        public DateTime GetTime(ulong timeMs)
        {
            throw new NotImplementedException();
        }
    }
}
