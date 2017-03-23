using LogViewer.Controls;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace LogViewer.Model
{
    public interface IDataLog
    {
        /// <summary>
        /// Returns the start time of the data in the log
        /// </summary>
        DateTime StartTime { get; }

        /// <summary>
        /// Returns the duration of the log from timestamp of first entry to the last.
        /// </summary>
        TimeSpan Duration { get; }

        /// <summary>
        /// Get all data values matching the given schema item.
        /// </summary>
        /// <param name="schema">The type of data values we are trying to fetch</param>
        /// <param name="startTime">Limit the values to anything after this startTime</param>
        /// <param name="duration">Limit the values to this time span from the start time</param>
        /// <returns></returns>
        IEnumerable<DataValue> GetDataValues(LogItemSchema schema, DateTime startTime, TimeSpan duration);

        /// <summary>
        /// This method returns a constant stream of values matching the schema in the case that the log
        /// support live reading of values, like hte MavlinkLog.  This stream blocks when there are no more
        /// matching values.
        /// </summary>
        IEnumerable<DataValue> LiveQuery(LogItemSchema schema, CancellationToken token);

        /// <summary>
        /// This method gets the LogEntry items representing a row from the log.
        /// </summary>
        IEnumerable<LogEntry> GetRows(string typeName, DateTime startTime, TimeSpan duration);

        /// <summary>
        /// Extract actual flight times from the log (based on when the motors were actually on).
        /// </summary>
        IEnumerable<Flight> GetFlights();

        /// <summary>
        /// Convert the time in the LogEntryGPS structure to a DateTime that can be cared with a Flight.
        /// </summary>
        /// <returns></returns>
        DateTime GetTime(ulong timeMs);

        /// <summary>
        /// This returns a potentially hierarchical schema showing the data values in this log and their associated names
        /// </summary>
        LogItemSchema Schema { get; }
    }

    public class Flight
    {
        public IDataLog Log { get; set; }
        public DateTime StartTime { get; set; }
        public TimeSpan Duration { get; set; }
        public string Name { get; set; }
    }
}
