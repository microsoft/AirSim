using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LogViewer.Model
{
    public class DataSeries
    {
        public string Name { get; set; }

        public List<DataValue> Values { get; set; }
    }

    public class DataValue
    {
        public double X { get; set; }
        public double Y { get; set; }

        public string Label { get; set; }

        // an optional object representing the underlying value.
        public object UserData { get; set; }

    }

    public class Range
    {
        public double Minimum { get; set; }
        public double Maximum { get; set; }
    }

}
