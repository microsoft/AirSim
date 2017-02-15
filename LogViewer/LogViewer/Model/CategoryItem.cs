using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LogViewer.Model
{

    /// <summary>
    /// This class is used to provide schema information about the IDataLog, notice it is hierarchical, so it can
    /// be bound to a Tree control so user can drill into log item data at any level, the selected items can then
    /// be queried via IDataLog.GetDataValues.
    /// </summary>
    public class LogItemSchema
    {
        public LogItemSchema Parent { get; set; }

        public string Name { get; set; }

        public string Type { get; set; }

        public List<LogItemSchema> ChildItems { get; set; }

        public bool HasChildren
        {
            get
            {
                return ChildItems != null && ChildItems.Count > 0;
            }
        }

        public bool IsNumeric
        {
            get
            {
                return Type == "sbyte" || Type == "byte" || Type == "SByte" || Type == "Byte" ||
                    Type == "Int16" || Type == "UInt16" ||
                    Type == "Int32" || Type == "UInt32" || Type == "Int64" || Type == "UInt64" ||
                    Type == "Single" || Type == "Double";
            }
        }

        public LogItemSchema Root
        {
            get
            {
                if (this.Parent == null)
                    return this;
                return this.Parent.Root;
            }
        }
    }

}
