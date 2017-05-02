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

        // in case the Name is not unique (as is the case with multi_id formats in px4 logs) 
        // the IDataLog implementor can use this field instead.
        public int Id { get; set; }

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
                return Type == "sbyte" || Type == "byte" || Type == "SByte" || Type == "Byte" || Type == "UInt8" || Type == "Int8" ||
                   Type == "Int16" || Type == "UInt16" ||
                   Type == "Int32" || Type == "UInt32" || Type == "Int64" || Type == "UInt64" || Type == "Float" ||
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

        internal void Combine(LogItemSchema s)
        {
            Dictionary<string, LogItemSchema> index = new Dictionary<string, Model.LogItemSchema>();
            if (this.HasChildren)
            {
                foreach (var i in this.ChildItems.ToArray())
                {
                    index[i.Name] = i;
                }
            }
            if (s.HasChildren)
            {
                foreach (var child in s.ChildItems.ToArray())
                {
                    LogItemSchema found = null;
                    if (index.TryGetValue(child.Name, out found))
                    {
                        found.Combine(child);
                    }
                    else
                    {
                        LogItemSchema copy = child.Clone();
                        copy.Parent = this;
                        this.ChildItems.Add(child);
                    }
                }
            }
        }

        private LogItemSchema Clone()
        {
            LogItemSchema copy = new LogItemSchema() { Id = this.Id, Name = this.Name, Type = this.Type };
            if (this.HasChildren)
            {
                List<LogItemSchema> copyChildren = new List<Model.LogItemSchema>();
                foreach (var child in this.ChildItems.ToArray())
                {
                    var childClone = child.Clone();
                    childClone.Parent = copy;
                    copyChildren.Add(childClone);
                }
                copy.ChildItems = copyChildren;
            }
            return copy;
        }
    }

}
