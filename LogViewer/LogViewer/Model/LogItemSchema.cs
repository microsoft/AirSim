using System.Collections.Generic;
using System.Linq;

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

        private List<LogItemSchema> childItems;

        public LogItemSchema()
        {
        }

        public string GetFullName()
        {
            if (Parent != null)
            {
                return Parent.GetFullName() + "." + Name;
            }
            return Name;
        }

        public List<LogItemSchema> CopyChildren()
        {
            var list = this.childItems;
            if (list == null)
            {
                return new List<LogItemSchema>();
            }
            lock (list)
            {
                return new List<LogItemSchema>(list);
            }
        }

        public bool HasChildren
        {
            get
            {                
                return childItems != null && childItems.Count > 0;
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

        public bool IsArray { get; internal set; }

        internal void Combine(LogItemSchema s)
        {
            Dictionary<string, LogItemSchema> index = new Dictionary<string, Model.LogItemSchema>();
            
            if (this.HasChildren)
            {
                lock (this.childItems)
                {
                    foreach (var i in this.childItems)
                    {
                        index[i.Name] = i;
                    }
                }
            }
            if (s.HasChildren)
            {
                lock (s.childItems)
                {
                    foreach (var child in s.childItems)
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
                            this.AddChild(child);
                        }
                    }
                }
            }
        }

        private LogItemSchema Clone()
        {
            LogItemSchema copy = new LogItemSchema() { Id = this.Id, Name = this.Name, Type = this.Type };
            if (this.HasChildren)
            {
                lock (this.childItems)
                {
                    foreach (var child in this.childItems)
                    {
                        var childClone = child.Clone();
                        copy.AddChild(childClone);
                    }
                }
            }
            return copy;
        }

        internal bool HasChild(string name)
        {
            if (this.childItems == null)
            {
                return false;
            }
            lock (this.childItems) 
            {
                return (from c in this.childItems where c.Name == name select c).Any();
            }
        }

        internal void AddChild(LogItemSchema item)
        {
            if (this.childItems == null)
            {
                this.childItems = new List<LogItemSchema>();
            }
            item.Parent = this;

            lock (this.childItems)
            {
                this.childItems.Add(item);
            }
        }

    }

}
