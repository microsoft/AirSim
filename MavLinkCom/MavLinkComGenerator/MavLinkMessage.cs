// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using System.Xml.Serialization;

namespace MavLinkComGenerator
{
    [XmlRoot(ElementName="mavlink")]
    public class MavLink
    {
        public MavLink() { }

        public string version { get; set; }
        public string dialog { get; set; }

        [XmlArrayItem(ElementName ="enum")]
        public List<MavEnum> enums { get; set; }

        [XmlArrayItem(ElementName = "message")]
        public List<MavMessage> messages { get; set; }
    }

    public class MavEnum
    {
        public MavEnum() { }

        [XmlAttribute]
        public string name { get; set;}

        public string description { get; set; }

        [XmlElement(ElementName = "entry")]
        public List<MavEnumEntry> entries { get; set; }
    }

    public class MavParam
    {
        [XmlAttribute]
        public int index { get; set; }

        [XmlText]
        public string description { get; set; }

        [XmlAttribute]
        public string label { get; set; }

    }

    public class MavEnumEntry
    {
        public MavEnumEntry() { }

        [XmlAttribute]
        public int value { get; set; }

        [XmlAttribute]
        public string name { get; set; }

        public string description { get; set; }

        // for MAV_CMD definitions.
        [XmlElement(ElementName = "param")]
        public List<MavParam> parameters { get; set; }
    }

    public class MavField
    {
        [XmlAttribute]
        public string type { get; set; }

        public bool isArray { get; set; }

        public int array_length { get; set; }

        [XmlAttribute]
        public string name { get; set; }

        [XmlText]
        public string description { get; set; }

        public int offset { get; set; }

        public MavField() { }
    }

    public class MavMessage
    {
        [XmlAttribute]
        public string id { get; set; }
        [XmlAttribute]
        public string name { get; set; }

        public string description { get; set; }

        [XmlElement(ElementName ="field")]
        public List<MavField> fields { get; set; }

        public int ExtensionPos { get; set; }

        public MavMessage() { }
    }
}
