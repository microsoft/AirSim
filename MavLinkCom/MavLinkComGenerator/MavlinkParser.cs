// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using System.Xml.Serialization;

namespace MavLinkComGenerator
{
    static class MavlinkParser
    {
        public static MavLink Parse(string xmlFile)
        {
            using (var fs = new FileStream(xmlFile, FileMode.Open, FileAccess.Read))
            {
                XmlSerializer s = new XmlSerializer(typeof(MavLink));
                MavLink mavlink = (MavLink)s.Deserialize(fs);

                // unfortunately the <extensions/> sections in each <message> is not something the XmlSerializer can handle,
                // so we have to post-process the MavLink tree with this additional information.
                fs.Seek(0, SeekOrigin.Begin);
                XDocument doc = XDocument.Load(fs);
                foreach (XElement message in doc.Root.Element("messages").Elements("message"))
                {
                    string id = (string)message.Attribute("id");
                    MavMessage msg = (from m in mavlink.messages where m.id == id select m).FirstOrDefault();
                    int count = 0;
                    foreach (XElement child in message.Elements())
                    {
                        string childName = child.Name.LocalName;
                        if (childName == "field")
                        {
                            count++;
                        }
                        else if (childName == "extensions")
                        {
                            msg.ExtensionPos = count;
                            break;
                        }
                    }
                }


                return mavlink;
            }
        }
    }
}
