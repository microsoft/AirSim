// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
                return mavlink;
            }
        }
    }
}
