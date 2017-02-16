// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MavLinkComGenerator
{
    class MavLinkGenerator
    {
        StreamWriter header;
        StreamWriter impl;
        MavLink definitions;

        public void Generate(MavLink definitions, string outputDirectory)
        {
            this.definitions = definitions;

            using (header = new StreamWriter(Path.Combine(outputDirectory, "MavLinkMessages.hpp")))
            {
                using (impl = new StreamWriter(Path.Combine(outputDirectory, "MavLinkMessages.cpp")))
                {
                    header.WriteLine("#ifndef MavLinkCom_MavLinkMessages_hpp");
                    header.WriteLine("#define MavLinkCom_MavLinkMessages_hpp");
                    header.WriteLine("");
                    header.WriteLine("#include <stdint.h>");
                    header.WriteLine("#include <string>");
                    header.WriteLine("#include \"MavLinkMessageBase.hpp\"");
                    header.WriteLine("");
                    header.WriteLine("namespace mavlinkcom");
                    header.WriteLine("{");
                    header.WriteLine("");

                    impl.WriteLine("#include \"MavLinkMessages.hpp\"");
                    impl.WriteLine("using namespace mavlinkcom;");
                    impl.WriteLine("");

                    GenerateEnums();
                    GenerateMessages();
                    GenerateCommands();

                    header.WriteLine("}");
                    header.WriteLine("");
                    header.WriteLine("#endif");
                }
            }
        }

        private void GenerateCommands()
        {
            var cmds = (from e in definitions.enums where e.name == "MAV_CMD" select e).FirstOrDefault();
            if (cmds == null)
            {
                return;
            }
            foreach (var cmd in cmds.entries)
            {
                Console.WriteLine("generating command {0}", cmd.name);

                if (!string.IsNullOrWhiteSpace(cmd.description))
                {
                    WriteComment("", cmd.description);
                }
                string name = CamelCase(cmd.name);
                header.WriteLine("class {0} : public MavLinkCommand {{", name);
                header.WriteLine("public:");
                header.WriteLine("    const static uint16_t kCommandId = {0};", cmd.value);
                header.WriteLine("    {0}() {{ command = kCommandId; }}", name);
                UniqueList unique = new UniqueList();
                foreach (var p in cmd.parameters)
                {
                    string fieldName = NameFromDescription(p.description);
                    if (fieldName != "Empty" && fieldName != "Reserved")
                    {
                        if (!string.IsNullOrWhiteSpace(p.description))
                        {
                            WriteComment("    ", p.description);
                        }
                        header.WriteLine("    float {0} = 0;", unique.Add(fieldName));
                    }
                }

                unique = new UniqueList();
                header.WriteLine("protected:");
                header.WriteLine("    virtual void pack();");

                impl.WriteLine("void {0}::pack() {{", name);
                int i = 0;
                foreach (var p in cmd.parameters)
                {
                    i++;
                    string fieldName = NameFromDescription(p.description);
                    if (fieldName != "Empty" && fieldName != "Reserved")
                    {
                        impl.WriteLine("    param{0} = {1};", i, unique.Add(fieldName));
                    }
                }
                impl.WriteLine("}");

                header.WriteLine("    virtual void unpack();");
                impl.WriteLine("void {0}::unpack() {{", name);

                unique = new UniqueList();
                i = 0;
                foreach (var p in cmd.parameters)
                {
                    i++;
                    string fieldName = NameFromDescription(p.description);
                    if (fieldName != "Empty" && fieldName != "Reserved")
                    {
                        impl.WriteLine("    {1} = param{0};", i, unique.Add(fieldName));
                    }
                }
                impl.WriteLine("}");
                header.WriteLine("};");
            }


        }

        char[] nameSeparators = new char[] { '_' };

        private string CamelCase(string name)
        {
            StringBuilder sb = new StringBuilder();
            foreach (string word in name.Split(nameSeparators, StringSplitOptions.RemoveEmptyEntries))
            {
                string lower = word.ToLowerInvariant();
                string camel = Char.ToUpperInvariant(lower[0]).ToString();
                if (lower.Length > 1)
                {
                    camel += lower.Substring(1);
                }
                sb.Append(camel);
            }
            return sb.ToString();
        }

        char[] terminators = new char[] { '.', '-', ':', ',', '(', '[' };
        char[] whitespace = new char[] { ' ', '\t', '\n', '\r' };

        private string NameFromDescription(string description)
        {
            StringBuilder sb = new StringBuilder();
            int i = description.IndexOf("this sets");
            if (i > 0)
            {
                description = description.Substring(i + 9);
            }
            i = description.IndexOfAny(terminators);
            if (i > 0)
            {
                description = description.Substring(0, i);
            }
            string[] words = description.Split(whitespace, StringSplitOptions.RemoveEmptyEntries);
            int count = 0;
            for (int j = 0; j < words.Length; j++)
            {
                string w = words[j];
                w = w.Replace("'", "");
                w = w.Replace("/", "");
                w = w.Replace("?", "");
                w = CamelCase(w);
                if (j > 0 && (w == "In" || w == "From" || w == "And" || w == "As" || w == "The" || w == "And" || w == "It" || w == "On"))
                {
                    continue; // skip filler words
                }
                if (j == 0 && Char.IsDigit(w[0]))
                {
                    w = "p" + w; // make it a legal C++ identifier.
                }
                sb.Append(w);
                count++;
                if (count == 3)
                {
                    break;
                }
            }
            return sb.ToString();
        }


        private void GenerateMessages()
        {
            foreach (var m in definitions.messages)
            {
                string name = CamelCase(m.name);
                Console.WriteLine("generating message {0}", name);
                if (!string.IsNullOrWhiteSpace(m.description))
                {
                    WriteComment("", m.description);
                }
                header.WriteLine("class MavLink{0} : public MavLinkMessageBase {{", name);
                header.WriteLine("public:");
                header.WriteLine("    const static uint8_t kMessageId = {0};", m.id);
                header.WriteLine("    MavLink{0}() {{ msgid = kMessageId; }}", name);

                // find array types and split the type string "int[16]" into element type "int" and isArray=true and array_length=16.
                foreach (var field in m.fields)
                {
                    string type = field.type;
                    if (type.Contains('['))
                    {
                        var tuple = ParseArrayType(type);
                        field.type = tuple.Item1;
                        field.isArray = true;
                        field.array_length = tuple.Item2;
                    }
                }

                // mavlink packs the fields by descending size for some odd reason.
                m.fields = m.fields.OrderByDescending(x => typeSize[x.type]).ToList();

                int length = m.fields.Count;
                for (int i = 0; i < length; i++)
                {
                    var field = m.fields[i];
                    if (!string.IsNullOrWhiteSpace(field.description))
                    {
                        WriteComment("    ", field.description);
                    }
                    var type = field.type;
                    if (type == "uint8_t_mavlink_version")
                    {
                        type = "uint8_t";
                    }
                    if (field.isArray)
                    {
                        header.WriteLine("    {0} {1}[{2}] = {{ 0 }};", type, field.name, field.array_length);
                    }
                    else
                    {
                        header.WriteLine("    {0} {1} = 0;", type, field.name);
                    }
                }

                header.WriteLine("protected:");

                header.WriteLine("    virtual int pack(char* buffer) const;");
                impl.WriteLine("int MavLink{0}::pack(char* buffer) const {{", name);

                int offset = 0;
                for (int i = 0; i < length; i++)
                {
                    var field = m.fields[i];
                    var type = field.type;
                    if (type == "uint8_t_mavlink_version")
                    {
                        type = "uint8_t";
                    }
                    int size = typeSize[type];
                    if (field.isArray)
                    {
                        // it is an array.
                        impl.WriteLine("    pack_{0}_array({1}, buffer, reinterpret_cast<const {0}*>(&this->{2}[0]), {3});", type, field.array_length, field.name, offset);
                        size *= field.array_length;
                    }
                    else
                    {
                        impl.WriteLine("    pack_{0}(buffer, reinterpret_cast<const {0}*>(&this->{1}), {2});", type, field.name, offset);
                    }
                    offset += size;
                }
                impl.WriteLine("    return {0};", offset);
                impl.WriteLine("}");
                impl.WriteLine("");

                header.WriteLine("    virtual int unpack(const char* buffer);");
                impl.WriteLine("int MavLink{0}::unpack(const char* buffer) {{", name);

                offset = 0;
                for (int i = 0; i < length; i++)
                {
                    var field = m.fields[i];
                    var type = field.type;
                    if (type == "uint8_t_mavlink_version")
                    {
                        type = "uint8_t";
                    }
                    int size = typeSize[type];
                    if (field.isArray)
                    {
                        // it is an array.
                        impl.WriteLine("    unpack_{0}_array({1}, buffer, reinterpret_cast<{0}*>(&this->{2}[0]), {3});", type, field.array_length, field.name, offset);
                        size *= field.array_length;
                    }
                    else
                    {
                        impl.WriteLine("    unpack_{0}(buffer, reinterpret_cast<{0}*>(&this->{1}), {2});", type, field.name, offset);
                    }
                    offset += size;
                }
                impl.WriteLine(" return {0};", offset);
                impl.WriteLine("}");
                impl.WriteLine("");

                header.WriteLine("};");
                header.WriteLine("");
            }
        }

        public Tuple<string,int> ParseArrayType(string type)
        {
            int i = type.IndexOf('[');
            int k = type.IndexOf(']', i);
            if (k > i)
            {
                string name = type.Substring(0, i);
                i++;
                string index = type.Substring(i, k - i);
                int j = 0;
                int.TryParse(index, out j);
                return new Tuple<string, int>(name, j);
            }
            throw new Exception("Invalid array type specification: " + type);
        }

        public static Dictionary<string, int> typeSize = new Dictionary<string, int>()
        {
            { "float"    , 4},
            {"double"   , 8 },
            {"char"     , 1},
            {"int8_t"   , 1},
            {"uint8_t"  , 1},
            {"uint8_t_mavlink_version"  , 1 },
            {"int16_t"  , 2 },
            {"uint16_t" , 2 },
            {"int32_t"  , 4 },
            {"uint32_t" , 4 },
            {"int64_t"  , 8 },
            {"uint64_t" , 8 }
        };

        private void GenerateEnums()
        {

            header.WriteLine("enum class MavLinkMessageIds {");
            bool first = true;
            foreach (var m in definitions.messages)
            {
                if (!first)
                {
                    header.WriteLine(",");
                }
                first = false;
                header.Write("    MAVLINK_MSG_ID_{0} = {1}", m.name, m.id);
            }
            header.WriteLine("");

            header.WriteLine("};");


            foreach (var e in definitions.enums)
            {
                Console.WriteLine("generating enum class {0}", e.name);

                if (!string.IsNullOrWhiteSpace(e.description))
                {
                    WriteComment("", e.description);
                }
                header.WriteLine("enum class {0} {{", e.name);

                bool zeroUsed = false;

                int length = e.entries.Count;
                for (int i = 0; i < length; i++)
                {
                    var field = e.entries[i];
                    if (!string.IsNullOrWhiteSpace(field.description))
                    {
                        WriteComment("    ", field.description);
                    }
                    header.Write("    {0}", field.name);
                    if (!zeroUsed || field.value != 0)
                    {
                        header.Write(" = {0}", field.value);
                    }
                    if (!zeroUsed && field.value == 0)
                    {
                        zeroUsed = true;
                    }
                    if (i + 1 < length)
                    {
                        header.Write(",");
                    }
                    header.WriteLine();
                }

                header.WriteLine("};");
                header.WriteLine("");
            }
        }

        private void WriteComment(string indent, string description)
        {
            int lineLength = 0;
            foreach (string word in description.Split(new char[] { '\r', '\n', ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries))
            {
                if (lineLength > 80)
                {
                    header.WriteLine();
                    lineLength = 0;
                }
                if (lineLength == 0)
                {
                    header.Write(indent + "// ");
                    lineLength = indent.Length + 3;
                }
                else
                {
                    lineLength++;
                    header.Write(" ");
                }
                header.Write(word);
                lineLength += word.Length;
            }
            if (lineLength > 0)
            {
                header.WriteLine();
            }
        }

        class UniqueList
        {
            HashSet<string> existing = new HashSet<string>();

            public string Add(String baseName)
            {
                string name = baseName;
                int index = 2;
                while (existing.Contains(name))
                {
                    name = baseName + index++;
                }
                existing.Add(name);
                return name;
            }
        }
    }
}
