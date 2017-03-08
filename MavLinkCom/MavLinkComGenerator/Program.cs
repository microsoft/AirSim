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
    class Program
    {
        string xmlInput = null;
        string outputFolder = null;

        private static void PrintUsage()
        {
            Console.WriteLine("USAGE: MavLinkComGenerator -xml:<pathToXML> -out:<pathToOutDir>");
        }

        static void Main(string[] args)
        {
            Program p = new MavLinkComGenerator.Program();
            if (!p.ParseCommandLine(args))
            {
                PrintUsage();
                return;
            }

            try
            {
                p.Run();
            }
            catch (Exception e)
            {
                Console.WriteLine("### Error: " + e.Message);
            }
        }

        private bool ParseCommandLine(string[] args)
        {
            for (int i = 0; i < args.Length; i++)
            {
                string arg = args[i];
                string colonArg = null;
                if (arg.StartsWith("/") || arg.StartsWith("-"))
                {
                    var colonIndex = arg.IndexOf(':');
                    if (colonIndex >= 0)
                    {
                        arg = args[i].Substring(1, colonIndex - 1);
                        colonArg = args[i].Substring(colonIndex + 1);
                    }

                    switch (arg)
                    {
                        case "xml":
                            xmlInput = colonArg;
                            break;
                        case "out":
                            outputFolder = colonArg;
                            break;

                        case "?":
                        case "h":
                        case "help":
                            return false;

                        default:
                            Console.WriteLine("Unrecognized option: {0}", arg);
                            return false;
                    }
                }
            }
            if (string.IsNullOrEmpty(xmlInput))
            {
                Console.WriteLine("Missing \"-xml filename\" option");
                return false;
            }
            if (string.IsNullOrEmpty(outputFolder))
            {
                Console.WriteLine("Missing \"-out directory\" option");
                return false;
            }
            if (!File.Exists(xmlInput))
            {
                Console.WriteLine("File not found: {0}", xmlInput);
                return false;
            }
            if (!Directory.Exists(outputFolder))
            {
                Console.WriteLine("Output directory not found: {0}", outputFolder);
                return false;
            }
            return true;
        }

        void Run()
        {
            //parse the XML
            MavLink mavlink = MavlinkParser.Parse(xmlInput);
            MavLinkGenerator gen = new MavLinkGenerator();
            gen.GenerateMessages(mavlink, outputFolder);
        }
    }
}
