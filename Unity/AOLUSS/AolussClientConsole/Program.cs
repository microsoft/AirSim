//*****************************************************************************
//* File: Program.cs
//* Project: Firefly (Microsoft Hackaton 2020)
//* Description: Demo of AOLUSS REST Client
//* Author: Mark West (mark.west@microsoft.com)
//*****************************************************************************

using System;

namespace AolussClientConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            runDemo();
        }

        /// <summary>
        /// Run Demo access to AOLUSS REST service
        /// </summary>
        static void runDemo()
        {
            try
            {
                Operations.RunDemo();
            }
            catch(Exception ex)
            {
                if (null == ex.InnerException)
                    Console.WriteLine("Exception : " + ex.Message);
                else
                    Console.WriteLine("Exception : " + ex.Message + " : " + ex.InnerException.Message);
            }
        }
    }
}
