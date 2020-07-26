//*****************************************************************************
//* File: MessageException.cs
//* Project: Firefly (Microsoft Hackaton 2020)
//* Description: Exception class for AOLUSS REST Client
//* Author: Mark West (mark.west@microsoft.com)
//*****************************************************************************

using System;

namespace AolussClientConsole
{
    public class MessageException : Exception
    {
        public MessageException()
        {
        }

        public MessageException(string message)
            : base(message)
        {
        }

        public MessageException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }
}
