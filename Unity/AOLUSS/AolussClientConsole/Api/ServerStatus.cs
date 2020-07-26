//*****************************************************************************
//* File: ServerStatus.cs
//* Project: Firefly (Microsoft Hackaton 2020)
//* Description: Demo of AOLUSS REST Client
//* Author: Mark West (mark.west@microsoft.com)
//*****************************************************************************

namespace AolussClientConsole
{
    using AolussClientLib.Client;
    using AolussClientLib.Api;
    using System;

    class ServerStatus
    {
        private ServerStatusApi instance;
        private bool isInitialized = false;

        const string UNINIT_MSG = "Call Initialize() before invoking an operation";

        /// <summary>
        /// Constructor
        /// </summary>
        public ServerStatus()
        {

        }

        /// <summary>
        /// Initialize
        /// </summary>
        public void Initialize()
        {
            instance = new ServerStatusApi();
        }

        /// <summary>
        /// Get Operator Status
        /// </summary>
        public void GetOperatorStatus()
        {
            instance.GetOperatorStatus();

            if (!isInitialized)
                throw new InvalidOperationException(UNINIT_MSG);

            try
            {
                instance.GetOperatorStatus();
            }
            catch (Exception ex)
            {
                throw new MessageException("GetOperatorStatus Failed", ex);
            }
        }
    }
}
