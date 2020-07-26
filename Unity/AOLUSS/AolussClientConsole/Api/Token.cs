//*****************************************************************************
//* File: Token
//* Project: Firefly (Microsoft Hackaton 2020)
//* Description: Demo of AOLUSS REST Client
//* Author: Mark West (mark.west@microsoft.com)
//*****************************************************************************

namespace AolussClientConsole
{
    using AolussClientLib.Api;
    using System;
    using AolussClientLib.Model;

    class Token
    {
        private TokenApi instance;
        private bool isInitialized = false;

        const string UNINIT_MSG = "Call Initialize() before invoking an operation";

        /// <summary>
        /// Constructor
        /// </summary>
        public Token()
        {

        }

        /// <summary>
        /// Initialize
        /// </summary>
        public void Initialize()
        {
            instance = new TokenApi();
        }

        /// <summary>
        /// Get Operator Status
        /// </summary>
        public JsonWebToken GetOperatorToken()
        {
            if (!isInitialized)
                throw new InvalidOperationException(UNINIT_MSG);

            try
            {
                return instance.GetOperatorToken();
            }
            catch (Exception ex)
            {
                throw new MessageException("GetOperatorToken Failed", ex);
            }
        }
    }
}
