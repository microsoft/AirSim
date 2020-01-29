using System;
using System.IO;
using System.Net;
using System.Text;

namespace CI.HttpClient
{
    /// <summary>
    /// Represents a HTTP response message including the status code and data
    /// </summary>
    public class HttpResponseMessage
    {
        /// <summary>
        /// The original request
        /// </summary>
        public HttpWebRequest OriginalRequest
        {
            get; set;
        }

        /// <summary>
        /// The original response
        /// </summary>
        public HttpWebResponse OriginalResponse
        {
            get; set;
        }

        /// <summary>
        /// Length of the content being downloaded
        /// </summary>
        public long ContentLength
        {
            get; set;
        }

        /// <summary>
        /// How much content as been downloaded so far
        /// </summary>
        public long TotalContentRead
        {
            get; set;
        }

        /// <summary>
        /// How much content has been downloaded since the last http response message was raised
        /// </summary>
        public long ContentReadThisRound
        {
            get; set;
        }

        /// <summary>
        /// Percentage completion of the download
        /// </summary>
        public int PercentageComplete
        {
            get
            {
                if (ContentLength <= 0)
                {
                    return 100;
                }
                else
                {
                    return (int)(((double)TotalContentRead / ContentLength) * 100);
                }
            }
        }

        /// <summary>
        /// The http status code
        /// </summary>
        public HttpStatusCode StatusCode
        {
            get; set;
        }

        /// <summary>
        /// The reason for the http status code
        /// </summary>
        public string ReasonPhrase
        {
            get; set;
        }

        /// <summary>
        /// Can the status code be considered a success code
        /// </summary>
        public bool IsSuccessStatusCode
        {
            get { return ((int)StatusCode >= 200) && ((int)StatusCode <= 299); }
        }

        /// <summary>
        /// The exception raised (if there was one)
        /// </summary>
        public Exception Exception
        {
            get; set;
        }

        /// <summary>
        /// Did the server return any content
        /// </summary>
        public bool HasContent
        {
            get { return _responseData != null && ContentReadThisRound > 0; }
        }

        private readonly byte[] _responseData;

        public HttpResponseMessage()
        {
        }

        public HttpResponseMessage(byte[] responseData)
        {
            _responseData = responseData;
        }

        /// <summary>
        /// Returns the response as a string
        /// </summary>
        /// <returns>The response as a string</returns>
        public string ReadAsString()
        {
            return Encoding.UTF8.GetString(_responseData);
        }

        /// <summary>
        /// Returns the response as a string using the specified encoding
        /// </summary>
        /// <param name="encoding">The encoding used by the server</param>
        /// <returns>The response as a string</returns>
        public string ReadAsString(Encoding encoding)
        {
            return encoding.GetString(_responseData);
        }

        /// <summary>
        /// Returns the response as a byte array
        /// </summary>
        /// <returns>The response as a byte array</returns>
        public byte[] ReadAsByteArray()
        {
            return _responseData;
        }

        /// <summary>
        /// Returns the response as a stream
        /// </summary>
        /// <returns>The response as a stream</returns>
        public Stream ReadAsStream()
        {
            return new MemoryStream(_responseData);
        }
    }
}