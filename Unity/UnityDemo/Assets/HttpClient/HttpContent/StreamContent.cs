using System.Collections.Generic;
using System.IO;

namespace CI.HttpClient
{
    public class StreamContent : IHttpContent
    {
        private readonly Stream _stream;

        public ContentReadAction ContentReadAction
        {
            get { return ContentReadAction.Single; }
        }

        /// <summary>
        /// Not currently implemented
        /// </summary>
        public IDictionary<string, string> Headers { get; private set; }

        /// <summary>
        /// Send content based on a stream, the specified mediaType sets the Content Type header
        /// </summary>
        /// <param name="stream">The stream that identifies the content</param>
        /// <param name="mediaType">The media type</param>
        public StreamContent(Stream stream, string mediaType)
        {
            _stream = stream;

            Headers = new Dictionary<string, string>()
            {
                { "Content-Type", mediaType }
            };
        }

        public long GetContentLength()
        {
            return _stream.Length;
        }

        public string GetContentType()
        {
            if (Headers.ContainsKey("Content-Type"))
            {
                return Headers["Content-Type"];
            }

            return string.Empty;
        }

        public Stream ReadAsStream()
        {
            return _stream;
        }
    }
}