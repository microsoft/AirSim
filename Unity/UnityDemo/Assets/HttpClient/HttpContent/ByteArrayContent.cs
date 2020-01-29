using System.Collections.Generic;
using System.IO;

namespace CI.HttpClient
{
    public class ByteArrayContent : IHttpContent
    {
        private readonly byte[] _content;

        public ContentReadAction ContentReadAction
        {
            get { return ContentReadAction.Single; }
        }

        /// <summary>
        /// Not currently implemented
        /// </summary>
        public IDictionary<string, string> Headers { get; private set; }

        /// <summary>
        /// Send content encoded as a byte array, the specified mediaType sets the Content Type header
        /// </summary>
        /// <param name="content">The byte array to send</param>
        /// <param name="mediaType">The media type</param>
        public ByteArrayContent(byte[] content, string mediaType)
        {
            _content = content;

            Headers = new Dictionary<string, string>()
            {
                { "Content-Type", mediaType }
            };
        }

        public long GetContentLength()
        {
#if NETFX_CORE
            return _content.Length; 
#else
            return _content.LongLength;
#endif
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
            return new MemoryStream(_content);
        }
    }
}