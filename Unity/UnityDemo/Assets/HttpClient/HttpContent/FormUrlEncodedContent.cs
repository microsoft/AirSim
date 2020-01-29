using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace CI.HttpClient
{
    public class FormUrlEncodedContent : IHttpContent
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
        /// Send content encoded as name/value pairs, the Content Type header will be set to application/x-www-form-urlencoded
        /// </summary>
        /// <param name="nameValueCollection">The key/value pairs to send</param>
        public FormUrlEncodedContent(IEnumerable<KeyValuePair<string, string>> nameValueCollection)
        {
            _content = SerialiseContent(nameValueCollection);

            Headers = new Dictionary<string, string>()
            {
                { "Content-Type", "application/x-www-form-urlencoded" }
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

        private byte[] SerialiseContent(IEnumerable<KeyValuePair<string, string>> nameValueCollection)
        {
            StringBuilder stringBuilder = new StringBuilder();

            foreach (KeyValuePair<string, string> nameValue in nameValueCollection)
            {
                UrlEncoded(stringBuilder, nameValue.Key, nameValue.Value);
            }

            return Encoding.ASCII.GetBytes(stringBuilder.ToString());
        }

        private void UrlEncoded(StringBuilder sb, string name, string value)
        {
            if (sb.Length != 0)
                sb.Append("&");
            sb.Append(Uri.EscapeUriString(name));
            sb.Append("=");
            sb.Append(Uri.EscapeUriString(value));
        }
    }
}