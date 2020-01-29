
namespace CI.HttpClient
{
    public class MultipartFormDataContent : MultipartContent
    {
        /// <summary>
        /// Send a combination of different HttpContents with a default boundary and the Content Type as multipart/form-data
        /// </summary>
        public MultipartFormDataContent()
            : base()
        {
        }

        /// <summary>
        /// Send a combination of different HttpContents with the specified boundary and the Content Type as multipart/form-data
        /// </summary>
        /// <param name="boundary"></param>
        public MultipartFormDataContent(string boundary)
            : base(boundary)
        {
        }

        /// <summary>
        /// Adds an IHttpContent to this multipart content - do not add other MultipartContents
        /// </summary>
        /// <param name="content">The IHttpContent</param>
        public override void Add(IHttpContent content)
        {
            content.Headers.Add("Content-Disposition", DEFAULT_SUBTYPE);

            base.Add(content);
        }

        /// <summary>
        /// Adds an IHttpContent to this multipart content - do not add other MultipartContents
        /// </summary>
        /// <param name="content">The IHttpContent</param>
        /// <param name="name">The name of the IHttpContent</param>
        public void Add(IHttpContent content, string name)
        {
            content.Headers.Add("Content-Disposition", string.Format("{0}; name=\"{1}\"", DEFAULT_SUBTYPE, name));

            base.Add(content);
        }

        /// <summary>
        /// Adds an IHttpContent to this multipart content - do not add other MultipartContents
        /// </summary>
        /// <param name="content">The IHttpContent</param>
        /// <param name="name">The name of the IHttpContent</param>
        /// <param name="filename">The filename for the IHttpContent</param>
        public void Add(IHttpContent content, string name, string filename)
        {
            content.Headers.Add("Content-Disposition", string.Format("{0}; name=\"{1}\"; filename=\"{2}\"", DEFAULT_SUBTYPE, name, filename));

            base.Add(content);
        }
    }
}