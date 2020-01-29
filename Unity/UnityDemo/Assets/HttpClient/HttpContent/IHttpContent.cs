using System.Collections.Generic;
using System.IO;

namespace CI.HttpClient
{
    public interface IHttpContent
    {
        ContentReadAction ContentReadAction { get; }
        IDictionary<string, string> Headers { get; }
        long GetContentLength();
        string GetContentType();
        Stream ReadAsStream();
    }
}