using System;
using System.Collections.Generic;
using System.Net;
using CI.HttpClient.Core;
using UnityEngine;

#if NETFX_CORE
using Windows.System.Threading;
#else
using System.Net.Cache;
using System.Security.Cryptography.X509Certificates;
using System.Threading;
#endif

namespace CI.HttpClient
{
    public class HttpClient
    {
        private const string DISPATCHER_GAMEOBJECT_NAME = "HttpClientDispatcher";
        private const int DEFAULT_BLOCK_SIZE = 10000;
        private const int DEFAULT_TIMEOUT = 100000;
        private const int DEFAULT_READ_WRITE_TIMEOUT = 300000;
        private const bool DEFAULT_KEEP_ALIVE = true;

        /// <summary>
        /// Chunk size when downloading data. Default is 10,000 bytes (10 kilobytes)
        /// </summary>
        public int DownloadBlockSize { get; set; }

        /// <summary>
        /// Chunk size when uploading data. Default is 10,000 bytes (10 kilobytes)
        /// </summary>
        public int UploadBlockSize { get; set; }

        /// <summary>
        /// Timeout value in milliseconds for opening read / write streams to the server. The default value is 100,000 milliseconds (100 seconds). Set by the system for Windows Store
        /// </summary>
        public int Timeout { get; set; }

        /// <summary>
        /// Timeout value in milliseconds when reading or writing data to / from the server. The default value is 300,000 milliseconds (5 minutes). Set by the system for Windows Store
        /// </summary>
        public int ReadWriteTimeout { get; set; }

#if !NETFX_CORE
        /// <summary>
        /// The cache policy that will be associated with requests. Not available for Windows Store
        /// </summary>
        public RequestCachePolicy Cache { get; set; }

        /// <summary>
        /// The collection of security certificates that will be associated with requests. Not available for Windows Store
        /// </summary>
        public X509CertificateCollection Certificates { get; set; }
#endif

        /// <summary>
        /// Cookies that will be associated with requests
        /// </summary>
        public CookieContainer Cookies { get; set; }

        /// <summary>
        /// Authentication information that will be associated with requests
        /// </summary>
        public ICredentials Credentials { get; set; }

        /// <summary>
        /// Indicates whether to make a persistent connection to the Internet resource. The default is true. Set by the system for Windows Store
        /// </summary>
        public bool KeepAlive { get; set; }

        /// <summary>
        /// Specifies a collection of the name/value pairs that make up the HTTP headers
        /// </summary>
        public IDictionary<string, string> Headers { get; set; }

        /// <summary>
        /// Proxy information that will be associated with requests
        /// </summary>
        public IWebProxy Proxy { get; set; }

        private readonly List<HttpWebRequest> _requests;
        private readonly object _lock;

        public static IDispatcher _dispatcher;

        /// <summary>
        /// Provides a class for sending HTTP requests and receiving HTTP responses from a resource identified by a URI
        /// </summary>
        public HttpClient()
        {
            DownloadBlockSize = DEFAULT_BLOCK_SIZE;
            UploadBlockSize = DEFAULT_BLOCK_SIZE;
            Timeout = DEFAULT_TIMEOUT;
            ReadWriteTimeout = DEFAULT_READ_WRITE_TIMEOUT;
            KeepAlive = DEFAULT_KEEP_ALIVE;
            Headers = new Dictionary<string, string>();
            _requests = new List<HttpWebRequest>();
            _lock = new object();

            CreateDispatcherGameObject();
        }

        /// <summary>
        /// Aborts all requests on this instance
        /// </summary>
        public void Abort()
        {
            lock (_lock)
            {
                foreach (HttpWebRequest request in _requests)
                {
                    request.Abort();
                }
            }
        }

        /// <summary>
        /// Sends a DELETE request to the specified Uri and returns the response body as a byte array. A completion option specifies if download progress should be reported
        /// </summary>
        /// <param name="uri">The Uri the request is sent to</param>
        /// <param name="completionOption">Determines how the response should be read</param>
        /// <param name="responseCallback">Callback raised once the request completes</param>
        public void Delete(Uri uri, HttpCompletionOption completionOption, Action<HttpResponseMessage> responseCallback)
        {
            QueueRequest(uri, HttpAction.Delete, completionOption, responseCallback);
        }

        /// <summary>
        /// Sends a Delete request to the specified Uri and returns the response body as a byte array. An uploadStatusCallback can be specified to report upload progress 
        /// and a completion option specifies if download progress should be reported
        /// </summary>
        /// <param name="uri">The Uri the request is sent to</param>
        /// <param name="content">Data to send</param>
        /// <param name="completionOption">Determines how the response should be read</param>
        /// <param name="responseCallback">Callback raised once the request completes</param>
        /// <param name="uploadStatusCallback">Callback that reports upload progress</param>
        public void Delete(Uri uri, IHttpContent content, HttpCompletionOption completionOption, Action<HttpResponseMessage> responseCallback,
            Action<UploadStatusMessage> uploadStatusCallback = null)
        {
            QueueRequest(uri, HttpAction.Delete, completionOption, content, responseCallback, uploadStatusCallback);
        }

        /// <summary>
        /// Sends a GET request to the specified Uri and returns the response body as a byte array. A completion option specifies if download progress should be reported
        /// </summary>
        /// <param name="uri">The Uri the request is sent to</param>
        /// <param name="completionOption">Determines how the response should be read</param>
        /// <param name="responseCallback">Callback raised once the request completes</param>
        public void Get(Uri uri, HttpCompletionOption completionOption, Action<HttpResponseMessage> responseCallback)
        {
            QueueRequest(uri, HttpAction.Get, completionOption, responseCallback);
        }

        /// <summary>
        /// Sends a PATCH request to the specified Uri and returns the response body as a byte array. An uploadStatusCallback can be specified to report upload progress 
        /// and a completion option specifies if download progress should be reported
        /// </summary>
        /// <param name="uri">The Uri the request is sent to</param>
        /// <param name="content">Data to send</param>
        /// <param name="completionOption">Determines how the response should be read</param>
        /// <param name="responseCallback">Callback raised once the request completes</param>
        /// <param name="uploadStatusCallback">Callback that reports upload progress</param>
        public void Patch(Uri uri, IHttpContent content, HttpCompletionOption completionOption, Action<HttpResponseMessage> responseCallback,
            Action<UploadStatusMessage> uploadStatusCallback = null)
        {
            QueueRequest(uri, HttpAction.Patch, completionOption, content, responseCallback, uploadStatusCallback);
        }

        /// <summary>
        /// Sends a POST request to the specified Uri and returns the response body as a byte array. An uploadStatusCallback can be specified to report upload progress 
        /// and a completion option specifies if download progress should be reported
        /// </summary>
        /// <param name="uri">The Uri the request is sent to</param>
        /// <param name="content">Data to send</param>
        /// <param name="completionOption">Determines how the response should be read</param>
        /// <param name="responseCallback">Callback raised once the request completes</param>
        /// <param name="uploadStatusCallback">Callback that reports upload progress</param>
        public void Post(Uri uri, IHttpContent content, HttpCompletionOption completionOption, Action<HttpResponseMessage> responseCallback,
            Action<UploadStatusMessage> uploadStatusCallback = null)
        {
            QueueRequest(uri, HttpAction.Post, completionOption, content, responseCallback, uploadStatusCallback);
        }

        /// <summary>
        /// Sends a PUT request to the specified Uri and returns the response body as a byte array. An uploadStatusCallback can be specified to report upload progress 
        /// and a completion option specifies if download progress should be reported
        /// </summary>
        /// <param name="uri">The Uri the request is sent to</param>
        /// <param name="content">Data to send</param>
        /// <param name="completionOption">Determines how the response should be read</param>
        /// <param name="responseCallback">Callback raised once the request completes</param>
        /// <param name="uploadStatusCallback">Callback that reports upload progress</param>
        public void Put(Uri uri, IHttpContent content, HttpCompletionOption completionOption, Action<HttpResponseMessage> responseCallback,
            Action<UploadStatusMessage> uploadStatusCallback = null)
        {
            QueueRequest(uri, HttpAction.Put, completionOption, content, responseCallback, uploadStatusCallback);
        }

        /// <summary>
        /// Allows changing of the dispatcher, currently only used for testing purposes
        /// </summary>
        /// <typeparam name="T">The type of dispatcher to create</typeparam>
        public void SetDispatcher<T>() where T : Component, IDispatcher
        {
            GameObject dispatcherGameObject = GameObject.Find(DISPATCHER_GAMEOBJECT_NAME);

            if (dispatcherGameObject != null)
            {
                foreach(var compontent in dispatcherGameObject.GetComponents(typeof(IDispatcher)))
                {
                    UnityEngine.Object.Destroy(compontent);
                }

                _dispatcher = dispatcherGameObject.AddComponent<T>();
            }
            else
            {
                _dispatcher = new GameObject(DISPATCHER_GAMEOBJECT_NAME).AddComponent<T>();
            }
        }

        private void QueueRequest(Uri uri, HttpAction httpAction, HttpCompletionOption httpCompletionOption, Action<HttpResponseMessage> responseCallback)
        {
            QueueWorkItem((t) =>
            {
                try
                {
                    HttpWebRequest request = CreateRequest(uri);
                    new HttpRequest(httpAction, request, _dispatcher).Execute(httpCompletionOption, responseCallback, DownloadBlockSize);
                    RemoveRequest(request);
                }
                catch (Exception e)
                {
                    RaiseErrorResponse(responseCallback, e);
                }
            });
        }

        private void QueueRequest(Uri uri, HttpAction httpAction, HttpCompletionOption httpCompletionOption, IHttpContent httpContent, 
            Action<HttpResponseMessage> responseCallback, Action<UploadStatusMessage> uploadStatusCallback)
        {
            QueueWorkItem((t) =>
            {
                try
                {
                    HttpWebRequest request = CreateRequest(uri);
                    new HttpRequest(httpAction, request, _dispatcher).Execute(httpContent, httpCompletionOption, responseCallback, uploadStatusCallback, DownloadBlockSize, UploadBlockSize);
                    RemoveRequest(request);
                }
                catch (Exception e)
                {
                    RaiseErrorResponse(responseCallback, e);
                }
            });
        }

#if NETFX_CORE
        private async void QueueWorkItem(WorkItemHandler action)
        {
            await ThreadPool.RunAsync(action);
        }
#else
        private void QueueWorkItem(WaitCallback action)
        {
            ThreadPool.QueueUserWorkItem(action);
        }
#endif

        private HttpWebRequest CreateRequest(Uri uri)
        {
            HttpWebRequest request = (HttpWebRequest)WebRequest.Create(uri);
            DisableWriteStreamBuffering(request);
            AddCache(request);
            AddCertificates(request);
            AddCookies(request);
            AddCredentials(request);
            AddKeepAlive(request);
            AddHeaders(request);
            AddProxy(request);
            AddTimeouts(request);
            AddRequest(request);
            return request;
        }

        private void DisableWriteStreamBuffering(HttpWebRequest request)
        {        
#if !NETFX_CORE
            request.AllowWriteStreamBuffering = false;
#endif
        }

        private void AddCache(HttpWebRequest request)
        {
#if !NETFX_CORE
            if (Cache != null)
            {
                request.CachePolicy = Cache;
            }
#endif
        }

        private void AddCertificates(HttpWebRequest request)
        {
#if !NETFX_CORE
            if (Certificates != null)
            {
                request.ClientCertificates = Certificates;
            }
#endif
        }

        private void AddCookies(HttpWebRequest request)
        {
            if (Cookies != null)
            {
                request.CookieContainer = Cookies;
            }
        }

        private void AddCredentials(HttpWebRequest request)
        {
            if (Credentials != null)
            {
                request.Credentials = Credentials;
            }
        }

        private void AddKeepAlive(HttpWebRequest request)
        {
#if !NETFX_CORE
            request.KeepAlive = KeepAlive;
#endif
        }

        private void AddHeaders(HttpWebRequest request)
        {
            if (Headers != null)
            {
                foreach (var header in Headers)
                {
#if NETFX_CORE
                    request.Headers[header.Key] = header.Value;
#else
                    switch (header.Key.ToLower())
                    {
                        case "accept":
                            request.Accept = header.Value;
                            break;
                        case "connection":
                            request.Connection = header.Value;
                            break;
                        case "content-length":
                            throw new NotSupportedException("Content Length is set automatically");
                        case "content-type":
                            throw new NotSupportedException("Content Type is set automatically");
                        case "expect":
                            request.Expect = header.Value;
                            break;
                        case "date":
                            throw new NotSupportedException("Date is automatically set by the system to the current date");
                        case "host":
                            throw new NotSupportedException("Host is automatically set by the system to current host information");
                        case "if-modified-since":
                            request.IfModifiedSince = DateTime.Parse(header.Value);
                            break;
                        case "range":
                            int range = int.Parse(header.Value);
                            request.AddRange(range);
                            break;                          
                        case "referer":
                            request.Referer = header.Value;
                            break;
                        case "transfer-encoding":
                            throw new NotSupportedException("Transfer Encoding is not currently supported");
                        case "user-agent":
                            request.UserAgent = header.Value;
                            break;
                        default:
                            request.Headers[header.Key] = header.Value;
                            break;
                    }
#endif
                }
            }
        }

        private void AddProxy(HttpWebRequest request)
        {
            if (Proxy != null)
            {
                request.Proxy = Proxy;
            }
        }

        private void AddTimeouts(HttpWebRequest request)
        {
#if !NETFX_CORE
            request.Timeout = Timeout;
            request.ReadWriteTimeout = ReadWriteTimeout;
#endif
        }

        private void AddRequest(HttpWebRequest request)
        {
            lock (_lock)
            {
                _requests.Add(request);
            }
        }

        private void RemoveRequest(HttpWebRequest request)
        {
            lock (_lock)
            {
                _requests.Remove(request);
            }
        }

        private void RaiseErrorResponse(Action<HttpResponseMessage> action, Exception exception)
        {
            if (action != null)
            {
                _dispatcher.Enqueue(() =>
                {
                    action(new HttpResponseMessage()
                    {
                        Exception = exception,
                    });
                });
            }
        }

        private void CreateDispatcherGameObject()
        {
            if (_dispatcher == null)
            {
                _dispatcher = new GameObject(DISPATCHER_GAMEOBJECT_NAME).AddComponent<Dispatcher>();
            }
        }
    }
}