using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using System.Text.RegularExpressions;

namespace CI.HttpClient.Core
{
    public abstract class HttpBase
    {
        protected HttpWebRequest _request;
        protected HttpWebResponse _response;
        protected IDispatcher _dispatcher;

        private bool _isAborted;

        protected void SetMethod(HttpAction httpAction)
        {
            _request.Method = httpAction.ToString().ToUpper();
        }

#if NETFX_CORE
        protected void SetContentHeaders(IHttpContent content)
        {
            _request.Headers[HttpRequestHeader.ContentLength] = content.GetContentLength().ToString();
            _request.ContentType = content.GetContentType();
        }
#else
        protected void SetContentHeaders(IHttpContent content)
        {
            _request.ContentLength = content.GetContentLength();
            _request.ContentType = content.GetContentType();
        }
#endif

#if NETFX_CORE
        protected void HandleRequestWrite(IHttpContent content, Action<UploadStatusMessage> uploadStatusCallback, int blockSize)
        {
            using (Stream stream = _request.GetRequestStreamAsync().Result)
            {
                if (content.ContentReadAction == ContentReadAction.Multipart)
                {
                    WriteMultipleContent(stream, content, uploadStatusCallback, blockSize);
                }
                else
                {
                    WriteSingleContent(stream, content, uploadStatusCallback, blockSize, content.GetContentLength(), 0);
                }
            }
        }
#else
        protected void HandleRequestWrite(IHttpContent content, Action<UploadStatusMessage> uploadStatusCallback, int blockSize)
        {
            using (Stream stream = _request.GetRequestStream())
            {
                if (content.ContentReadAction == ContentReadAction.Multipart)
                {
                    WriteMultipleContent(stream, content, uploadStatusCallback, blockSize);
                }
                else
                {
                    WriteSingleContent(stream, content, uploadStatusCallback, blockSize, content.GetContentLength(), 0);
                }
            }
        }
#endif

        private void WriteMultipleContent(Stream stream, IHttpContent content, Action<UploadStatusMessage> uploadStatusCallback, int blockSize)
        {
            long contentLength = content.GetContentLength();
            long totalContentUploaded = 0;
            MultipartContent multipartContent = content as MultipartContent;

            foreach (IHttpContent singleContent in multipartContent)
            {
                stream.Write(multipartContent.BoundaryStartBytes, 0, multipartContent.BoundaryStartBytes.Length);
                totalContentUploaded += multipartContent.BoundaryStartBytes.Length;

                foreach (var header in singleContent.Headers)
                {
                    byte[] headerBytes = Encoding.UTF8.GetBytes(header.Key + ": " + header.Value);

                    stream.Write(headerBytes, 0, headerBytes.Length);
                    totalContentUploaded += headerBytes.Length;

                    stream.Write(multipartContent.CRLFBytes, 0, multipartContent.CRLFBytes.Length);
                    totalContentUploaded += multipartContent.CRLFBytes.Length;
                }

                stream.Write(multipartContent.CRLFBytes, 0, multipartContent.CRLFBytes.Length);
                totalContentUploaded += multipartContent.CRLFBytes.Length;

                totalContentUploaded += WriteSingleContent(stream, singleContent, uploadStatusCallback, blockSize, contentLength, totalContentUploaded);

                stream.Write(multipartContent.CRLFBytes, 0, multipartContent.CRLFBytes.Length);
                totalContentUploaded += multipartContent.CRLFBytes.Length;
            }

            if (!multipartContent.Any())
            {
                stream.Write(multipartContent.BoundaryStartBytes, 0, multipartContent.BoundaryStartBytes.Length);
                totalContentUploaded += multipartContent.BoundaryStartBytes.Length;
            }

            stream.Write(multipartContent.BoundaryEndBytes, 0, multipartContent.BoundaryEndBytes.Length);
            totalContentUploaded += multipartContent.BoundaryEndBytes.Length;

            RaiseUploadStatusCallback(uploadStatusCallback, contentLength, (multipartContent.CRLFBytes.Length * 2) + multipartContent.BoundaryEndBytes.Length, totalContentUploaded);
        }

        private long WriteSingleContent(Stream stream, IHttpContent content, Action<UploadStatusMessage> uploadStatusCallback, int blockSize, long overallContentLength, long totalContentUploadedOverall)
        {
            long contentLength = content.GetContentLength();
            int contentUploadedThisRound = 0;
            int totalContentUploaded = 0;
            byte[] requestBuffer = new byte[blockSize];
            Stream contentStream = content.ReadAsStream();

            while (totalContentUploaded != contentLength)
            {
                contentUploadedThisRound = 0;

                int read = 0;
                while ((read = contentStream.Read(requestBuffer, read, blockSize - read)) > 0)
                {
                    contentUploadedThisRound += read;
                }

                if (contentUploadedThisRound > 0)
                {
                    stream.Write(requestBuffer, 0, contentUploadedThisRound);
                }

                totalContentUploaded += contentUploadedThisRound;
                totalContentUploadedOverall += contentUploadedThisRound;

                RaiseUploadStatusCallback(uploadStatusCallback, overallContentLength, contentUploadedThisRound, totalContentUploadedOverall);
            }

            return totalContentUploaded;
        }

#if NETFX_CORE
        protected void HandleResponseRead(Action<HttpResponseMessage> responseCallback, HttpCompletionOption completionOption, int blockSize)
        {
            try
            {
                _response = (HttpWebResponse)_request.GetResponseAsync().Result;
            }
            catch (AggregateException e)
            {
                if (e.InnerExceptions.Any() && e.InnerExceptions.First() is WebException)
                {
                    _response = (HttpWebResponse)(e.InnerExceptions.First() as WebException).Response;
                }
                else
                {
                    throw;
                }
            }

            if (_response == null)
            {
                throw new Exception("Server did not return a response");
            }

            using (Stream stream = _response.GetResponseStream())
            {
                if (responseCallback == null)
                {
                    return;
                }

                long totalContentRead = 0;
                int contentReadThisRound = 0;

                int readThisLoop = 0;
                List<byte> allContent = new List<byte>();
                byte[] buffer = new byte[blockSize];

                do
                {
                    readThisLoop = stream.Read(buffer, contentReadThisRound, blockSize - contentReadThisRound);

                    contentReadThisRound += readThisLoop;

                    if (contentReadThisRound == blockSize || readThisLoop == 0)
                    {
                        totalContentRead += contentReadThisRound;

                        byte[] responseData = new byte[contentReadThisRound];

                        Array.Copy(buffer, responseData, contentReadThisRound);

                        if (completionOption == HttpCompletionOption.AllResponseContent)
                        {
                            allContent.AddRange(responseData);
                        }

                        if (completionOption == HttpCompletionOption.StreamResponseContent || readThisLoop == 0)
                        {
                            RaiseResponseCallback(responseCallback, completionOption == HttpCompletionOption.AllResponseContent ? allContent.ToArray() : responseData,
                                completionOption == HttpCompletionOption.AllResponseContent ? totalContentRead : contentReadThisRound, totalContentRead);
                        }

                        contentReadThisRound = 0;
                    }
                } while (readThisLoop > 0);
            }
        }
#else
        protected void HandleResponseRead(Action<HttpResponseMessage> responseCallback, HttpCompletionOption completionOption, int blockSize)
        {
            try
            {
                _response = (HttpWebResponse)_request.GetResponse();
            }
            catch (WebException e)
            {
                _response = (HttpWebResponse)e.Response;
            }

            if (_response == null)
            {
                throw new Exception("Server did not return a response");
            }

            using (Stream stream = _response.GetResponseStream())
            {
                if (responseCallback == null)
                {
                    return;
                }

                long totalContentRead = 0;
                int contentReadThisRound = 0;

                int readThisLoop = 0;
                List<byte> allContent = new List<byte>();
                byte[] buffer = new byte[blockSize];

                do
                {
                    readThisLoop = stream.Read(buffer, contentReadThisRound, blockSize - contentReadThisRound);

                    contentReadThisRound += readThisLoop;

                    if (contentReadThisRound == blockSize || readThisLoop == 0)
                    {
                        totalContentRead += contentReadThisRound;

                        byte[] responseData = new byte[contentReadThisRound];
                        
                        Array.Copy(buffer, responseData, contentReadThisRound);

                        if (completionOption == HttpCompletionOption.AllResponseContent)
                        {
                            allContent.AddRange(responseData);
                        }

                        if (completionOption == HttpCompletionOption.StreamResponseContent || readThisLoop == 0)
                        {
                            RaiseResponseCallback(responseCallback, completionOption == HttpCompletionOption.AllResponseContent ? allContent.ToArray() : responseData,
                                completionOption == HttpCompletionOption.AllResponseContent ? totalContentRead : contentReadThisRound, totalContentRead);
                        }

                        contentReadThisRound = 0;
                    }
                } while (readThisLoop > 0);
            }
        }
#endif

        private void RaiseUploadStatusCallback(Action<UploadStatusMessage> uploadStatusCallback, long contentLength, long contentUploadedThisRound, long totalContentUploaded)
        {
            if (uploadStatusCallback != null)
            {
                _dispatcher.Enqueue(() =>
                {
                    uploadStatusCallback(new UploadStatusMessage()
                    {
                        ContentLength = contentLength,
                        ContentUploadedThisRound = contentUploadedThisRound,
                        TotalContentUploaded = totalContentUploaded
                    });
                });
            }
        }

        private void RaiseResponseCallback(Action<HttpResponseMessage> responseCallback, byte[] data, long contentReadThisRound, long totalContentRead)
        {
            _dispatcher.Enqueue(() =>
            {
                try
                {
                    responseCallback(new HttpResponseMessage(data)
                    {
                        OriginalRequest = _request,
                        OriginalResponse = _response,
                        ContentLength = _response.ContentLength,
                        ContentReadThisRound = contentReadThisRound,
                        TotalContentRead = totalContentRead,
                        StatusCode = _response.StatusCode,
                        ReasonPhrase = _response.StatusDescription
                    });
                }
                catch (ObjectDisposedException)
                {
                    RaiseAbortedResponse(responseCallback);
                }
            });
        }

        protected void RaiseErrorResponse(Action<HttpResponseMessage> action, Exception exception)
        {
            if (action != null)
            {
                _dispatcher.Enqueue(() =>
                {
                    try
                    {
                        action(new HttpResponseMessage()
                        {
                            OriginalRequest = _request,
                            OriginalResponse = _response,
                            Exception = exception,
                            StatusCode = GetStatusCode(exception, _response),
                            ReasonPhrase = GetReasonPhrase(exception, _response)
                        });
                    }
                    catch (ObjectDisposedException)
                    {
                        RaiseAbortedResponse(action);
                    }
                });
            }
        }

        private void RaiseAbortedResponse(Action<HttpResponseMessage> action)
        {
            if (_isAborted)
            {
                return;
            }

            action(new HttpResponseMessage()
            {
                OriginalRequest = null,
                OriginalResponse = null,
                Exception = new Exception("The request was aborted"),
                StatusCode = HttpStatusCode.InternalServerError,
                ReasonPhrase = "Unknown"
            });

            _isAborted = true;
        }

        private HttpStatusCode GetStatusCode(Exception exception, HttpWebResponse response)
        {
            if (response != null)
            {
                return response.StatusCode;
            }

            if (exception.Message.Contains("The remote server returned an error:"))
            {
                int statusCode = 0;

                Match match = Regex.Match(exception.Message, "\\(([0-9]+)\\)");

                if (match.Groups.Count == 2 && int.TryParse(match.Groups[1].Value, out statusCode))
                {
                    return (HttpStatusCode)statusCode;
                }
            }

            return HttpStatusCode.InternalServerError;
        }

        private string GetReasonPhrase(Exception exception, HttpWebResponse response)
        {
            if (response != null)
            {
                return response.StatusDescription;
            }

            if (exception.Message.Contains("The remote server returned an error:"))
            {
                Match match = Regex.Match(exception.Message, "\\([0-9]+\\) (.+)");

                if (match.Groups.Count == 2)
                {
                    return match.Groups[1].Value;
                }
            }

            return "Unknown";
        }
    }
}