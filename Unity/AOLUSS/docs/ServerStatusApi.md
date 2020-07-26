# IO.Swagger.Api.ServerStatusApi

All URIs are relative to *http://localhost:8401/operator*

Method | HTTP request | Description
------------- | ------------- | -------------
[**GetOperatorStatus**](ServerStatusApi.md#getoperatorstatus) | **GET** /status | Server status

<a name="getoperatorstatus"></a>
# **GetOperatorStatus**
> void GetOperatorStatus ()

Server status

### Example
```csharp
using System;
using System.Diagnostics;
using IO.Swagger.Api;
using IO.Swagger.Client;
using IO.Swagger.Model;

namespace Example
{
    public class GetOperatorStatusExample
    {
        public void main()
        {
            var apiInstance = new ServerStatusApi();

            try
            {
                // Server status
                apiInstance.GetOperatorStatus();
            }
            catch (Exception e)
            {
                Debug.Print("Exception when calling ServerStatusApi.GetOperatorStatus: " + e.Message );
            }
        }
    }
}
```

### Parameters
This endpoint does not need any parameter.

### Return type

void (empty response body)

### Authorization

No authorization required

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: Not defined

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)
