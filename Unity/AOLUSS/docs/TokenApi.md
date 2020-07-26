# IO.Swagger.Api.TokenApi

All URIs are relative to *http://localhost:8401/operator*

Method | HTTP request | Description
------------- | ------------- | -------------
[**GetOperatorToken**](TokenApi.md#getoperatortoken) | **GET** /token | Get a JWT token for accessing the operator API.

<a name="getoperatortoken"></a>
# **GetOperatorToken**
> JsonWebToken GetOperatorToken ()

Get a JWT token for accessing the operator API.

### Example
```csharp
using System;
using System.Diagnostics;
using IO.Swagger.Api;
using IO.Swagger.Client;
using IO.Swagger.Model;

namespace Example
{
    public class GetOperatorTokenExample
    {
        public void main()
        {
            // Configure HTTP basic authorization: basicAuth
            Configuration.Default.Username = "YOUR_USERNAME";
            Configuration.Default.Password = "YOUR_PASSWORD";

            var apiInstance = new TokenApi();

            try
            {
                // Get a JWT token for accessing the operator API.
                JsonWebToken result = apiInstance.GetOperatorToken();
                Debug.WriteLine(result);
            }
            catch (Exception e)
            {
                Debug.Print("Exception when calling TokenApi.GetOperatorToken: " + e.Message );
            }
        }
    }
}
```

### Parameters
This endpoint does not need any parameter.

### Return type

[**JsonWebToken**](JsonWebToken.md)

### Authorization

[basicAuth](../README.md#basicAuth)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)
