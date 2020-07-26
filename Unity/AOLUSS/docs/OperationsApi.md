# IO.Swagger.Api.OperationsApi

All URIs are relative to *http://localhost:8401/operator*

Method | HTTP request | Description
------------- | ------------- | -------------
[**DeleteOperatorOperation**](OperationsApi.md#deleteoperatoroperation) | **GET** /delete | Delete all operation data
[**GetOperatorOperation**](OperationsApi.md#getoperatoroperation) | **GET** /operation | Get an operation
[**PostOperatorMessage**](OperationsApi.md#postoperatormessage) | **POST** /message | Post a message for an operation
[**PostOperatorPosition**](OperationsApi.md#postoperatorposition) | **POST** /position | Post a position for an operation
[**PutOperatorOperation**](OperationsApi.md#putoperatoroperation) | **POST** /operation | Post an operation

<a name="deleteoperatoroperation"></a>
# **DeleteOperatorOperation**
> ResponseOk DeleteOperatorOperation (Guid? gufi)

Delete all operation data

### Example
```csharp
using System;
using System.Diagnostics;
using IO.Swagger.Api;
using IO.Swagger.Client;
using IO.Swagger.Model;

namespace Example
{
    public class DeleteOperatorOperationExample
    {
        public void main()
        {

            var apiInstance = new OperationsApi();
            var gufi = new Guid?(); // Guid? | The GUFI of the operation

            try
            {
                // Delete all operation data
                ResponseOk result = apiInstance.DeleteOperatorOperation(gufi);
                Debug.WriteLine(result);
            }
            catch (Exception e)
            {
                Debug.Print("Exception when calling OperationsApi.DeleteOperatorOperation: " + e.Message );
            }
        }
    }
}
```

### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **gufi** | [**Guid?**](Guid?.md)| The GUFI of the operation | 

### Return type

[**ResponseOk**](ResponseOk.md)

### Authorization

[BearerToken](../README.md#BearerToken)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)
<a name="getoperatoroperation"></a>
# **GetOperatorOperation**
> void GetOperatorOperation (Guid? gufi)

Get an operation

### Example
```csharp
using System;
using System.Diagnostics;
using IO.Swagger.Api;
using IO.Swagger.Client;
using IO.Swagger.Model;

namespace Example
{
    public class GetOperatorOperationExample
    {
        public void main()
        {

            var apiInstance = new OperationsApi();
            var gufi = new Guid?(); // Guid? | The GUFI of the operation

            try
            {
                // Get an operation
                apiInstance.GetOperatorOperation(gufi);
            }
            catch (Exception e)
            {
                Debug.Print("Exception when calling OperationsApi.GetOperatorOperation: " + e.Message );
            }
        }
    }
}
```

### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **gufi** | [**Guid?**](Guid?.md)| The GUFI of the operation | 

### Return type

void (empty response body)

### Authorization

[BearerToken](../README.md#BearerToken)

### HTTP request headers

 - **Content-Type**: Not defined
 - **Accept**: application/json

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)
<a name="postoperatormessage"></a>
# **PostOperatorMessage**
> void PostOperatorMessage (ModelOperatorMessage body)

Post a message for an operation

### Example
```csharp
using System;
using System.Diagnostics;
using IO.Swagger.Api;
using IO.Swagger.Client;
using IO.Swagger.Model;

namespace Example
{
    public class PostOperatorMessageExample
    {
        public void main()
        {

            var apiInstance = new OperationsApi();
            var body = new ModelOperatorMessage(); // ModelOperatorMessage | 

            try
            {
                // Post a message for an operation
                apiInstance.PostOperatorMessage(body);
            }
            catch (Exception e)
            {
                Debug.Print("Exception when calling OperationsApi.PostOperatorMessage: " + e.Message );
            }
        }
    }
}
```

### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **body** | [**ModelOperatorMessage**](ModelOperatorMessage.md)|  | 

### Return type

void (empty response body)

### Authorization

[BearerToken](../README.md#BearerToken)

### HTTP request headers

 - **Content-Type**: application/json
 - **Accept**: application/json

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)
<a name="postoperatorposition"></a>
# **PostOperatorPosition**
> void PostOperatorPosition (ModelOperatorPosition body)

Post a position for an operation

### Example
```csharp
using System;
using System.Diagnostics;
using IO.Swagger.Api;
using IO.Swagger.Client;
using IO.Swagger.Model;

namespace Example
{
    public class PostOperatorPositionExample
    {
        public void main()
        {

            var apiInstance = new OperationsApi();
            var body = new ModelOperatorPosition(); // ModelOperatorPosition | 

            try
            {
                // Post a position for an operation
                apiInstance.PostOperatorPosition(body);
            }
            catch (Exception e)
            {
                Debug.Print("Exception when calling OperationsApi.PostOperatorPosition: " + e.Message );
            }
        }
    }
}
```

### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **body** | [**ModelOperatorPosition**](ModelOperatorPosition.md)|  | 

### Return type

void (empty response body)

### Authorization

[BearerToken](../README.md#BearerToken)

### HTTP request headers

 - **Content-Type**: application/json
 - **Accept**: application/json

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)
<a name="putoperatoroperation"></a>
# **PutOperatorOperation**
> void PutOperatorOperation (ModelOperatorOperation body)

Post an operation

### Example
```csharp
using System;
using System.Diagnostics;
using IO.Swagger.Api;
using IO.Swagger.Client;
using IO.Swagger.Model;

namespace Example
{
    public class PutOperatorOperationExample
    {
        public void main()
        {

            var apiInstance = new OperationsApi();
            var body = new ModelOperatorOperation(); // ModelOperatorOperation | 

            try
            {
                // Post an operation
                apiInstance.PutOperatorOperation(body);
            }
            catch (Exception e)
            {
                Debug.Print("Exception when calling OperationsApi.PutOperatorOperation: " + e.Message );
            }
        }
    }
}
```

### Parameters

Name | Type | Description  | Notes
------------- | ------------- | ------------- | -------------
 **body** | [**ModelOperatorOperation**](ModelOperatorOperation.md)|  | 

### Return type

void (empty response body)

### Authorization

[BearerToken](../README.md#BearerToken)

### HTTP request headers

 - **Content-Type**: application/json
 - **Accept**: application/json

[[Back to top]](#) [[Back to API list]](../README.md#documentation-for-api-endpoints) [[Back to Model list]](../README.md#documentation-for-models) [[Back to README]](../README.md)
