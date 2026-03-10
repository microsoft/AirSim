// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
// 这些必须在 .h 里包含，否则类型不完整（会触发你那串报错）
#include "HttpServerRequest.h" // FHttpServerRequest
#include "HttpResultCallback.h" // FHttpResultCallback
#include "HttpPath.h" // FHttpPath
#if __has_include("Interfaces/IHttpRouter.h")
    #include "Interfaces/IHttpRouter.h" // IHttpRouter, FHttpRouteHandle, EHttpServerRequestVerbs
#else
    #include "IHttpRouter.h"
#endif
#include "JammerHttpService.generated.h"

// 前置声明
class IHttpRouter;

/**
 * 
 */
UCLASS()
class BLOCKS_API AJammerHttpService : public AActor
{
    GENERATED_BODY()
public:
    AJammerHttpService();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UPROPERTY(EditAnywhere, Category = "HTTP")
    int32 Port = 18080;

private:
    TSharedPtr<IHttpRouter> Router;
    FHttpRouteHandle RoutePing;
    FHttpRouteHandle RouteJammers;
    FHttpRouteHandle RouteGetPower;

    bool HandlePing(const FHttpServerRequest& Req, const FHttpResultCallback& OnComplete);
    bool HandleGetJammers(const FHttpServerRequest& Req, const FHttpResultCallback& OnComplete);
    bool HandleGetPower(const FHttpServerRequest& Req, const FHttpResultCallback& OnComplete);

    // ★ 改为 2 个参数：不再传 ResponseCode
    static void SendJson(const TSharedRef<class FJsonObject>& Obj,
                         const FHttpResultCallback& OnComplete);
    static void SendText(const FString& Text,
                         const FHttpResultCallback& OnComplete);
};