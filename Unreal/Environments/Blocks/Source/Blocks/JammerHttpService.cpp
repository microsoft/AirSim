#include "JammerHttpService.h"
#include "JammerActor.h"

#include "HttpServerModule.h"
#include "HttpServerResponse.h"
#include "EngineUtils.h" // TActorIterator
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"

AJammerHttpService::AJammerHttpService()
{
    PrimaryActorTick.bCanEverTick = false;
}

void AJammerHttpService::BeginPlay()
{
    Super::BeginPlay();

    FHttpServerModule& HttpModule = FHttpServerModule::Get();
    Router = HttpModule.GetHttpRouter(Port);
    if (!Router.IsValid()) {
        UE_LOG(LogTemp, Error, TEXT("[HTTP] GetHttpRouter failed on port %d"), Port);
        return;
    }

    // /ping
    RoutePing = Router->BindRoute(
        FHttpPath(TEXT("/ping")),
        EHttpServerRequestVerbs::VERB_GET,
        [this](const FHttpServerRequest& Req, const FHttpResultCallback& Done) {
            return this->HandlePing(Req, Done);
        });

    // /jammers
    RouteJammers = Router->BindRoute(
        FHttpPath(TEXT("/jammers")),
        EHttpServerRequestVerbs::VERB_GET,
        [this](const FHttpServerRequest& Req, const FHttpResultCallback& Done) {
            return this->HandleGetJammers(Req, Done);
        });

    // /jammer_power?name=BP_Jammer_1&x=0&y=0&z=0   （也支持 POST JSON）
    RouteGetPower = Router->BindRoute(
        FHttpPath(TEXT("/jammer_power")),
        EHttpServerRequestVerbs::VERB_GET | EHttpServerRequestVerbs::VERB_POST,
        [this](const FHttpServerRequest& Req, const FHttpResultCallback& Done) {
            return this->HandleGetPower(Req, Done);
        });

    HttpModule.StartAllListeners();
    UE_LOG(LogTemp, Log, TEXT("[HTTP] Listening on port %d"), Port);
}

void AJammerHttpService::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (Router.IsValid()) {
        if (RoutePing.IsValid()) Router->UnbindRoute(RoutePing);
        if (RouteJammers.IsValid()) Router->UnbindRoute(RouteJammers);
        if (RouteGetPower.IsValid()) Router->UnbindRoute(RouteGetPower);
    }
    FHttpServerModule::Get().StopAllListeners();

    Super::EndPlay(EndPlayReason);
}

// ---------- Handlers ----------

bool AJammerHttpService::HandlePing(const FHttpServerRequest& Req, const FHttpResultCallback& Done)
{
    TSharedRef<FJsonObject> J = MakeShared<FJsonObject>();
    J->SetStringField(TEXT("status"), TEXT("ok"));
    SendJson(J, Done);
    return true;
}

bool AJammerHttpService::HandleGetJammers(const FHttpServerRequest& Req, const FHttpResultCallback& Done)
{
    UWorld* World = GetWorld();
    if (!World) {
        TSharedRef<FJsonObject> E = MakeShared<FJsonObject>();
        E->SetStringField(TEXT("error"), TEXT("no world"));
        SendJson(E, Done);
        return true;
    }

    TArray<TSharedPtr<FJsonValue>> Arr;
    for (TActorIterator<AJammerActor> It(World); It; ++It) {
        const AJammerActor* J = *It;
        TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
        O->SetStringField(TEXT("name"), J->GetName());
        O->SetStringField(TEXT("path"), J->GetPathName());
        O->SetBoolField(TEXT("isJamming"), J->IsJamming());
        O->SetNumberField(TEXT("basePower"), J->GetJammerPower());
        O->SetNumberField(TEXT("radiusCm"), J->GetRadiusCm());
        const FVector P = J->GetActorLocation();
        TSharedRef<FJsonObject> Pos = MakeShared<FJsonObject>();
        Pos->SetNumberField(TEXT("X"), P.X);
        Pos->SetNumberField(TEXT("Y"), P.Y);
        Pos->SetNumberField(TEXT("Z"), P.Z);
        O->SetObjectField(TEXT("location"), Pos);
        Arr.Add(MakeShared<FJsonValueObject>(O));
    }

    TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetArrayField(TEXT("jammers"), Arr);
    SendJson(Root, Done);
    return true;
}

static bool ParseVec3FromQuery(const TMap<FString, FString>& Q, FVector& Out, bool& bHas)
{
    bHas = false;
    if (const FString* Xs = Q.Find(TEXT("x"))) {
        Out.X = FCString::Atof(**Xs);
        bHas = true;
    }
    if (const FString* Ys = Q.Find(TEXT("y"))) {
        Out.Y = FCString::Atof(**Ys);
        bHas = true;
    }
    if (const FString* Zs = Q.Find(TEXT("z"))) {
        Out.Z = FCString::Atof(**Zs);
        bHas = true;
    }
    return bHas;
}

bool AJammerHttpService::HandleGetPower(const FHttpServerRequest& Req, const FHttpResultCallback& Done)
{
    FString Name;
    FVector Pos(0, 0, 0);
    bool bHasPos = false;

    if (Req.Verb == EHttpServerRequestVerbs::VERB_GET) {
        if (const FString* N = Req.QueryParams.Find(TEXT("name"))) Name = *N;
        ParseVec3FromQuery(Req.QueryParams, Pos, bHasPos);
    }
    else {
        // POST JSON: {"name":"BP_Jammer_1","x":1000,"y":0,"z":0}
        FString BodyStr = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Req.Body.GetData())));
        TSharedPtr<FJsonObject> J;
        auto Reader = TJsonReaderFactory<>::Create(BodyStr);
        if (FJsonSerializer::Deserialize(Reader, J) && J.IsValid()) {
            J->TryGetStringField(TEXT("name"), Name);
            double dx = 0, dy = 0, dz = 0;
            if (J->TryGetNumberField(TEXT("x"), dx)) {
                Pos.X = (float)dx;
                bHasPos = true;
            }
            if (J->TryGetNumberField(TEXT("y"), dy)) {
                Pos.Y = (float)dy;
                bHasPos = true;
            }
            if (J->TryGetNumberField(TEXT("z"), dz)) {
                Pos.Z = (float)dz;
                bHasPos = true;
            }
        }
    }

    AJammerActor* Target = nullptr;
    for (TActorIterator<AJammerActor> It(GetWorld()); It; ++It) {
        if (Name.IsEmpty() || It->GetName() == Name) {
            Target = *It;
            break;
        }
    }

    TSharedRef<FJsonObject> Out = MakeShared<FJsonObject>();
    if (!Target) {
        Out->SetStringField(TEXT("error"), TEXT("jammer not found"));
        if (!Name.IsEmpty()) Out->SetStringField(TEXT("name"), Name);
        SendJson(Out, Done);
        return true;
    }

    const float Power = bHasPos ? Target->GetJammerPowerAtLocation(Pos) : Target->GetJammerPower();
    Out->SetStringField(TEXT("name"), Target->GetName());
    Out->SetNumberField(TEXT("power"), Power);

    if (bHasPos) {
        TSharedRef<FJsonObject> P = MakeShared<FJsonObject>();
        P->SetNumberField(TEXT("X"), Pos.X);
        P->SetNumberField(TEXT("Y"), Pos.Y);
        P->SetNumberField(TEXT("Z"), Pos.Z);
        Out->SetObjectField(TEXT("queryLocation"), P);
    }

    SendJson(Out, Done);
    return true;
}

void AJammerHttpService::SendJson(const TSharedRef<FJsonObject>& Obj,
                                  const FHttpResultCallback& OnComplete)
{
    FString Body;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Body);
    FJsonSerializer::Serialize(Obj, Writer);

    // ★ 4.27 常见：Create(Body, ContentType) 只有 2 个参数
    TUniquePtr<FHttpServerResponse> Resp =
        FHttpServerResponse::Create(Body, TEXT("application/json; charset=utf-8"));

    // ★ 不同分发 Headers 的 Value 类型不同，二选一（能编过哪条用哪条）：
    // 1) 如果 Value 是 FString：
    // Resp->Headers.Add(TEXT("Access-Control-Allow-Origin"), FString(TEXT("*")));
    // 2) 如果 Value 是 TArray<FString>（你遇到的“无法从 wchar_t[2] 转换”通常是这种）：
    Resp->Headers.Add(TEXT("Access-Control-Allow-Origin"), TArray<FString>{ FString(TEXT("*")) });

    OnComplete(MoveTemp(Resp));
}

void AJammerHttpService::SendText(const FString& Text,
                                  const FHttpResultCallback& OnComplete)
{
    TUniquePtr<FHttpServerResponse> Resp =
        FHttpServerResponse::Create(Text, TEXT("text/plain; charset=utf-8"));

    // 同上，按你的分发选择其一：
    // Resp->Headers.Add(TEXT("Access-Control-Allow-Origin"), FString(TEXT("*")));
    Resp->Headers.Add(TEXT("Access-Control-Allow-Origin"), TArray<FString>{ FString(TEXT("*")) });

    OnComplete(MoveTemp(Resp));
}
