// Fill out your copyright notice in the Description page of Project Settings.
#include "WeatherLib.h"
#include "Materials/MaterialParameterCollection.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "Blueprint/WidgetBlueprintLibrary.h"

AExponentialHeightFog* UWeatherLib::weather_fog_ = nullptr;

UMaterialParameterCollectionInstance* UWeatherLib::getWeatherMaterialCollectionInstance(UWorld* World)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    if (World) {
        UMaterialParameterCollection* WeatherParameterCollection = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, getWeatherParamsObjectPath()));

        //UWorld* World = GetWorld();
        if (WeatherParameterCollection) {
            UMaterialParameterCollectionInstance* Instance = World->GetParameterCollectionInstance(WeatherParameterCollection);
            if (Instance) {
                return Instance;
            }
            else {
                UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollectionInstance1!"));
            }
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollection1!"));
        }
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get World!"));
    }

    return NULL;
}
void UWeatherLib::initWeather(UWorld* World, TArray<AActor*> ActorsToAttachTo)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    if (World) {
        UClass* WeatherActorClass = getWeatherActorPath().TryLoadClass<AActor>();
        if (WeatherActorClass) {
            for (int32 i = 0; i < ActorsToAttachTo.Num(); i++) {
                const FVector Location = ActorsToAttachTo[i]->GetActorLocation();
                const FRotator Rotation = ActorsToAttachTo[i]->GetActorRotation();
                FActorSpawnParameters WeatherActorSpawnInfo;
                WeatherActorSpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                AActor* SpawnedWeatherActor = World->SpawnActor(WeatherActorClass, &Location, &Rotation, WeatherActorSpawnInfo);

                SpawnedWeatherActor->AttachToActor(ActorsToAttachTo[i], FAttachmentTransformRules(EAttachmentRule::SnapToTarget, true));
            }
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid weather actor class!"));
        }
        // still need the menu class for f10
        UClass* MenuActorClass = getWeatherMenuObjectPath().TryLoadClass<AActor>();
        if (MenuActorClass) {
            //UClass* Class, FTransform const* Transform, const FActorSpawnParameters& SpawnParameters = FActorSpawnParameters()
            const FVector Location = FVector(0, 0, 0);
            const FRotator Rotation = FRotator(0.0f, 0.0f, 0.0f);
            FActorSpawnParameters SpawnInfo;
            SpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            World->SpawnActor(MenuActorClass, &Location, &Rotation, SpawnInfo);
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid menu actor class!"));
        }
    }

    //showWeatherMenu(WorldContextObject);
}
void UWeatherLib::setWeatherParamScalar(UWorld* World, EWeatherParamScalar Param, float Amount)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamScalarName(Param);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        WeatherMaterialCollectionInstance->SetScalarParameterValue(ParamName, Amount);

        // if weather is not enabled, dont allow any weather values to be set
        // must be called after SetScalarParam, because WeatherEnabled is a scalar param
        // and must be set to true or false before this.
        // WeatherEnabled will always be false
        // NOTE: weather enabled must be set first, before other params for this to work
        if (!getIsWeatherEnabled(World)) {
            WeatherMaterialCollectionInstance->SetScalarParameterValue(ParamName, 0.0f);
        }

        if (weather_fog_) {
            weather_fog_->GetRootComponent()->SetVisibility(true);
        }
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
}
float UWeatherLib::getWeatherParamScalar(UWorld* World, EWeatherParamScalar Param)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamScalarName(Param);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        float Amount;
        WeatherMaterialCollectionInstance->GetScalarParameterValue(ParamName, Amount); //SetScalarParameterValue(ParamName, Amount);

        return Amount;
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
    return 0.0f;
}
FVector UWeatherLib::getWeatherWindDirection(UWorld* World)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamVectorName(EWeatherParamVector::WEATHER_PARAM_VECTOR_WIND);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        FLinearColor Direction;
        WeatherMaterialCollectionInstance->GetVectorParameterValue(ParamName, Direction); //SetScalarParameterValue(ParamName, Amount);

        return FVector(Direction);
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
    return FVector(0, 0, 0);
}
void UWeatherLib::setWeatherWindDirection(UWorld* World, FVector NewWind)
{
    UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
    if (WeatherMaterialCollectionInstance) {
        FName ParamName = GetWeatherParamVectorName(EWeatherParamVector::WEATHER_PARAM_VECTOR_WIND);
        if (ParamName == TEXT("")) {
            UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
        }
        WeatherMaterialCollectionInstance->SetVectorParameterValue(ParamName, NewWind);
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
    }
}
bool UWeatherLib::getIsWeatherEnabled(UWorld* World)
{
    if (getWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_WEATHERENABLED) == 1.0f) {
        return true;
    }
    return false;
}
void UWeatherLib::setWeatherEnabled(UWorld* World, bool bEnabled)
{
    float Value = 0;
    if (bEnabled) {
        Value = 1;
    }
    setWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_WEATHERENABLED, Value);
}
void UWeatherLib::showWeatherMenu(UWorld* World)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);

    if (UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>()) {
        UUserWidget* MenuWidget = CreateWidget<UUserWidget>(World, MenuWidgetClass);

        if (MenuWidget) {
            MenuWidget->AddToViewport();
        }

        APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
        if (PC) {
            PC->bShowMouseCursor = true;
            PC->DisableInput(PC);
        }
    }
    else {
        UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could not load weather widget!"));
    }
}
void UWeatherLib::hideWeatherMenu(UWorld* World)
{

    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>();

    if (World && MenuWidgetClass) {
        // get all menu actors, if any
        TArray<UUserWidget*> FoundWidgets;
        UWidgetBlueprintLibrary::GetAllWidgetsOfClass(World, FoundWidgets, UUserWidget::StaticClass());

        UE_LOG(LogTemp, Warning, TEXT("%s Warning, WeatherAPI"), *MenuWidgetClass->GetClass()->GetFName().ToString());

        if (FoundWidgets.Num() > 0) {
            for (int32 i = 0; i < FoundWidgets.Num(); i++) {
                // hacky test to make sure we are getting the right class. for some reason cast above doesn't work, so we use this instead to test for class
                if (FoundWidgets[i] && FoundWidgets[i]->GetClass()->GetFName().ToString() == getWeatherMenuClassName()) {
                    FoundWidgets[i]->RemoveFromParent();
                    FoundWidgets[i]->RemoveFromViewport();
                }
            }
            APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
            if (PC) {
                PC->bShowMouseCursor = false;
                PC->EnableInput(PC);
            }
        }
    }
}
bool UWeatherLib::isMenuVisible(UWorld* World)
{
    //UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
    UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>();

    if (World && MenuWidgetClass) {
        // get all menu actors, if any
        TArray<UUserWidget*> FoundWidgets;
        UWidgetBlueprintLibrary::GetAllWidgetsOfClass(World, FoundWidgets, UUserWidget::StaticClass());

        UE_LOG(LogTemp, Warning, TEXT("%s Warning, WeatherAPI"), *MenuWidgetClass->GetClass()->GetFName().ToString());

        if (FoundWidgets.Num() > 0) {
            for (int32 i = 0; i < FoundWidgets.Num(); i++) {
                // hacky test to make sure we are getting the right class. for some reason cast above doesn't work, so we use this instead to test for class
                if (FoundWidgets[i] && FoundWidgets[i]->GetClass()->GetFName().ToString() == getWeatherMenuClassName()) {
                    return true;
                }
            }
        }
    }

    // get all menu actors, if any, then hide the menu
    return false;
}
void UWeatherLib::toggleWeatherMenu(UWorld* World)
{
    if (isMenuVisible(World)) {
        hideWeatherMenu(World);
    }
    else {
        showWeatherMenu(World);
    }
}
UWorld* UWeatherLib::widgetGetWorld(UUserWidget* Widget)
{
    if (Widget) {
        return Widget->GetWorld();
    }
    return NULL;
}
UWorld* UWeatherLib::actorGetWorld(AActor* Actor)
{
    if (Actor) {
        return Actor->GetWorld();
    }
    return NULL;
}
void UWeatherLib::setWeatherFog(AExponentialHeightFog* fog)
{
    weather_fog_ = fog;
}

FText UWeatherLib::GetKeyboardShortcutsText()
{
    FString KeyboardShortcutsString;

    KeyboardShortcutsString += TEXT("Welcome to AirSim!");
    KeyboardShortcutsString += TEXT("\n\n");

    KeyboardShortcutsString += TEXT("HUD Toggles:\n");
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleHelp"), TEXT("Show Help Text"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleWeather"), TEXT("Show Weather Options"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventToggleRecording"), TEXT("Recording"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleReport"), TEXT("Debug Report"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleTrace"), TEXT("Trace Line"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleAll"), TEXT("All Sub-Windows"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleSubwindow0"), TEXT("Depth Window"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleSubwindow1"), TEXT("Segmentation Window"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventToggleSubwindow2"), TEXT("Scene Window"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("InputEventResetAll"), TEXT("Reset Vehicles"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += TEXT("\n");

    KeyboardShortcutsString += TEXT("Camera Controls:\n");
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventFpvView"), TEXT("FPV View"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventFlyWithView"), TEXT("\"Fly With Me\" View"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventGroundView"), TEXT("Ground Observer View"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventManualView"), TEXT("Manual Camera View"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventSpringArmChaseView"), TEXT("Spring Arm View"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventFrontView"), TEXT("Front View"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventBackupView"), TEXT("Backup View"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputEventNoDisplayView"), TEXT("No Display"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += TEXT("\n");

    KeyboardShortcutsString += TEXT("Manual Camera Controls:\n");
    KeyboardShortcutsString += MakeInputString(TEXT("inputManualForward"), TEXT("Fly Forward & Back"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputManualArrowRight"), TEXT("Fly Left & Right"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputManualArrowUp"), TEXT("Fly Up & Down"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputManualRightYaw"), TEXT("Yaw"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputManualUpPitch"), TEXT("Pitch"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("inputManualRightRoll"), TEXT("Roll"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += TEXT("\n");

    KeyboardShortcutsString += TEXT("Car Controls:\n");
    KeyboardShortcutsString += MakeInputString(TEXT("MoveForward"), TEXT("Drive & Reverse"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("MoveRight"), TEXT("Turn Right & Left"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("Footbrake"), TEXT("Footbrake"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += MakeInputString(TEXT("Handbrake"), TEXT("Handbrake"), EInputTypes::EKeyboard);
    KeyboardShortcutsString += TEXT("\n");

    return FText::FromString(KeyboardShortcutsString);
}

FString UWeatherLib::MakeInputString(const FName InputName, const FString Descriptor, EInputTypes InputTypes, int WhitespacePadding)
{
    const FString InputKeys = GetInputMapKeysString(InputName, InputTypes);
    FString WhitespaceString;

	WhitespaceString = WhitespaceString.RightPad(WhitespacePadding);
    WhitespaceString = WhitespaceString.LeftChop(InputKeys.Len());

	return  InputKeys + WhitespaceString + Descriptor + TEXT("\n");
}
FString UWeatherLib::GetInputMapKeysString(FName InputName, EInputTypes InputTypes)
{
    const UInputSettings* InputSettings = UInputSettings::GetInputSettings();

    TArray<FInputActionKeyMapping> ActionKeyMapping;
    InputSettings->GetActionMappingByName(InputName, ActionKeyMapping);

    TArray<FInputAxisKeyMapping> AxisKeyMapping;
    InputSettings->GetAxisMappingByName(InputName, AxisKeyMapping);

    FString JoinedStr = FString(TEXT(""));

    // Remove Keys that don't match type InputType
	for(int i = ActionKeyMapping.Num()-1; i >= 0; i--)
	{
		if (InputTypes == EInputTypes::EKeyboard && ActionKeyMapping[i].Key.IsGamepadKey())
		{
			ActionKeyMapping.RemoveAt(i);
		}
		if (InputTypes == EInputTypes::EGamepad && !ActionKeyMapping[i].Key.IsGamepadKey())
		{
			ActionKeyMapping.RemoveAt(i);
		}
	}
	for (int i = AxisKeyMapping.Num()-1; i >= 0; i--)
	{
		if (InputTypes == EInputTypes::EKeyboard && AxisKeyMapping[i].Key.IsGamepadKey())
		{
			AxisKeyMapping.RemoveAt(i);
		}
		if (InputTypes == EInputTypes::EGamepad && !AxisKeyMapping[i].Key.IsGamepadKey())
		{
			AxisKeyMapping.RemoveAt(i);
		}
	}

    // Get Action Map Keys
    JoinedStr += GetInputActionMapString(ActionKeyMapping);
    JoinedStr += GetInputAxisMapString(AxisKeyMapping);

    // Shorten Gamepad Terms
    JoinedStr = JoinedStr.Replace(TEXT("Gamepad "), TEXT(""));
    JoinedStr = JoinedStr.Replace(TEXT("Right "), TEXT("R-"));
    JoinedStr = JoinedStr.Replace(TEXT("Left "), TEXT("L-"));
    JoinedStr = JoinedStr.Replace(TEXT("Thumbstick"), TEXT("Stick"));
    JoinedStr = JoinedStr.Replace(TEXT(" Axis"), TEXT(""));

    return JoinedStr;
}

FString UWeatherLib::GetInputActionMapString(TArray<FInputActionKeyMapping> ActionKeyMapping, bool bReverseOrder)
{
    FString JoinedStr;

    if (bReverseOrder)
    {
        for (int Index = 0; Index < ActionKeyMapping.Num(); Index++)
        {
            JoinedStr += ActionKeyMapping[Index].Key.GetDisplayName(false).ToString();
            if (Index + 1 < ActionKeyMapping.Num())
            {
                JoinedStr += TEXT(", ");
            }
        }
    }
    else
    {
	    for (int Index = ActionKeyMapping.Num() - 1; Index >= 0; Index--)
	    {
	    	JoinedStr += ActionKeyMapping[Index].Key.GetDisplayName(false).ToString();
	    	if (Index - 1 >= 0)
	    	{
	    		JoinedStr += TEXT(", ");
	    	}
	    }
    }
    return JoinedStr;
}

FString UWeatherLib::GetInputAxisMapString(TArray<FInputAxisKeyMapping> AxisKeyMapping, bool bReverseOrder)
{
    FString JoinedStr;
    if(bReverseOrder)
    {
        for (int Index = 0; Index < AxisKeyMapping.Num(); Index++)
        {
            JoinedStr += AxisKeyMapping[Index].Key.GetDisplayName(false).ToString();
            if (Index + 1 < AxisKeyMapping.Num())
            {
                JoinedStr += TEXT(", ");
            }
        }
    }
    else{
		for (int Index = AxisKeyMapping.Num() - 1; Index >= 0; Index--)
		{
			JoinedStr += AxisKeyMapping[Index].Key.GetDisplayName(false).ToString();
			if (Index - 1 >= 0)
			{
				JoinedStr += TEXT(", ");
			}
		}
	}
    return JoinedStr;
}
