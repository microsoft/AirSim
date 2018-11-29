// Fill out your copyright notice in the Description page of Project Settings.
#include "WeatherLib.h"
#include "Materials/MaterialParameterCollection.h"
#include "Runtime/Engine/Classes/Kismet/GameplayStatics.h"
#include "Blueprint/UserWidget.h"
#include "Blueprint/WidgetBlueprintLibrary.h"


UMaterialParameterCollectionInstance* UWeatherLib::getWeatherMaterialCollectionInstance(UWorld* World)
{
	//UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (World)
	{
		UMaterialParameterCollection* WeatherParameterCollection = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, getWeatherParamsObjectPath()));

		//UWorld* World = GetWorld();
		if (WeatherParameterCollection)
		{
			UMaterialParameterCollectionInstance* Instance = World->GetParameterCollectionInstance(WeatherParameterCollection);
			if (Instance)
			{
				return Instance;
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollectionInstance1!"));
			}
			
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get WeatherParameterCollection1!"));
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get World!"));
	}

	return NULL;
}
void UWeatherLib::initWeather(UWorld* World, TArray<AActor*> ActorsToAttachTo)
{
	//UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (World)
	{
		UClass* WeatherActorClass = getWeatherActorPath().TryLoadClass<AActor>();
		if (WeatherActorClass)
		{
			for (int32 i = 0; i < ActorsToAttachTo.Num(); i++)
			{
				const FVector Location = ActorsToAttachTo[i]->GetActorLocation();
				const FRotator Rotation = ActorsToAttachTo[i]->GetActorRotation();
				FActorSpawnParameters WeatherActorSpawnInfo;
				WeatherActorSpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
				AActor* SpawnedWeatherActor = World->SpawnActor(WeatherActorClass, &Location, &Rotation, WeatherActorSpawnInfo);

				SpawnedWeatherActor->AttachToActor(ActorsToAttachTo[i], FAttachmentTransformRules(EAttachmentRule::SnapToTarget, true));

			}
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid weather actor class!"));
		}
		// still need the menu class for f10 
		UClass* MenuActorClass = getWeatherMenuObjectPath().TryLoadClass<AActor>();
		if (MenuActorClass)
		{
			//UClass* Class, FTransform const* Transform, const FActorSpawnParameters& SpawnParameters = FActorSpawnParameters()
			const FVector Location = FVector(0, 0, 0);
			const FRotator Rotation = FRotator(0.0f, 0.0f, 0.0f);
			FActorSpawnParameters SpawnInfo;
			SpawnInfo.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			World->SpawnActor(MenuActorClass, &Location, &Rotation, SpawnInfo);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid menu actor class!"));
		}

	}

	//showWeatherMenu(WorldContextObject);

}
void UWeatherLib::setWeatherParamScalar(UWorld* World, EWeatherParamScalar Param, float Amount)
{
	UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
	if (WeatherMaterialCollectionInstance)
	{
		FName ParamName = GetWeatherParamScalarName(Param);
		if (ParamName == TEXT(""))
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
		}
		WeatherMaterialCollectionInstance->SetScalarParameterValue(ParamName, Amount);

		// if weather is not enabled, dont allow any weather values to be set
		// must be called after SetScalarParam, because WeatherEnabled is a scalar param
		// and must be set to true or false before this.
		// WeatherEnabled will always be false
		// NOTE: weather enabled must be set first, before other params for this to work
		if (!getIsWeatherEnabled(World))
		{
			WeatherMaterialCollectionInstance->SetScalarParameterValue(ParamName, 0.0f);
		}

	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
	}
}
float UWeatherLib::getWeatherParamScalar(UWorld* World, EWeatherParamScalar Param)
{
	UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
	if (WeatherMaterialCollectionInstance)
	{
		FName ParamName = GetWeatherParamScalarName(Param);
		if (ParamName == TEXT(""))
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
		}
		float Amount;
		WeatherMaterialCollectionInstance->GetScalarParameterValue(ParamName, Amount);//SetScalarParameterValue(ParamName, Amount);

		return Amount;
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
	}
	return 0.0f;
}
FVector UWeatherLib::getWeatherWindDirection(UWorld* World)
{
	UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
	if (WeatherMaterialCollectionInstance)
	{
		FName ParamName = GetWeatherParamVectorName(EWeatherParamVector::WEATHER_PARAM_VECTOR_WIND);
		if (ParamName == TEXT(""))
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
		}
		FLinearColor Direction;
		WeatherMaterialCollectionInstance->GetVectorParameterValue(ParamName, Direction);//SetScalarParameterValue(ParamName, Amount);

		return FVector(Direction);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
	}
	return FVector(0,0,0);
}
void UWeatherLib::setWeatherWindDirection(UWorld* World, FVector NewWind)
{
	UMaterialParameterCollectionInstance* WeatherMaterialCollectionInstance = UWeatherLib::getWeatherMaterialCollectionInstance(World);
	if (WeatherMaterialCollectionInstance)
	{
		FName ParamName = GetWeatherParamVectorName(EWeatherParamVector::WEATHER_PARAM_VECTOR_WIND);
		if (ParamName == TEXT(""))
		{
			UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI got invalid paramname!"));
		}
		WeatherMaterialCollectionInstance->SetVectorParameterValue(ParamName, NewWind);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could NOT get MaterialCollectionInstance!"));
	}
}
bool UWeatherLib::getIsWeatherEnabled(UWorld* World)
{
	if (getWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_WEATHERENABLED) == 1.0f)
	{
		return true;
	}
	return false;
}
void UWeatherLib::setWeatherEnabled(UWorld* World, bool bEnabled)
{
	float Value = 0;
	if (bEnabled)
	{
		Value = 1;
	}
	setWeatherParamScalar(World, EWeatherParamScalar::WEATHER_PARAM_SCALAR_WEATHERENABLED, Value);
}
void UWeatherLib::showWeatherMenu(UWorld* World)
{
	//UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);

	if (UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>())
	{
		UUserWidget* MenuWidget = CreateWidget<UUserWidget>(World, MenuWidgetClass);

		if (MenuWidget)
		{
			MenuWidget->AddToViewport();
		}

		APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
		if (PC)
		{
			PC->bShowMouseCursor = true;
			PC->DisableInput(PC);
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Warning, WeatherAPI could not load weather widget!"));
	}
}
void UWeatherLib::hideWeatherMenu(UWorld* World)
{

	//UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	UClass* MenuWidgetClass = getWeatherMenuWidgetClass().TryLoadClass<UUserWidget>();

	if (World && MenuWidgetClass)
	{
		// get all menu actors, if any
		TArray<UUserWidget*> FoundWidgets;
		UWidgetBlueprintLibrary::GetAllWidgetsOfClass(World, FoundWidgets, UUserWidget::StaticClass());

		UE_LOG(LogTemp, Warning, TEXT("%s Warning, WeatherAPI"), *MenuWidgetClass->GetClass()->GetFName().ToString());

		if (FoundWidgets.Num() > 0)
		{
			for (int32 i = 0; i < FoundWidgets.Num(); i++)
			{
				// hacky test to make sure we are getting the right class. for some reason cast above doesn't work, so we use this instead to test for class
				if (FoundWidgets[i] && FoundWidgets[i]->GetClass()->GetFName().ToString() == getWeatherMenuClassName())
				{
					FoundWidgets[i]->RemoveFromParent();
					FoundWidgets[i]->RemoveFromViewport();
				}

			}
			APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
			if (PC)
			{
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

	if (World && MenuWidgetClass)
	{
		// get all menu actors, if any
		TArray<UUserWidget*> FoundWidgets;
		UWidgetBlueprintLibrary::GetAllWidgetsOfClass(World, FoundWidgets, UUserWidget::StaticClass());

		UE_LOG(LogTemp, Warning, TEXT("%s Warning, WeatherAPI"), *MenuWidgetClass->GetClass()->GetFName().ToString());

		if (FoundWidgets.Num() > 0)
		{
			for (int32 i = 0; i < FoundWidgets.Num(); i++)
			{
				// hacky test to make sure we are getting the right class. for some reason cast above doesn't work, so we use this instead to test for class
				if (FoundWidgets[i] && FoundWidgets[i]->GetClass()->GetFName().ToString() == getWeatherMenuClassName())
				{
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
	if (isMenuVisible(World))
	{
		hideWeatherMenu(World);
	}
	else
	{
		showWeatherMenu(World);
	}
}
UWorld* UWeatherLib::widgetGetWorld(UUserWidget* Widget)
{
	if (Widget)
	{
		return Widget->GetWorld();
	}
	return NULL;
}
UWorld* UWeatherLib::actorGetWorld(AActor* Actor)
{
	if (Actor)
	{
		return Actor->GetWorld();
	}
	return NULL;
}