#pragma once

// dirent.h is a directory parsing header file. 
// The Unix version is natively included in linux,
// Windows compatible version must be downloaded
#ifdef _WIN32
	// Use local dirent.h header file
	#include "dirent.h"
#else
	// Use global header file
	#include <dirent.h>
#endif

#include "common/Settings.hpp"
#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Runtime/UMG/Public/Components/ComboBoxString.h"
#include "SettingsWidget.generated.h"

UCLASS()
class AIRSIM_API USettingsWidget : public UUserWidget
{
	GENERATED_BODY()
	
public:
	bool user_input_ = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dropdown item")
	FString menu_string_;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Test text")
	FText title_text_ = FText::FromString("");

	UFUNCTION(BlueprintCallable, Category = "C++ Interface")
	bool changedText();
	
	UFUNCTION(BlueprintCallable, Category = "C++ Interface")
	bool checkForInput();

	void populateDropdown();

	UFUNCTION(BlueprintImplementableEvent, Category = "C++ Interface")
	void updateMenu();
};