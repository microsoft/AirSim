#include "SettingsWidget.h"

bool USettingsWidget::changedText()
{
    FString settings_string = title_text_.ToString();
    UE_LOG(LogTemp, Warning, TEXT("load file: %s"), *settings_string);
	user_input_ = true;
    return true;
}

bool USettingsWidget::checkForInput()
{
    return user_input_;
}

void USettingsWidget::populateDropdown()
{
    std::string dir(msr::airlib::Settings::Settings::getUserDirectoryFullPath(""));
    std::vector<std::string> files;
    std::shared_ptr<DIR> directory_ptr(opendir(dir.c_str()), [](DIR* dir){ dir && closedir(dir); });
    struct dirent *dirent_ptr;
    if (!directory_ptr) {
        std::cout << "Error opening : " << std::strerror(errno) << dir << std::endl;
        return;
    }
    while ((dirent_ptr = readdir(directory_ptr.get())) != nullptr) {
        std::string filename(dirent_ptr->d_name);
        if (filename.length() >= 4){
            if (filename.substr(filename.length()-4) == "json")
            {
                menu_string_ = FString(std::string(dirent_ptr->d_name).c_str());
                updateMenu();
            }
        } 
    }
}