#pragma once
#include <string>

#include "common/common_utils/Utils.hpp"
STRICT_MODE_OFF
// this json library is not strict clean
#include "common/common_utils/json.hpp"
STRICT_MODE_ON

namespace msr {
	namespace airlib {

		class Settings
		{
		private:
			static Settings settings_;
			std::wstring file_;
			nlohmann::json doc_;
            bool load_success = false;
		public:
			static const wchar_t* getProductName() {
				return L"AirSim";
			}

			static Settings& singleton() {
				return settings_;
			}

            static std::wstring getFullPath(std::wstring fileName);
            bool isLoadSuccess();
			static Settings& loadJSonFile(std::wstring fileName);
			void saveJSonFile(std::wstring fileName);
			std::wstring getFileName() { return file_; }

			std::string getString(std::string name, std::string defaultValue = "");
			void setString(std::string name, std::string value);

			double getDouble(std::string name, double defaultValue = 0);
			void setDouble(std::string name, double value);

			bool getBool(std::string name, bool defaultValue = false);
			void setBool(std::string name, bool value);

			int getInt(std::string name, int defaultValue = 0);
			void setInt(std::string name, int value);

			Settings getChild(std::string name);
			void setChild(std::string name, Settings& value);


			static std::wstring getUserDocumentsFolder();
			static std::wstring ensureAppDataFolder(std::wstring productName);
			static std::wstring getUserHomeFolder();
			static void createDirectory(std::wstring parentFolder, std::wstring name);

			Settings();
			~Settings();
		};

	}
}