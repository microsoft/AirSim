#pragma once
#include <string>

#include "common/common_utils/Utils.hpp"
#include "common/common_utils/FileSystem.hpp"

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
			std::string file_;
			nlohmann::json doc_;
            bool load_success_ = false;
		public:
			static Settings& singleton() {
				return settings_;
			}

			std::string getFileName() { return file_; }

			static std::string Settings::getFullPath(std::string fileName)
			{
				std::string path = common_utils::FileSystem::ensureAppDataFolder();
				return path + common_utils::FileSystem::kPathSeparator + fileName;
			}

			static Settings& Settings::loadJSonFile(std::string fileName)
			{
				std::string path = getFullPath(fileName);
				settings_.file_ = path;

				settings_.load_success_ = false;

				std::ifstream s;
				common_utils::FileSystem::openTextFile(path, s);
				if (!s.fail()) {
					s >> settings_.doc_;
					settings_.load_success_ = true;
				}

				return singleton();
			}
			bool Settings::isLoadSuccess()
			{
				return load_success_;
			}

			void Settings::saveJSonFile(std::string fileName)
			{
				std::string path = getFullPath(fileName);
				std::ofstream s;
				common_utils::FileSystem::createTextFile(path, s);
				s << std::setw(2) << doc_ << std::endl;
			}

			Settings Settings::getChild(std::string name)
			{
				Settings child;
				if (doc_.count(name) == 1) {
					child.doc_ = doc_[name].get<nlohmann::json>();
				}
				return child;
			}

			std::string Settings::getString(std::string name, std::string defaultValue)
			{
				if (doc_.count(name) == 1) {
					return doc_[name].get<std::string>();
				}
				else {
					doc_[name] = defaultValue;
					return defaultValue;
				}
			}

			double Settings::getDouble(std::string name, double defaultValue)
			{
				if (doc_.count(name) == 1) {
					return doc_[name].get<double>();
				}
				else {
					doc_[name] = defaultValue;
					return defaultValue;
				}
			}

			bool Settings::getBool(std::string name, bool defaultValue)
			{
				if (doc_.count(name) == 1) {
					return doc_[name].get<bool>();
				}
				else {
					doc_[name] = defaultValue;
					return defaultValue;
				}
			}

			int Settings::getInt(std::string name, int defaultValue)
			{
				if (doc_.count(name) == 1) {
					return doc_[name].get<int>();
				}
				else {
					doc_[name] = defaultValue;
					return defaultValue;
				}
			}

			void Settings::setString(std::string name, std::string value)
			{
				doc_[name] = value;
			}
			void Settings::setDouble(std::string name, double value)
			{
				doc_[name] = value;
			}
			void Settings::setBool(std::string name, bool value)
			{
				doc_[name] = value;
			}
			void Settings::setInt(std::string name, int value)
			{
				doc_[name] = value;
			}

			void Settings::setChild(std::string name, Settings& value)
			{
				doc_[name] = value.doc_;
			}

			Settings(){}
			~Settings(){}
		};

	}
}