#pragma once
#include <string>
#include <mutex>
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
            std::string file_;
            nlohmann::json doc_;
            bool load_success_ = false;

        private:
            static std::mutex& getFileAccessMutex()
            {
                static std::mutex file_access;
                return file_access;
            }

        public:
            static Settings& singleton() {
                static Settings instance;
                return instance;
            }

            std::string getFileName() { return file_; }

            static std::string getFullPath(std::string fileName)
            {
                std::string path = common_utils::FileSystem::getAppDataFolder();
                return common_utils::FileSystem::combine(path, fileName);
            }

            static Settings& loadJSonFile(std::string fileName)
            {
                std::lock_guard<std::mutex> guard(getFileAccessMutex());
                std::string path = getFullPath(fileName);
                singleton().file_ = fileName;

                singleton().load_success_ = false;

                std::ifstream s;
                common_utils::FileSystem::openTextFile(path, s);
                if (!s.fail()) {
                    s >> singleton().doc_;
                    singleton().load_success_ = true;
                }

                return singleton();
            }
            bool isLoadSuccess()
            {
                return load_success_;
            }

            bool hasFileName()
            {
                return !getFileName().empty();
            }

            void saveJSonFile(std::string fileName)
            {
                std::lock_guard<std::mutex> guard(getFileAccessMutex());
                std::string path = getFullPath(fileName);
                std::ofstream s;
                common_utils::FileSystem::createTextFile(path, s);
                s << std::setw(2) << doc_ << std::endl;
            }

            bool getChild(std::string name, Settings& child) const
            {
                if (doc_.count(name) == 1 && doc_[name].type() == nlohmann::detail::value_t::object) {
                    child.doc_ = doc_[name].get<nlohmann::json>();
                    return true;
                }
                return false;
            }

            std::string getString(std::string name, std::string defaultValue) const
            {
                if (doc_.count(name) == 1) {
                    return doc_[name].get<std::string>();
                }
                else {
                    return defaultValue;
                }
            }

            double getDouble(std::string name, double defaultValue) const
            {
                if (doc_.count(name) == 1) {
                    return doc_[name].get<double>();
                }
                else {
                    return defaultValue;
                }
            }

            bool getBool(std::string name, bool defaultValue) const
            {
                if (doc_.count(name) == 1) {
                    return doc_[name].get<bool>();
                }
                else {
                    return defaultValue;
                }
            }

            int getInt(std::string name, int defaultValue) const
            {
                if (doc_.count(name) == 1) {
                    return doc_[name].get<int>();
                }
                else {
                    return defaultValue;
                }
            }

            bool setString(std::string name, std::string value)
            {
                if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::string || doc_[name] != value) {
                    doc_[name] = value;
                    return true;
                }
                return false;
            }
            bool setDouble(std::string name, double value)
            {
                if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::number_float || static_cast<double>(doc_[name]) != value) {
                    doc_[name] = value;
                    return true;
                }
                return false;
            }
            bool setBool(std::string name, bool value)
            {
                if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::boolean || static_cast<bool>(doc_[name]) != value) {
                    doc_[name] = value;
                    return true;
                }
                return false;
            }
            bool setInt(std::string name, int value)
            {
                if (doc_.count(name) != 1 || doc_[name].type() != nlohmann::detail::value_t::number_integer || static_cast<int>(doc_[name]) != value) {
                    doc_[name] = value;
                    return true;
                }
                return false;
            }

            void setChild(std::string name, Settings& value)
            {
                doc_[name] = value.doc_;
            }
        };

    }
}