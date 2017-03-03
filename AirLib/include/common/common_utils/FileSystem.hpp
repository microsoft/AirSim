// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef common_utils_FileSystem_hpp
#define common_utils_FileSystem_hpp

#include <codecvt>
#include <fstream>
#include <string>
#include "Utils.hpp"

// This defines a default folder name for all the files created by AirLib so they 
// are all gathered nicely in one place in the user's documents folder.
#ifndef ProductFolderName
#define ProductFolderName "AirSim" 
#endif
    
namespace common_utils { 
class FileSystem
{
    typedef unsigned int uint;
public:

    static const char kPathSeparator =
    #ifdef _WIN32
                                '\\';
    #else
                                '/';
    #endif

    static std::string createDirectory(std::string parentFolder, std::string name);

    static std::string getUserHomeFolder()
    {
        //Windows uses USERPROFILE, Linux uses HOME
    #ifdef _WIN32
        std::wstring userProfile = _wgetenv(L"USERPROFILE");
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        return converter.to_bytes(userProfile);
    #else
        return std::getenv("HOME");
    #endif
    }

    static std::string getUserDocumentsFolder();

    static std::string ensureAppDataFolder() {
        std::string docs = getUserDocumentsFolder();
        // make sure this directory exists.
        return createDirectory(docs, ProductFolderName);
    }

    static std::string getFileExtension(const std::string str)
    {
        // bugbug: this is not unicode safe.
        int len = static_cast<int>(str.size());
        const char* ptr = str.c_str();
        int i = 0;
        for (i = len - 1; i >= 0; i--)
        {
            if (ptr[i] == '.')
                break;
        }
        if (i < 0) return "";
        auto ui = static_cast<uint>(i);
        return str.substr(ui, len - ui);
    }

    static std::string getLogFileNamePath(std::string prefix, std::string suffix, std::string extension, bool add_timestamp)
    {
        std::string timestamp = add_timestamp ? Utils::to_string(Utils::now()) : "";
        std::stringstream filename_ss;
        filename_ss << ensureAppDataFolder() << kPathSeparator << prefix << suffix << timestamp << extension;
        return filename_ss.str();
    }

    static void openTextFile(std::string filepath, std::ifstream& file){
        
#ifdef _WIN32
        // WIN32 will create the wrong file names if we don't first convert them to UTF-16.
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        std::wstring wide_path = converter.from_bytes(filepath);
        file.open(wide_path, std::ios::in);
#else
        file.open(filepath, std::ios::in);
#endif
    }
    
    static void createBinaryFile(std::string filepath, std::ofstream& file){
        
#ifdef _WIN32
        // WIN32 will create the wrong file names if we don't first convert them to UTF-16.
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        std::wstring wide_path = converter.from_bytes(filepath);
        file.open(wide_path, std::ios::binary | std::ios::trunc);
#else
        file.open(filepath, std::ios::binary | std::ios::trunc);
#endif
    }
    
    static void createTextFile(std::string filepath, std::ofstream& file){
        
#ifdef _WIN32
        // WIN32 will create the wrong file names if we don't first convert them to UTF-16.
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        std::wstring wide_path = converter.from_bytes(filepath);
        file.open(wide_path, std::ios::out | std::ios::trunc);
#else
        file.open(filepath, std::ios::trunc);
#endif

        if (file.fail())
            throw std::ios_base::failure(std::strerror(errno));
    }
    
    static std::string createLogFile(std::string suffix, std::ofstream& flog)
    {
        std::string filepath = getLogFileNamePath("log_", suffix, ".tsv", true);
        createTextFile(filepath, flog);

        Utils::logMessage("log file started: %s", filepath.c_str());
        flog.exceptions(flog.exceptions() | std::ios::failbit | std::ifstream::badbit);
        return filepath;
    }

    static std::string readLineFromFile(std::ifstream& file)
    {
        std::string line;
        try {
            std::getline(file, line);
        }
        catch(...) {
            if (!file.eof())
                throw;
        }
        return line;
    }    

    static void appendLineToFile(std::string filepath, std::string line)
    {
        std::ofstream file;
#ifdef _WIN32
        // WIN32 will create the wrong file names if we don't first convert them to UTF-16.
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        std::wstring wide_path = converter.from_bytes(filepath);
        file.open(wide_path, std::ios::out | std::ios::app);
#else
        file.open(filepath, std::ios::out | std::ios::app);
#endif
        if (file.fail())
            throw std::ios_base::failure(std::strerror(errno));
        file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
        file << line << std::endl;
    }

};

} 
#endif