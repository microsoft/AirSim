// in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "common/common_utils/FileSystem.hpp"
#include <codecvt>
#include <fstream>

#ifdef _WIN32
#include <Shlobj.h>
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#else
#include <unistd.h>
#include <sys/param.h> // MAXPATHLEN definition
#include <sys/stat.h> // get mkdir.
#endif

using namespace common_utils;

// File names are unicode (std::wstring), because users can create folders containing unicode characters on both
// Windows, OSX and Linux.
std::string FileSystem::createDirectory(std::string parentFolder, std::string name) {
    std::string path = parentFolder + kPathSeparator + name;
#ifdef _WIN32

    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
    std::wstring wide_parent = converter.from_bytes(parentFolder);
    std::wstring wide_name = converter.from_bytes(name);
    int hr = CreateDirectoryEx(wide_parent.c_str(), wide_name.c_str(), NULL);
    if (hr != 0) {
        common_utils::Utils::logMessage("CreateDirectoryEx failed %d\n", GetLastError());
    }
#else
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
    return path;
}


std::string FileSystem::getUserDocumentsFolder() {
#ifdef _WIN32
    // Windows users can move the Documents folder to any location they want
    // SHGetFolderPath knows how to find it.
    wchar_t szPath[MAX_PATH];

    if (0 == SHGetFolderPath(NULL,
        CSIDL_MYDOCUMENTS | CSIDL_FLAG_CREATE,
        NULL,
        0,
        szPath))
    {
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        return converter.to_bytes(szPath);
    }

    // fall back in case SHGetFolderPath failed for some reason.
    return getUserHomeFolder() + kPathSeparator + "Documents";
#else
    return getUserHomeFolder() + kPathSeparator + "Documents";
#endif
}

#endif