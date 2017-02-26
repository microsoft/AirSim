// in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "controllers/Settings.h"
#include "common/common_utils/Utils.hpp"
#include <codecvt>
#include <fstream>

// #include "Shlobj.h" (this doesn't work because it defines a bunch of other conflicting stuff).
#ifdef _WIN32

extern "C" {

#ifndef _SHLOBJ_H_
	__declspec(dllimport)
		int
		__stdcall
		SHGetFolderPathW(_In_ void*  hwnd, _In_ int csidl, _In_opt_ void * hToken, uint32_t dwFlags, _Out_writes_(MAX_PATH) wchar_t* pszPath);

#define SHGetFolderPath SHGetFolderPathW

#define CSIDL_PERSONAL                  0x0005        // My Documents
#define CSIDL_MYDOCUMENTS               CSIDL_PERSONAL //  Personal was just a silly name for My Documents
#define CSIDL_FLAG_CREATE               0x8000        // combine with CSIDL_ value to force folder creation in SHGetFolderPath()
#endif

#ifndef CreateDirectoryEx
	__declspec(dllimport)
		bool
		__stdcall
		CreateDirectoryExW(
			_In_     const wchar_t* lpTemplateDirectory,
			_In_     const wchar_t* lpNewDirectory,
			_In_opt_ void* lpSecurityAttributes
		);

#define CreateDirectoryEx  CreateDirectoryExW

#endif

#ifndef MAX_PATH
#define MAX_PATH          260
#endif

#ifndef _ERRHANDLING_H_

	__declspec(dllimport)
		int
		__stdcall
		GetLastError(
			void
		);
#endif
}


#endif

#ifdef _WIN32
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#define getcwd _getcwd
#else
#include <unistd.h>
#include <sys/param.h> // MAXPATHLEN definition
#include <sys/stat.h> // get mkdir.
#endif

using json = nlohmann::json;

using namespace msr::airlib;

Settings Settings::settings_;

Settings::Settings()
{
}


Settings::~Settings()
{
}

std::wstring Settings::getFullPath(std::wstring fileName)
{
	wchar_t sep = static_cast<wchar_t>(common_utils::Utils::kPathSeparator);
	std::wstring path = ensureAppDataFolder(Settings::getProductName());
	return path + sep + fileName;
}

Settings& Settings::loadJSonFile(std::wstring fileName)
{
	std::wstring path = getFullPath(fileName);
	settings_.file_ = path;

    settings_.load_success = false;

#ifdef _WIN32
    std::ifstream s(path);
#else
    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
    std::string path_string = converter.to_bytes(path);
    std::ifstream s(path_string);
#endif

	if (!s.fail()) {
		json doc_;
		s >> settings_.doc_;
        settings_.load_success = true;
	}

	return singleton();
}
bool Settings::isLoadSuccess()
{
    return load_success;
}

void Settings::createDirectory(std::wstring parentFolder, std::wstring name) {
	wchar_t sep = static_cast<wchar_t>(common_utils::Utils::kPathSeparator);
	std::wstring path = parentFolder + sep + name;
#ifdef _WIN32    
	int hr = CreateDirectoryEx(parentFolder.c_str(), path.c_str(), NULL);
	if (hr != 0) {
		common_utils::Utils::logMessage("CreateDirectoryEx failed %d\n", GetLastError());
	}
#else
    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
    std::string path_string = converter.to_bytes(path);
	  mkdir(path_string.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
}

std::wstring Settings::getUserHomeFolder()
{
	//Windows uses USERPROFILE, Linux uses HOME
#ifdef _WIN32
	return _wgetenv(L"USERPROFILE");
#else
	std::string result = std::getenv("HOME");
	return std::wstring(result.begin(), result.end());
#endif
}


std::wstring Settings::ensureAppDataFolder(std::wstring productName) {

	wchar_t sep = static_cast<wchar_t>(common_utils::Utils::kPathSeparator);
	std::wstring docs = getUserDocumentsFolder();
	std::wstring path = docs + sep + productName;
	// make sure this directory exists.
	createDirectory(docs, productName);
	return path;
}

std::wstring Settings::getUserDocumentsFolder() {
	//Windows uses USERPROFILE, Linux uses HOME
	wchar_t sep = static_cast<wchar_t>(common_utils::Utils::kPathSeparator);
#ifdef _WIN32
	wchar_t szPath[MAX_PATH];

	if (0 == SHGetFolderPath(NULL,
		CSIDL_MYDOCUMENTS | CSIDL_FLAG_CREATE,
		NULL,
		0,
		szPath))
	{
		return std::wstring(szPath);
	}

	return getUserHomeFolder() + sep + L"Documents";
#else
	return getUserHomeFolder() + sep + L"Documents";
#endif
}

void Settings::saveJSonFile(std::wstring fileName)
{
    std::wstring path = getFullPath(fileName);
#ifdef _WIN32
	std::ofstream s(path);
#else
    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
    std::string path_string = converter.to_bytes(path);
    std::ofstream s(path_string);
#endif
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

#endif