// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "Utils.hpp"

#ifndef _MSC_VER
static int _vscprintf(const char * format, va_list pargs)
{
    int retval;
    va_list argcopy;
    va_copy(argcopy, pargs);
    retval = vsnprintf(NULL, 0, format, argcopy);
    va_end(argcopy);
    return retval;
}
#endif
#ifdef _WIN32
#include <Windows.h> // OutputDebugStringA
#endif

using namespace common_utils;
using namespace std;

void Utils::logMessage(const char* message, ...) {
    va_list args;
    va_start(args, message);

    int size = _vscprintf(message, args) + 1;
    std::unique_ptr<char[]> buf(new char[size]);

#ifndef _MSC_VER
    vsnprintf(buf.get(), size, message, args);
#else
    vsnprintf_s(buf.get(), size, _TRUNCATE, message, args);
#endif

    va_end(args);

    printf("%s\n", buf.get());
#ifdef _WIN32
    OutputDebugStringA(buf.get());
    OutputDebugStringA("\n");
#endif
}

void Utils::logError(const char* message, ...) {
    va_list args;
    va_start(args, message);

    int size = _vscprintf(message, args) + 1;
    std::unique_ptr<char[]> buf(new char[size]);

#ifndef _MSC_VER
    vsnprintf(buf.get(), size, message, args);
#else
    vsnprintf_s(buf.get(), size, _TRUNCATE, message, args);
#endif

    va_end(args);

    fprintf(stderr, "%s\n", buf.get());
#ifdef _WIN32
    OutputDebugStringA(buf.get());
    OutputDebugStringA("\n");
#endif
}

string Utils::stringf(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    int size = _vscprintf(format, args) + 1;
    std::unique_ptr<char[]> buf(new char[size] ); 

    #ifndef _MSC_VER
        vsnprintf(buf.get(), size, format, args);
    #else
        vsnprintf_s(buf.get(), size, _TRUNCATE, format, args);
    #endif

    va_end(args);            

    return string(buf.get());
}

string Utils::getFileExtension(const string str)
{
	int len = static_cast<int>(str.size());
	const char* ptr = str.c_str();
	int i = 0;
	for (i = len - 1; i >= 0; i--)
	{
		if (ptr[i] == '.')
			break;
	}
	if (i < 0) return "";
	return str.substr(i, len - i);
}

string Utils::trim(const string& str, char ch)
{
	int len = static_cast<int>(str.size());
	const char* ptr = str.c_str();
	int i = 0;
	for (i = 0; i < len; i++)
	{
		if (ptr[i] != ch)
			break;
	}
	int j = 0;
	for (j = len - 1; j > 0; j--)
	{
		if (ptr[j] != ch)
			break;
	}
	if (i > j) return "";
	return str.substr(i, j - i + 1);
}


