// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef common_utils_Utils_hpp
#define common_utils_Utils_hpp

#ifdef __GNUC__
#define STRICT_MODE_OFF                                           \
    _Pragma("GCC diagnostic push")                                  \
    _Pragma("GCC diagnostic ignored \"-Wreturn-type\"")             \
    _Pragma("GCC diagnostic ignored \"-Wdelete-non-virtual-dtor\"") \
    _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")        \
    _Pragma("GCC diagnostic ignored \"-pedantic\"")                 \
    _Pragma("GCC diagnostic ignored \"-Wshadow\"")                  \
    _Pragma("GCC diagnostic ignored \"-Wold-style-cast\"")          \
    _Pragma("GCC diagnostic ignored \"-Wswitch-default\"")          \
	_Pragma("GCC diagnostic ignored \"-Wmissing-field-initializers\"") \
	_Pragma("GCC diagnostic ignored \"-Wredundant-decls\"")


/* Addition options that can be enabled
    _Pragma("GCC diagnostic ignored \"-Wpedantic\"")                \
    _Pragma("GCC diagnostic ignored \"-Wformat=\"")                 \
    _Pragma("GCC diagnostic ignored \"-Werror\"")                   \
    _Pragma("GCC diagnostic ignored \"-Werror=\"")                  \
    _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")         \
*/

#define STRICT_MODE_ON                                            \
    _Pragma("GCC diagnostic pop")
#elif defined(_MSC_VER)
#define STRICT_MODE_OFF   __pragma(warning( push, 0 ))
#define STRICT_MODE_ON    __pragma( pop )

#endif


#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <cstdarg>
#include <cstring>
#include <array>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <iomanip>
#include "type_utils.hpp"
#include <limits>


#ifndef _WIN32
#include <limits.h>
#include <sys/param.h>
#endif

//Stubs for C++17 optional type
#if (defined __cplusplus) && (__cplusplus >= 201700L)
#include <optional>
#else
#include "optional.hpp"
#endif

#if (defined __cplusplus) && (__cplusplus >= 201700L)
using std::optional;
#else
using std::experimental::optional;
#endif

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PIf
#define M_PIf ((float)3.1415926535897932384626433832795028841972)
#endif

#ifndef M_PI
#define M_PI ((double)3.1415926535897932384626433832795028841972)
#endif

#ifndef M_PIl
#define M_PIl ((long double)3.1415926535897932384626433832795028841972)
#endif

#define EARTH_RADIUS (6378137.0f)

/*
    This file is collection of routines that can be included in ANY project just
    by dropping in common_utils.hpp. Therefore there should not be any dependency
    in the code below other than STL. The code should be able to compilable on
    all major platforms.
*/


#ifndef _MSC_VER
static int _vscprintf(const char * format, va_list pargs) {
    int retval;
    va_list argcopy;
    va_copy(argcopy, pargs);
    retval = vsnprintf(NULL, 0, format, argcopy);
    va_end(argcopy);
    return retval;
}
#endif

namespace common_utils {

class Utils {
  private:
    typedef std::chrono::system_clock system_clock;
    typedef std::chrono::steady_clock steady_clock;
    typedef std::string string;
    typedef std::stringstream stringstream;
    //this is not required for most compilers
    typedef unsigned int uint;
    template <typename T>
    using time_point = std::chrono::time_point<T>;

  public:
    static const char kPathSeparator =
#ifdef _WIN32
        '\\';
#else
        '/';
#endif

    static void enableImmediateConsoleFlush() {
        //disable buffering
        setbuf(stdout, NULL);
    }
    static double degreesToRadians(double degrees) {
        return static_cast<double>(degrees*(M_PIl / 180.0));
    }
    static float degreesToRadians(float degrees) {
        return static_cast<float>(degrees*(M_PI / 180.0f));
    }
    static void logMessage(const char* message, ...) {
        va_list args;
        va_start(args, message);

        vprintf(message, args);
        printf("\n");
        fflush (stdout);

        va_end(args);
    }
    static void logError(const char* message, ...) {
        va_list args;
        va_start(args, message);

        vfprintf(stderr, message, args);
        fprintf(stderr, "\n");
        fflush (stderr);

        va_end(args);
    }

    template <typename T>
    static int sign(T val) {
        return T(0) < val ? 1 : (T(0) > val ? -1 : 0);
    }

    /// Limits absolute value whole preserving sign
    template <typename T>
    static T limitAbsValue(T val, T min_value, T max_value) {
        T val_abs = std::abs(val);
        T val_limited = std::max(val_abs, min_value);
        val_limited = std::min(val_limited, max_value);
        return sign(val) * val_limited;
    }

    template<typename Range>
    static const string printRange(Range&& range, const string& delim = ", ",
                                   const string& prefix="(", const string& suffix=")") {
        return printRange(std::begin(range), std::end(range), delim, prefix, suffix);
    }
    template<typename Iterator>
    static const string printRange(Iterator start, Iterator last, const string& delim = ", ",
                                   const string& prefix="(", const string& suffix=")") {
        stringstream ss;
        ss << prefix;

        for (Iterator i = start; i != last; ++i) {
            if (i != start)
                ss << delim;
            ss << *i;
        }

        ss << suffix;
        return ss.str();
    }

    static string stringf(const char* format, ...) {
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

    static string getFileExtension(const string str) {
        int len = static_cast<int>(str.size());
        const char* ptr = str.c_str();
        int i = 0;
        for (i = len - 1; i >= 0; i--) {
            if (ptr[i] == '.')
                break;
        }
        if (i < 0) return "";
        return str.substr(i, len - i);
    }

    static string trim(const string& str, char ch) {
        int len = static_cast<int>(str.size());
        const char* ptr = str.c_str();
        int i = 0;
        for (i = 0; i < len; i++) {
            if (ptr[i] != ch)
                break;
        }
        int j = 0;
        for (j = len - 1; j > 0; j--) {
            if (ptr[j] != ch)
                break;
        }
        if (i > j) return "";
        return str.substr(i, j - i + 1);
    }


    //http://stackoverflow.com/a/28703383/207661
    template <typename R>
    static constexpr R bitmask(unsigned int const onecount) {
        //  return (onecount != 0)
        //      ? (static_cast<R>(-1) >> ((sizeof(R) * CHAR_BIT) - onecount))
        //      : 0;
        return static_cast<R>(-(onecount != 0))
               & (static_cast<R>(-1) >> ((sizeof(R) * CHAR_BIT) - onecount));
    }

    static inline int floorToInt(float x) {
        return static_cast<int> (std::floor(x));
    }

    static constexpr float maxFloat() {
        return std::numeric_limits<float>::max();
    }
    static constexpr uint maxUInt() {
        return std::numeric_limits<uint>::max();
    }
    static constexpr float minFloat() {
        return std::numeric_limits<float>::max();
    }

    static constexpr float nanFloat() {
        return std::numeric_limits<float>::quiet_NaN();
    }

    static constexpr float DegreesToRadians(float degrees) {
        return static_cast<float>(M_PI * degrees / 180);
    }

    static void saveToFile(string file_name, const char* data, uint size) {
        std::ofstream file(file_name, std::ios::binary);
        file.write(data, size);
    }

    template<typename Container>
    static typename std::enable_if<type_utils::is_container<Container>::value, void>::type
    append(Container& to, const Container& from) {
        using std::begin;
        using std::end;
        to.insert(end(to), begin(from), end(from));
    }

    template<typename Container>
    static typename std::enable_if<type_utils::is_container<Container>::value, void>::type
    copy(const Container& from, Container& to) {
        using std::begin;
        using std::end;
        std::copy(begin(from), end(from), begin(to));
    }

    template<typename T>
    static void copy(const T* from, T* to, uint count) {
        std::copy(from, from + count, to);
    }

    static const char* to_string(time_point<steady_clock> t) {
        time_t tt = system_clock::to_time_t(std::chrono::time_point_cast<system_clock::duration>(system_clock::now() + (t - steady_clock::now())));
        return ctime(&tt);
    }

    static time_point<system_clock> now() {
        return system_clock::now();
    }

    static string to_string(time_point<system_clock> time) {
        time_t tt = system_clock::to_time_t(time);

        char str[1024];
        if (std::strftime(str, sizeof(str), "%Y-%m-%d-%H-%M-%S", std::localtime(&tt)))
            return string(str);
        else return string();

        /* GCC doesn't implement put_time yet
        stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M-%S");
        return ss.str();
        */
    }

    static string getEnv(const string& var) {
        char* ptr = std::getenv(var.c_str());
        return ptr ? ptr : "";
    }

};

} //namespace
#endif
