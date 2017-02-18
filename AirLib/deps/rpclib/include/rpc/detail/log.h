#pragma once

#ifndef LOG_H_SPSC31OG
#define LOG_H_SPSC31OG

#ifdef RPCLIB_ENABLE_LOGGING

#include "format.h"
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <sstream>

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#ifdef _MSC_VER
#include <Windows.h>
#endif

namespace rpc {
namespace detail {
class logger {
public:
    static logger &instance() {
        static logger inst;
        return inst;
    }

    template <typename... Args>
    void trace(const char *channel, const char *file, const char *func,
               std::size_t line, const char *msg, Args... args) {
        (void)func;
        basic_log("TRACE", channel,
                  RPCLIB_FMT::format("{} ({}:{})",
                                     RPCLIB_FMT::format(msg, args...), file,
                                     line));
    }

    template <typename... Args>
    void debug(const char *channel, const char *file, const char *func,
               std::size_t line, const char *msg, Args... args) {
        (void)func;
        basic_log("DEBUG", channel,
                  RPCLIB_FMT::format("{} ({}:{})",
                                     RPCLIB_FMT::format(msg, args...), file,
                                     line));
    }

    template <typename... Args>
    void warn(const char *channel, const char *msg, Args... args) {
        basic_log("WARN", channel, RPCLIB_FMT::format(msg, args...));
    }

    template <typename... Args>
    void error(const char *channel, const char *msg, Args... args) {
        basic_log("ERROR", channel, RPCLIB_FMT::format(msg, args...));
    }

    template <typename... Args>
    void info(const char *channel, const char *msg, Args... args) {
        basic_log("INFO", channel, RPCLIB_FMT::format(msg, args...));
    }

private:
    logger() {}

#ifdef _MSC_VER
    static std::string now() {
        std::stringstream ss;
        SYSTEMTIME t;
        GetSystemTime(&t);
        ss << RPCLIB_FMT::format("{}-{}-{} {}:{}:{}.{:03}", t.wYear, t.wMonth,
                                 t.wDay, t.wHour, t.wMinute, t.wSecond,
                                 t.wMilliseconds);
        return ss.str();
    }
#else
    static std::string now() {
        std::stringstream ss;
        timespec now_t = {};
        clock_gettime(CLOCK_REALTIME, &now_t);
        ss << std::put_time(
                  std::localtime(reinterpret_cast<time_t *>(&now_t.tv_sec)),
                  "%F %T")
           << RPCLIB_FMT::format(
                  ".{:03}", round(static_cast<double>(now_t.tv_nsec) / 1.0e6));
        return ss.str();
    }
#endif

    void basic_log(const char *severity, const char *channel,
                   std::string const &msg) {
        using RPCLIB_FMT::arg;
        std::lock_guard<std::mutex> lock(mut_print_);
        RPCLIB_FMT::print("{time:16}  {severity:6}  {channel:12}    {msg:40}\n",
                          arg("severity", severity), arg("channel", channel),
                          arg("time", now()), arg("msg", msg));
    }

private:
    std::mutex mut_print_;
};
} /* detail */
} /* rpc */

#ifdef _MSC_VER
#define RPCLIB_CREATE_LOG_CHANNEL(Name)                                        \
    static constexpr const char *rpc_channel_name = #Name;
#elif defined(__GNUC__)
#define RPCLIB_CREATE_LOG_CHANNEL(Name)                                        \
    _Pragma("GCC diagnostic push")                                            \
    _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")                       \
    static constexpr const char *rpc_channel_name = #Name;                  \
    _Pragma("GCC diagnostic pop")
#elif defined(__clang__)
    _Pragma("clang diagnostic push")                                            \
    _Pragma("clang diagnostic ignored \"-Wunused-variable\"")                       \
    static constexpr const char *rpc_channel_name = #Name;                  \
    _Pragma("clang diagnostic pop")
#endif

RPCLIB_CREATE_LOG_CHANNEL(global)

#define LOG_INFO(...)                                                          \
    rpc::detail::logger::instance().info(rpc_channel_name, __VA_ARGS__)

#define LOG_WARN(...)                                                          \
    rpc::detail::logger::instance().warn(rpc_channel_name, __VA_ARGS__)

#define LOG_ERROR(...)                                                         \
    rpc::detail::logger::instance().error(rpc_channel_name, __VA_ARGS__)

#define LOG_DEBUG(...)                                                         \
    rpc::detail::logger::instance().debug(rpc_channel_name, __FILE__,    \
                                             __func__, __LINE__, __VA_ARGS__)

#define LOG_TRACE(...)                                                         \
    rpc::detail::logger::instance().trace(rpc_channel_name, __FILE__,    \
                                             __func__, __LINE__, __VA_ARGS__)

#define LOG_EXPR(Level, Expr) LOG_##Level("`" #Expr "` = {}", Expr)

#else

#define LOG_INFO(...)
#define LOG_WARN(...)
#define LOG_ERROR(...)
#define LOG_DEBUG(...)
#define LOG_TRACE(...)
#define LOG_EXPR(...)
#define RPCLIB_CREATE_LOG_CHANNEL(...)

#endif

#endif /* end of include guard: LOG_H_SPSC31OG */
