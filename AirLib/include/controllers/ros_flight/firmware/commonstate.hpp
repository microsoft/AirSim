#pragma once

#include <cstdio>

namespace ros_flight {

class CommonState {
public:
    typedef enum
    {
        ARMED,
        DISARMED,
        FAILSAFE_ARMED,
        FAILSAFE_DISARMED
    } armed_state_t;

    armed_state_t get_armed_state() {
        return _armed_state;
    }
    void setArmedState(armed_state_t state) {
        _armed_state = state;
    }

    bool is_armed()
    {
        return  _armed_state == armed_state_t::ARMED;
    }
    bool is_disarmed()
    {
        return  _armed_state == armed_state_t::DISARMED;
    }
    void set_disarm()
    {
        setArmedState(armed_state_t::DISARMED);
    }
    void set_arm()
    {
        setArmedState(armed_state_t::ARMED);
    }

    static std::string stringf(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        IGNORE_FORMAT_STRING_ON
        auto size = _vscprintf(format, args) + 1U;
        IGNORE_FORMAT_STRING_OFF
        std::unique_ptr<char[]> buf(new char[size] ); 

#ifndef _MSC_VER
        IGNORE_FORMAT_STRING_ON
        vsnprintf(buf.get(), size, format, args);
        IGNORE_FORMAT_STRING_OFF
#else
        vsnprintf_s(buf.get(), size, _TRUNCATE, format, args);
#endif

        va_end(args);            

        return std::string(buf.get());
    }

private:
#ifndef _MSC_VER
    static int _vscprintf(const char * format, va_list pargs)
    {
        int retval;
        va_list argcopy;
        va_copy(argcopy, pargs); 
        IGNORE_FORMAT_STRING_ON
        retval = vsnprintf(NULL, 0, format, argcopy);
        IGNORE_FORMAT_STRING_OFF
        va_end(argcopy);
        return retval;
    }
#endif

private:
    armed_state_t _armed_state;
};


} //namespace