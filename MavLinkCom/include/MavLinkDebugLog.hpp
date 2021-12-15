// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef mavlink_Logger_hpp
#define mavlink_Logger_hpp

#include <iostream>
#include <string>

namespace mavlinkcom
{
class MavLinkDebugLog
{
public:
    virtual void log(int level, const std::string& message)
    {
        if (level >= 0)
            std::cout << message;
        else
            std::cerr << message;
    }

    static MavLinkDebugLog* getSetLogger(MavLinkDebugLog* logger = nullptr)
    {
        static MavLinkDebugLog logger_default_;
        static MavLinkDebugLog* logger_;

        if (logger != nullptr)
            logger_ = logger;
        else if (logger_ == nullptr)
            logger_ = &logger_default_;

        return logger_;
    }

    virtual ~MavLinkDebugLog() = default;
};
}

#endif
