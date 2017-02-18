// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef MavLinkCom_MavLinkLog_hpp
#define MavLinkCom_MavLinkLog_hpp

#include <string>
#include <stdio.h>
#include "MavLinkMessageBase.hpp"

namespace mavlinkcom {

// This class reads/writes MavLinkMessages to a local file.
class MavLinkLog {
    std::string file_name_;
    FILE* ptr_;
    bool reading_;
    bool writing_;
  public:
    MavLinkLog();
    ~MavLinkLog();
    bool isOpen();
    void openForReading(const std::string& filename);
    void openForWriting(const std::string& filename);
    void close();
    void write(const mavlinkcom::MavLinkMessage& msg);
    bool read(mavlinkcom::MavLinkMessage& msg);
};

}


#endif
