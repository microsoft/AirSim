// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_WorldSimApiBase_hpp
#define air_WorldSimApiBase_hpp

#include "common/CommonStructs.hpp"


namespace msr { namespace airlib {


class WorldSimApiBase {
public:
    virtual ~WorldSimApiBase() = default;

    virtual bool isPaused() const = 0;
    virtual void reset() = 0;
    virtual void pause(bool is_paused) = 0;
    virtual void continueForTime(double seconds) = 0;

    virtual bool setSegmentationObjectID(const std::string& mesh_name, int object_id, bool is_name_regex = false) = 0;
    virtual int getSegmentationObjectID(const std::string& mesh_name) const = 0;

    virtual void printLogMessage(const std::string& message,
        const std::string& message_param = "", unsigned char severity = 0) = 0;

    virtual Pose getObjectPose(const std::string& object_name) const = 0;
};


}} //namespace
#endif
