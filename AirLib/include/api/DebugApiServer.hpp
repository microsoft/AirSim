// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_DebugApiServer_hpp
#define air_DebugApiServer_hpp

#include "ControlServerBase.hpp"
#include "common/common_utils/Utils.hpp"

namespace msr { namespace airlib {

    class DebugApiServer : public ControlServerBase {
    public:
        virtual void start(bool block = false) override
        {
            common_utils::Utils::log("Debug server started");
        }
        virtual void stop() override
        {
            common_utils::Utils::log("Debug server stopped");
        }
        virtual ~DebugApiServer() = default;
    };

}} //namespace
#endif
