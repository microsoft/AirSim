// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ControlServerBase_hpp
#define air_ControlServerBase_hpp

#include "common/Common.hpp"
#include <functional>


namespace msr { namespace airlib {

class ControlServerBase {
public:
    virtual void start(bool block = false) = 0;
    virtual void stop() = 0;
    virtual ~ControlServerBase() = default;
};

}} //namespace
#endif
