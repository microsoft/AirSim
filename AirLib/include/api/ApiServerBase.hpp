// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_ApiServerBase_hpp
#define air_ApiServerBase_hpp

#include "common/Common.hpp"
#include <functional>


namespace msr { namespace airlib {

class ApiServerBase {
public:
    virtual void start(bool block = false) = 0;
    virtual void stop() = 0;
    virtual ~ApiServerBase() = default;
};

}} //namespace
#endif
