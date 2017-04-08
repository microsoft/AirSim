// in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "common/common_utils/Log.hpp"

using namespace common_utils;

// default implementation that can be overridden by the user via Log::setLog().
Log* Log::log_ = new Log();

#endif