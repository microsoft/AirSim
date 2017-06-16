// in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//if using Unreal Build system then include precompiled header file first
#ifdef AIRLIB_PCH
#include "AirSim.h"
#endif

#include "controllers/Settings.hpp"

using namespace msr::airlib;

Settings Settings::settings_;

std::mutex Settings::file_access_;

#endif