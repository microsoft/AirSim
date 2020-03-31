// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//there is no #pragma once in this header

//following is required to support Unreal Build System
#if (defined _WIN32 || defined _WIN64) && (defined UE_GAME || defined UE_EDITOR)
#pragma warning(pop)
#include "Windows/HideWindowsPlatformAtomics.h"
#include "Windows/HideWindowsPlatformTypes.h"
#include "Windows/PostWindowsApi.h"
#endif
