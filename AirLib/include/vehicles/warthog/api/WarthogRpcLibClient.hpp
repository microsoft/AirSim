// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_WarthogRpcLibClient_hpp
#define air_WarthogRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "vehicles/warthog/api/WarthogApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "common/ImageCaptureBase.hpp"

namespace msr
{
namespace airlib
{

    class WarthogRpcLibClient : public RpcLibClientBase
    {
    public:
        WarthogRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

        void setWarthogControls(const WarthogApiBase::WarthogControls& controls, const std::string& vehicle_name = "");
        WarthogApiBase::WarthogState getWarthogState(const std::string& vehicle_name = "");
        WarthogApiBase::WarthogControls getWarthogControls(const std::string& vehicle_name = "");
        virtual ~WarthogRpcLibClient(); //required for pimpl
    };
}
} //namespace
#endif
