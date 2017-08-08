// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ControllerBase_hpp
#define msr_airlib_ControllerBase_hpp

#include "common/UpdatableObject.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

/*
    This class defines the interface for vehicle controllers. A typical vehicle controller would consume
    sensor values as inputs and return control signals for various vertices on physics body as output. The update()
    method should be used to process the sensor inputs from vehicle as needed (the implementor should have vehicle object
    possibly through initialization). The getVertexControlSignal() is called to retrieve the value of control signal to be applied
    to physics body vertex.
    While reset() may be called at any time, all other methods should be expected be called after start() and before stop().
*/
class ControllerBase : public UpdatableObject {
public:
    //return 0 to 1 (corresponds to zero to full thrust)
    virtual real_T getVertexControlSignal(unsigned int rotor_index) = 0;
    virtual size_t getVertexCount() = 0;
    
    virtual void getStatusMessages(std::vector<std::string>& messages)
    {
        unused(messages);
        //default implementation
    }

    virtual void reportTelemetry(float renderTime)
    {
        unused(renderTime);
        //no default action
    }

};

class ControllerException : public std::runtime_error {
public:
    ControllerException(const std::string& message)
        : runtime_error(message) { 
    }
};   

}} //namespace
#endif
