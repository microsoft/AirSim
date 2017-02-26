// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_air_copter_sim_ControllerBase_hpp
#define msr_air_copter_sim_ControllerBase_hpp

#include "common/UpdatableObject.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

/*
    This class defined the interface for vehicle controllers. A typical vehicle controller would consume
    sensor values as inputs and return control signals for various vertices on physics body as output. The update()
    method should be used to process the sensor inputs from vehicle as needed (the implementor should have vehicle object
    possibly through initialization). The getVertexControlSignal() is called to retrieve the value of control signal to be applied
    to physics body vertex. In addition, start() and stop() method might be used for purposes like setting up connection to vehicle. 
    While reset() may be called at any time, all other methods should be expected be called after start() and before stop().
*/
class ControllerBase : public UpdatableObject {
public:
    //reset any state in the controller
	virtual void reset() override = 0;
	virtual void update(real_T dt) override = 0;

    //return 0 to 1 (corresponds to zero to full thrust)
	virtual real_T getVertexControlSignal(unsigned int rotor_index) = 0;
    virtual size_t getVertexCount() = 0;
    
    virtual void getStatusMessages(std::vector<std::string>& messages)
    {
        //default implementation
    }

    virtual void start()
    {
        //default implementation
        //potentially open any serial/TCP connections or allocate resources
    }
    virtual void stop()
    {
        //default implementation
        //clean up any resources
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
