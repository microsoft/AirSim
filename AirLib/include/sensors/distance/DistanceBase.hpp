// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_DistanceBase_hpp
#define msr_airlib_DistanceBase_hpp


#include "sensors/SensorBase.hpp"


namespace msr { namespace airlib {

class DistanceBase  : public SensorBase {
public:
    DistanceBase(const std::string& sensor_name = "")
        : SensorBase(sensor_name)
    {}

public:
    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        UpdatableObject::reportState(reporter);

        reporter.writeValue("Dist-Curr", output_.distance);
    }

    const DistanceSensorData& getOutput() const
    {
        return output_;
    }

protected:
    void setOutput(const DistanceSensorData& output)
    {
        output_ = output;
    }


private:
    DistanceSensorData output_;
};


}} //namespace
#endif
