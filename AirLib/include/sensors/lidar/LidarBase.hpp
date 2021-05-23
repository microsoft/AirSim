// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_LidarBase_hpp
#define msr_airlib_LidarBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr
{
namespace airlib
{

    class LidarBase : public SensorBase
    {
    public:
        LidarBase(const std::string& sensor_name = "")
            : SensorBase(sensor_name)
        {
        }

    public:
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            UpdatableObject::reportState(reporter);

            reporter.writeValue("Lidar-Timestamp", output_.time_stamp);
            reporter.writeValue("Lidar-NumPoints", static_cast<int>(output_.point_cloud.size() / 3));
        }

        const LidarData& getOutput() const
        {
            return output_;
        }

    protected:
        void setOutput(const LidarData& output)
        {
            output_ = output;
        }

    private:
        LidarData output_;
    };
}
} //namespace
#endif
