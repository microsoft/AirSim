// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Lidar_hpp
#define msr_airlib_Lidar_hpp

#include <random>
#include "common/Common.hpp"
#include "LidarSimpleParams.hpp"
#include "LidarBase.hpp"
#include "common/DelayLine.hpp"
#include "common/FrequencyLimiter.hpp"

namespace msr { namespace airlib {

class LidarSimple : public LidarBase {
public:
    LidarSimple(AirSimSettings::SensorSetting* sensor_setting = nullptr)
        : LidarBase(sensor_setting != nullptr ? sensor_setting->sensor_name : "")
    {
        // initialize params
        if (sensor_setting != nullptr)
            params_.initializeFromSettings(*static_cast<const AirSimSettings::LidarSetting*>(sensor_setting));

        //initialize frequency limiter
        freq_limiter_.initialize(params_.update_frequency, params_.startup_delay);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void reset() override
    {
        LidarBase::reset();

        freq_limiter_.reset();
        last_time_ = clock()->nowNanos();

        updateOutput();
    }

    virtual void update() override
    {
        LidarBase::update();

        freq_limiter_.update();

        if (freq_limiter_.isWaitComplete()) {
            updateOutput();
        }
    }

    virtual void reportState(StateReporter& reporter) override
    {
        //call base
        LidarBase::reportState(reporter);

        reporter.writeValue("Lidar-NumChannels", params_.number_of_channels);
        reporter.writeValue("Lidar-Range", params_.range);
        reporter.writeValue("Lidar-FOV-Upper", params_.vertical_FOV_upper);
        reporter.writeValue("Lidar-FOV-Lower", params_.vertical_FOV_lower);
    }
    //*** End: UpdatableState implementation ***//

    virtual ~LidarSimple() = default;

    const LidarSimpleParams& getParams() const
    {
        return params_;
    }

protected:
    virtual void getPointCloud(const Pose& lidar_pose, const Pose& vehicle_pose, 
        TTimeDelta delta_time, vector<real_T>& point_cloud) = 0;

    
private: //methods
    void updateOutput()
    {
        TTimeDelta delta_time = clock()->updateSince(last_time_);

        point_cloud_.clear();

        const GroundTruth& ground_truth = getGroundTruth();

        getPointCloud(params_.relative_pose, // relative lidar pose
            ground_truth.kinematics->pose,   // relative vehicle pose
            delta_time, 
            point_cloud_);

        LidarData output;
        output.point_cloud = point_cloud_;
        output.time_stamp = clock()->nowNanos();

        last_time_ = output.time_stamp;

        setOutput(output);
    }

private:
    LidarSimpleParams params_;
    vector<real_T> point_cloud_;

    FrequencyLimiter freq_limiter_;
    TTimePoint last_time_;
};

}} //namespace
#endif
