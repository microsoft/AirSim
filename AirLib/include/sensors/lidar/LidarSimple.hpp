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

namespace msr {
    namespace airlib {

        class LidarSimple : public LidarBase {
        public:
            LidarSimple(const LidarSimpleParams& params = LidarSimpleParams())
                : params_(params)
            {
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
                reporter.writeValue("Lidar-FOV-Upper", params_.vertical_FOV_Upper);
                reporter.writeValue("Lidar-FOV-Lower", params_.vertical_FOV_Lower);
            }
            //*** End: UpdatableState implementation ***//

            virtual ~LidarSimple() = default;

        protected:
            virtual vector<real_T> getPointCloud(const Pose& pose, float deltaTime) = 0;

            const LidarSimpleParams& getParams()
            {
                return params_;
            }

        private: //methods
            void updateOutput()
            {
                TTimeDelta dt = clock()->updateSince(last_time_);

                Output output;
                const GroundTruth& ground_truth = getGroundTruth();

                //order of Pose addition is important here because it also adds quaternions which is not commutative!
                // TODO: need to understand if there are unnecessary copies of vector being made that can be avoided.
                auto pointCloud = getPointCloud(params_.relative_pose + ground_truth.kinematics->pose, dt);
                output.point_cloud = pointCloud;
                output.relative_pose = params_.relative_pose;
                output.time_stamp = clock()->nowNanos();

                last_time_ = output.time_stamp;

                setOutput(output);
            }

        private:
            LidarSimpleParams params_;

            FrequencyLimiter freq_limiter_;
            TTimePoint last_time_;
        }; 

    }
} //namespace
#endif
