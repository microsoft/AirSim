// Liscence info

#ifndef msr_airlib_AirSimSimpleEkfBase_hpp
#define msr_airlib_AirSimSimpleEkfBase_hpp

#include <cmath>
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"
#include "firmware/interfaces/IEkf.hpp"
#include "AirSimSimpleEkfParams.hpp"

namespace msr
{
namespace airlib
{

    class AirSimSimpleEkfBase : public simple_flight::IEkf
    {
    public:
        virtual bool checkEkfEnabled() const override
        {
            return params_.ekf_enabled;
        }

        // getters
        virtual const simple_flight::VectorNXf& getEkfStates() const override
        {
            return states_;
        }
        virtual const VectorMath::Vector17f& getEkfMeasurements() const override
        {
            return measurement_;
        }
        virtual const simple_flight::MatrixNXxNXf& getEkfCovariance() const override
        {
            return error_covariance_;
        }
        virtual const VectorMath::Matrix3x3f& getEkfEulerAnglesCovariance() const override
        {
            return euler_angles_error_covariance_;
        }

    protected:
        // setters
        void setEkfCovariance(simple_flight::MatrixNXxNXf error_covariance)
        {
            error_covariance_ = error_covariance;
        }

    protected:
        AirSimSimpleEkfParams params_;

        simple_flight::VectorNXf states_;
        simple_flight::MatrixNXxNXf error_covariance_;
        VectorMath::Matrix3x3f euler_angles_error_covariance_;
        VectorMath::Vector17f measurement_ = VectorMath::Vector17f::Zero();
    };

}
} //namespace
#endif