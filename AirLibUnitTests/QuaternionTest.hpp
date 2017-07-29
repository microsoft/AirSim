#ifndef msr_AirLibUnitTests_QuaternionTest_hpp
#define msr_AirLibUnitTests_QuaternionTest_hpp

#include "TestBase.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

class QuaternionTest : public TestBase
{
public:
    virtual void run() override
    {
        RandomGeneratorR r(-1000, 1000);
        RandomGeneratorR rw(0, 2000);
        for (auto i = 0; i < 1000; ++i) {
            Quaternionr q(rw.next(), r.next(), r.next(), r.next());
            q.normalize();

            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(q, pitch, roll, yaw);

            Quaternionr qd = VectorMath::toQuaternion(pitch, roll, yaw);
            if (qd.w() < 0) {
                qd.coeffs() = - qd.coeffs();
            }

            auto dw = std::abs(qd.w() - q.w());
            auto dx = std::abs(qd.z() - q.z());
            auto dy = std::abs(qd.y() - q.y());
            auto dz = std::abs(qd.z() - q.z());

            testAssert(dw + dx + dy + dz < 1E-5, "quaternion transformations are not symmetric");

        }
    }
};

} }

#endif