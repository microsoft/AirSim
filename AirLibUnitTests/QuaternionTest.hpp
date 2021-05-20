#ifndef msr_AirLibUnitTests_QuaternionTest_hpp
#define msr_AirLibUnitTests_QuaternionTest_hpp

#include "TestBase.hpp"
#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

    class QuaternionTest : public TestBase
    {
    public:
        virtual void run() override
        {
            //eulerAngleTest();
            //lookAtTest();
            rotationOrderTest();
        }

    private:
        void rotationOrderTest()
        {
            Quaternionr q1 = VectorMath::toQuaternion(0, 0, Utils::degreesToRadians(90.0f));
            Quaternionr q2 = VectorMath::toQuaternion(Utils::degreesToRadians(90.0f), 0, 0);
            Quaternionr q12 = q1 * q2;
            Quaternionr q21 = q2 * q1;

            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(q12, pitch, roll, yaw);
            VectorMath::toEulerianAngle(q21, pitch, roll, yaw);
        }
        void worldBodyTransformTest()
        {
            //vehicle wrt to world
            Quaternionr vehicle_world_q = VectorMath::toQuaternion(0, 0, Utils::degreesToRadians(45.0f));
            //lidar wrt to vehicle
            Quaternionr lidar_vehicle_q = VectorMath::toQuaternion(0, 0, Utils::degreesToRadians(10.0f));
            //lidar wrt to world
            Quaternionr lidar_world_q = VectorMath::rotateQuaternion(lidar_vehicle_q, vehicle_world_q, false);
            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(vehicle_world_q, pitch, roll, yaw);
            VectorMath::toEulerianAngle(lidar_world_q, pitch, roll, yaw);
            Utils::log(Utils::stringf("%f", Utils::radiansToDegrees(yaw)));
        }

        Quaternionr lookAt(Vector3r sourcePoint, Vector3r destPoint)
        {
            Vector3r toVector = (destPoint - sourcePoint).normalized();

            Vector3r rotAxis = VectorMath::front().cross(toVector).normalized();
            if (rotAxis.squaredNorm() == 0)
                rotAxis = VectorMath::up();
            float dot = VectorMath::front().dot(toVector);
            float ang = std::acos(dot);

            return VectorMath::toQuaternion(rotAxis, ang);
        }

        Quaternionr toQuaternion(const Vector3r& axis, float angle)
        {
            auto s = std::sin(angle / 2.0f);
            auto u = axis.normalized();
            return Quaternionr(std::cos(angle / 2.0f), u.x() * s, u.y() * s, u.z() * s);
        }

        void lookAtTest()
        {
            auto q = lookAt(Vector3r::Zero(), Vector3r(-1.f, 0, 0));
            std::cout << VectorMath::toString(q) << std::endl;

            float pitch, roll, yaw;
            VectorMath::toEulerianAngle(q, pitch, roll, yaw);
            std::cout << pitch << "\t" << roll << "\t" << yaw << "\t" << std::endl;

            //q = VectorMath::toQuaternion(0, 0, Utils::degreesToRadians(180.0f));
            //std::cout << VectorMath::toString(q) << std::endl;
            //VectorMath::toEulerianAngle(q, pitch, roll, yaw);
            //std::cout << pitch << "\t" << roll << "\t" << yaw << "\t" << std::endl;
        }

        void eulerAngleTest()
        {
            RandomGeneratorR r(-1000.0f, 1000.0f);
            RandomGeneratorR rw(-1000.0f, 1000.0f);
            for (auto i = 0; i < 1000; ++i) {
                Quaternionr q(rw.next(), r.next(), r.next(), r.next());
                q.normalize();

                float pitch, roll, yaw;
                VectorMath::toEulerianAngle(q, pitch, roll, yaw);

                Quaternionr qd = VectorMath::toQuaternion(pitch, roll, yaw);
                if (std::signbit(qd.w()) != std::signbit(q.w())) {
                    qd.coeffs() = -qd.coeffs();
                }

                auto dw = std::abs(qd.w() - q.w());
                auto dx = std::abs(qd.z() - q.z());
                auto dy = std::abs(qd.y() - q.y());
                auto dz = std::abs(qd.z() - q.z());

                testAssert(dw + dx + dy + dz < 1E-5, "quaternion transformations are not symmetric");
            }
        }
    };
}
}

#endif