
#ifndef Jacobian_Test_hpp
#define Jacobian_Test_hpp

#include "TestBase.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/AirSimSimpleEkfModel.hpp"

namespace msr
{
namespace airlib
{
    class JacobianTest : public TestBase
    {
    public:
        virtual void run() override
        {
            float x[17] = {1.0f,    2.0f,   3.0f,
                           1.2f,    2.3f,   3.4f,
                           0.9f,    0.1f,   0.2f,    0.3f,
                           0.0f,    0.0f,   0.0f,    0.0f,    0.0f,    0.0f,  0.0f};
            float u[6]  = {0.0f,    0.0f,   -9.80665f,   1.0f,      2.0f,     3.0f};
            float u2[6] = {0.0f,    0.0f,   -9.80665f,  -3.23f,    -1.38f,   -0.2f};
            float x_dot[17];
            float x_dotdot[17];
            float x_dotdotdot[17];

            AirSimSimpleEkfModel::evaluateStateDot(x_dot,x,u);
            AirSimSimpleEkfModel::evaluateStateDot(x_dotdot,x_dot,u2);
            AirSimSimpleEkfModel::evaluateStateDot(x_dotdotdot,x_dotdot,u2);

            float x_dotNew[17];
            float x_dotdotNew[17];
            float x_dotdotdotNew[17];

            AirSimSimpleEkfModel::evaluateStateDotOld(x_dotNew,x,u);
            AirSimSimpleEkfModel::evaluateStateDotOld(x_dotdotNew,x_dotNew,u2);
            AirSimSimpleEkfModel::evaluateStateDotOld(x_dotdotdotNew,x_dotdotNew,u2);

            for (int i=0; i<17; i++)
            {
                std::cout << x[i] << '\t';
                std::cout << x_dot[i] << '\t';
                std::cout << x_dotdot[i] << '\t';
                std::cout << x_dotdotdot[i] << '\n';
            }
            for (int i=0; i<17; i++)
            {
                std::cout << x[i] << '\t';
                std::cout << x_dotNew[i] << '\t';
                std::cout << x_dotdotNew[i] << '\t';
                std::cout << x_dotdotdotNew[i] << '\n';
            }
            VectorMath::Matrix17x17f A;
            VectorMath::Matrix17x17f ANew;
            AirSimSimpleEkfModel::evaluateA(&A, x_dotdotdot,u2);
            AirSimSimpleEkfModel::evaluateAOld(&ANew, x_dotdotdot,u2);

            for (int i=0; i<17; i++){
                for (int j=0; j<17; j++){
                    std::cout << A(i,j) << "\t   ";
                }
                std::cout << '\n';
            }

            for (int i=0; i<17; i++){
                for (int j=0; j<17; j++){
                    std::cout << ANew(i,j) << "\t   ";
                }
                std::cout << '\n';
            }

            // QuaternionT q = VectorMath::toQuaternion(RealT pitch, RealT roll, RealT yaw);
            Quaternionr q = VectorMath::toQuaternion(10.0*M_PI/180, 10.0*M_PI/180, 10.0*M_PI/180);
            std::cout << q.w() << '\n';
            std::cout << q.x() << '\n';
            std::cout << q.y() << '\n';
            std::cout << q.z() << '\n';


            // VectorMath::Matrix17x17f A_finite;
            // VectorMath::Matrix17x17f A;
            // VectorMath::Matrix17x17f A_error;

            // // evaluateA(&A, x, u);
            // AirSimSimpleEkfModel::evaluateFiniteDifferenceA(&A_finite, x_dotdotdot,u2);
            // AirSimSimpleEkfModel::evaluateA(&A, x_dotdotdot,u2);

            // volatile bool isOK;
            // volatile float row[17]; 
            // volatile float column[17];
            // isOK = AirSimSimpleEkfModel::checkA(&A_error, &A, &A_finite, row, column);
            
            // std::cout << '\n';
            // for (int i=0; i<17; i++)
            // {
            //     for (int j=0; j<17; j++)
            //     {
            //         std::cout << A_finite(i,j) << "\t   ";
            //     }
            //     std::cout << '\n';
            // }
            // std::cout << '\n';
            // for (int i=0; i<17; i++)
            // {
            //     for (int j=0; j<17; j++)
            //     {
            //         std::cout << A(i,j) << "\t   ";
            //     }
            //     std::cout << '\n';
            // }
            // std::cout << '\n';
            // for (int i=0; i<17; i++)
            // {
            //     for (int j=0; j<17; j++)
            //     {
            //         std::cout << A_error(i,j) << "\t   ";
            //     }
            //     std::cout << '\n';
            // }

        }

    };
}
}
#endif