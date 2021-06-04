#ifndef msr_AirLibUnitTests_TestBase_hpp
#define msr_AirLibUnitTests_TestBase_hpp

#include <string>
#include <exception>
#include "common/common_utils/Utils.hpp"

namespace msr
{
namespace airlib
{

    class TestBase
    {
    public:
        virtual ~TestBase() = default;
        virtual void run() = 0;

        void testAssert(double lhs, double rhs, const std::string& message)
        {
            testAssert(lhs == rhs, message);
        }

        void testAssert(bool condition, const std::string& message)
        {
            if (!condition) {
                common_utils::Utils::DebugBreak();
                throw std::runtime_error(message.c_str());
            }
        }
    };
}
}
#endif