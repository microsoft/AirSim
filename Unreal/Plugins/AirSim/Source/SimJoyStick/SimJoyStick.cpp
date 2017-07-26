#include "SimJoyStick.h"

#if defined _WIN32 || defined _WIN64

#include "common/common_utils/StrictMode.hpp"

STRICT_MODE_OFF
//below headers are required for using windows.h types in Unreal
#include "Windows/WindowsHWrapper.h"
#include <XInput.h>

//Below is old way of doing it?
//#include "AllowWindowsPlatformTypes.h"
//#define WIN32_LEAN_AND_MEAN
//#define NOMINMAX
//#include <windows.h>
//#include <XInput.h>
//#include "HideWindowsPlatformTypes.h"
STRICT_MODE_ON

struct SimJoyStick::impl {
public:
    void getJoyStickState(unsigned int index, SimJoyStick::State& state)
    {
        if (!initialized_success_ || index >= kMaxControllers) {
            state.is_connected = false;
            return;
        }

        // Simply get the state of the controller from XInput.
        auto dwResult = XInputGetState(index, &controllers_[index].state);

        if( dwResult == ERROR_SUCCESS )
            controllers_[index].bConnected = true;
        else
            controllers_[index].bConnected = false;

        state.is_connected = controllers_[index].bConnected;

        if (state.is_connected) {
            state.left_x = controllers_[index].state.Gamepad.sThumbLX;
            state.left_y = controllers_[index].state.Gamepad.sThumbLY;
            state.right_x = controllers_[index].state.Gamepad.sThumbRX;
            state.right_y = controllers_[index].state.Gamepad.sThumbRY;

            state.buttons = controllers_[index].state.Gamepad.wButtons;
            state.left_trigger = controllers_[index].state.Gamepad.bLeftTrigger != 0;
            state.right_trigger = controllers_[index].state.Gamepad.bRightTrigger != 0;
        }
        //else don't waste time
    }

private:
    struct ControllerState
    {
        XINPUT_STATE state;
        bool bConnected;
    };

    static constexpr unsigned int kMaxControllers = 4;
    ControllerState controllers_[kMaxControllers];
};

#else

//implementation for unsupported OS
struct SimJoyStick::impl {
public:
    void getJoyStickState(unsigned int index, SimJoyStick::State& state)
    {
        //not implemented
        state.is_connected = false;
    }
};

#endif

bool SimJoyStick::initialized_success_ = false;

void SimJoyStick::setInitializedSuccess(bool success)
{
    initialized_success_ = success;
}
bool SimJoyStick::isInitializedSuccess()
{
    return initialized_success_;
}

SimJoyStick::SimJoyStick()
{
    pimpl_.reset(new impl());
}
SimJoyStick::~SimJoyStick()
{
    //required for pimpl
}

void SimJoyStick::getJoyStickState(unsigned int index, SimJoyStick::State& state)
{
    pimpl_->getJoyStickState(index, state);
}
