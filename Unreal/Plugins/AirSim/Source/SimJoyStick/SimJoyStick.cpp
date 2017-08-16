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

#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include "unistd.h"

//implementation for unsupported OS
struct SimJoyStick::impl {
private:
    
    

    class JoystickEvent
    {
    public:
      /** Minimum value of axes range */
      static const short MIN_AXES_VALUE = -32768;

      /** Maximum value of axes range */
      static const short MAX_AXES_VALUE = 32767;
      
      /**
       * The timestamp of the event, in milliseconds.
       */
      unsigned int time;
      
      /**
       * The value associated with this joystick event.
       * For buttons this will be either 1 (down) or 0 (up).
       * For axes, this will range between MIN_AXES_VALUE and MAX_AXES_VALUE.
       */
      short value;
      
      /**
       * The event type.
       */
      unsigned char type;
      
      /**
       * The axis/button number.
       */
      unsigned char number;

      /**
       * Returns true if this event is the result of a button press.
       */
      bool isButton()
      {
        static constexpr unsigned char JS_EVENT_BUTTON = 0x01; // button pressed/released
        return (type & JS_EVENT_BUTTON) != 0;
      }

      /**
       * Returns true if this event is the result of an axis movement.
       */
      bool isAxis()
      {
        static constexpr unsigned char JS_EVENT_AXIS = 0x02; // joystick moved
        return (type & JS_EVENT_AXIS) != 0;
      }

      /**
       * Returns true if this event is part of the initial state obtained when
       * the joystick is first connected to.
       */
      bool isInitialState()
      {
        static constexpr unsigned char JS_EVENT_INIT = 0x80; // initial state of device
        return (type & JS_EVENT_INIT) != 0;
      }

      /**
       * The ostream inserter needs to be a friend so it can access the
       * internal data structures.
       */
      friend std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);
    };


public:
    ~impl()
    {
        if (fd_ >= 0)
            close(fd_);
    }

    void getJoyStickState(unsigned int index, SimJoyStick::State& state)
    {
        static constexpr bool blocking = false;

        if (index != last_index_) {
            if (fd_ >= 0)
                close(fd_);

            std::stringstream devicePath;
            devicePath << "/dev/input/js" << index;

            fd_ = open(devicePath.str().c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);

            last_index_ = index;
        }

        if (fd_ >= 0) {
            int bytes = read(fd_, &event_, sizeof(event_)); 

            if (bytes == -1 || bytes != sizeof(event_)) {
                // NOTE if this condition is not met, we're probably out of sync and this
                // Joystick instance is likely unusable
                
            }
            else {
                state.is_connected = true;

                if (event_.isButton()) {
                    if (event_.value == 0)
                        state.buttons &= ~(1 << event_.number);
                    else
                        state.buttons |= (1 << event_.number);
                }
                else if (event_.isAxis()) {
                    switch(event_.number) {
                    case 0: state.left_y = event_.value; break;
                    case 1: state.right_x = event_.value; break;
                    case 2: state.right_y = event_.value; break;
                    case 3: state.left_x = event_.value; break;
                    default: break;
                    }
                }
                //else ignore
            }
        }
        else
            state.is_connected = false;
    }

private:
    unsigned int last_index_ = -1;
    int fd_ = -1;
    JoystickEvent event_;
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
