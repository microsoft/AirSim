#ifndef common_utils_Signal_hpp
#define common_utils_Signal_hpp

#include <functional>
#include <map>

// A signal object may call multiple slots with the
// same signature. You can connect functions to the signal
// which will be called when the emit() method on the
// signal object is invoked. Any argument passed to emit()
// will be passed to the given functions.

/*
//USAGE:

#include "Signal.hpp"
#include <iostream>

int main() {

// create new signal
Signal<std::string, int> signal;

// attach a slot
signal.connect([](std::string arg1, int arg2) {
std::cout << arg1 << " " << arg2 << std::endl;
});

signal.emit("The answer:", 42);

return 0;
}

*/

namespace common_utils
{

template <typename... Args>
class Signal
{

public:
    Signal()
        : current_id_(0) {}

    // Currently deleted since unused and there shouldn't be any need for this
    Signal(Signal const& other) = delete;
    Signal& operator=(Signal const& other) = delete;

    // connects a member function to this Signal
    template <typename T>
    int connect_member(T* inst, void (T::*func)(Args...))
    {
        return connect([=](Args... args) {
            (inst->*func)(args...);
        });
    }

    // connects a const member function to this Signal
    template <typename T>
    int connect_member(T* inst, void (T::*func)(Args...) const)
    {
        return connect([=](Args... args) {
            (inst->*func)(args...);
        });
    }

    // connects a std::function to the signal. The returned
    // value can be used to disconnect the function again
    int connect(std::function<void(Args...)> const& slot) const
    {
        slots_.insert(std::make_pair(++current_id_, slot));
        return current_id_;
    }

    // disconnects a previously connected function
    void disconnect(int id) const
    {
        slots_.erase(id);
    }

    // disconnects all previously connected functions
    void disconnect_all() const
    {
        slots_.clear();
    }

    // calls all connected functions
    void emit(Args... p)
    {
        for (auto it = slots_.begin(); it != slots_.end();) {
            // Increment here so that the entry can be erased from inside the method as well
            (it++)->second(p...);
        }
    }

private:
    mutable std::map<int, std::function<void(Args...)>> slots_;
    mutable int current_id_;
};
}

#endif /* common_utils_Signal_hpp */
