#pragma once

#include "IUpdatable.hpp"
#include "IBoardClock.hpp"
#include "IBoardInputPins.hpp"
#include "IBoardOutputPins.hpp"
#include "IBoardSensors.hpp"

namespace simple_flight
{

class IBoard : public IUpdatable
    , public IBoardClock
    , public IBoardInputPins
    , public IBoardOutputPins
    , public IBoardSensors
{
};

} //namespace