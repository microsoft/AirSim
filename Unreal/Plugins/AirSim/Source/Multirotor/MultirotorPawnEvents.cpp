#include "MultirotorPawnEvents.h"

MultirotorPawnEvents::ActuatorsSignal& MultirotorPawnEvents::getActuatorSignal()
{
    return actuator_signal_;
}