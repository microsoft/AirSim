#include "PawnEvents.h"

PawnEvents::CollisionSignal& PawnEvents::getCollisionSignal()
{
    return collision_signal_;
}

PawnEvents::PawnTickSignal& PawnEvents::getPawnTickSignal()
{
    return pawn_tick_signal_;

}