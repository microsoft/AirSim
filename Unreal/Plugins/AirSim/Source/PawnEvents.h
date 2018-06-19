#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "common/common_utils/Signal.hpp"


class PawnEvents {
public: //types
    typedef common_utils::Signal<UPrimitiveComponent*, AActor*, UPrimitiveComponent*, bool, FVector,
        FVector, FVector, const FHitResult&> CollisionSignal;
    typedef common_utils::Signal<float> PawnTickSignal;

public:
    CollisionSignal& getCollisionSignal();
    PawnTickSignal& getPawnTickSignal();

private:
    CollisionSignal collision_signal_;
    PawnTickSignal pawn_tick_signal_;
};
