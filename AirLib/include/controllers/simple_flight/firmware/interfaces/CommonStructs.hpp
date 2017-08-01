#pragma once

namespace simple_flight {

typedef float TReal;
static constexpr unsigned int kAxisCount = 3;

template<typename T>
class Axis3 {
public:
    Axis3(const T& x_val = T(), const T& y_val = T(), const T& z_val = T())
        : vals_ {x_val, y_val, z_val}
    {
    }

    //access by index
    T& operator[] (unsigned int index)
    {
        return vals_[index];
    }
    const T& operator[] (unsigned int index) const
    {
        return vals_[index];
    }

    //access as axis
    const T& x() const { return vals_[0]; }
    const T& y() const { return vals_[1]; }
    const T& z() const { return vals_[2]; }
    T& x() { return vals_[0]; }
    T& y() { return vals_[1]; }
    T& z() { return vals_[2]; }

    //access as angles
    const T& roll() const { return vals_[0]; }
    const T& pitch() const { return vals_[1]; }
    const T& yaw() const { return vals_[2]; }
    T& roll() { return vals_[0]; }
    T& pitch() { return vals_[1]; }
    T& yaw() { return vals_[2]; }

private:
    T vals_[kAxisCount];
};
typedef Axis3<TReal> Axis3r;

template<typename T>
struct Axis4 {
    T throttle = 0;
    Axis3<T> axis3;

    Axis4(const T& throttle_val = T(), const T& x_val = T(), const T& y_val = T(), const T& z_val = T())
        : throttle(throttle_val), axis(x_val, y_val, z_val)
    {
    }
};
typedef Axis4<TReal> Axis4r;


enum class GoalModeType {
    AngleLevel,
    AngleRate,
    VelocityWorld,
    PositionWorld,
    Unknown
};

class GoalMode : public Axis3<GoalModeType> {
public:
    GoalMode(GoalModeType x_val = GoalModeType::AngleLevel, GoalModeType y_val = GoalModeType::AngleLevel, 
        GoalModeType z_val = GoalModeType::AngleRate)
    : Axis3<GoalModeType>(x_val, y_val, z_val)    
    {
    }

    static GoalMode getStandardAngleMode()
    {
        return GoalMode();
    }

    static GoalMode getAllRateMode()
    {
        return GoalMode(GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::AngleRate);
    }

    static GoalMode getUnknown()
    {
        return GoalMode(GoalModeType::Unknown, GoalModeType::Unknown, GoalModeType::Unknown);
    }
};

} //namespace