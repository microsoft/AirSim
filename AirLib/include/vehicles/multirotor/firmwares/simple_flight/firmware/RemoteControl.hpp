#pragma once

#include <vector>
#include <cstdint>
#include "interfaces/IBoardClock.hpp"
#include "interfaces/IBoardInputPins.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/CommonStructs.hpp"

namespace simple_flight
{

class RemoteControl : public IGoal
    , public IUpdatable
{
public:
    RemoteControl(const Params* params, const IBoardClock* clock, const IBoardInputPins* board_inputs,
                  VehicleState* vehicle_state, IStateEstimator* state_estimator, ICommLink* comm_link)
        : params_(params), clock_(clock), board_inputs_(board_inputs), vehicle_state_(vehicle_state), state_estimator_(state_estimator), comm_link_(comm_link)
    {
    }

    virtual void reset() override
    {
        IUpdatable::reset();

        goal_ = Axis4r::zero();
        goal_mode_ = params_->default_goal_mode;
        allow_api_control_ = params_->rc.allow_api_always;
        last_rec_read_ = 0;
        last_angle_mode_ = std::numeric_limits<TReal>::min();
        request_duration_ = 0;
    }

    virtual void update() override
    {
        IUpdatable::update();

        uint64_t time = clock_->millis();

        //don't keep reading if not updated
        uint64_t dt = time - last_rec_read_;
        if (dt <= params_->rc.read_interval_ms)
            return;
        last_rec_read_ = time;

        //read channel values
        Axis4r channels;
        for (unsigned int axis = 0; axis < Axis4r::AxisCount(); ++axis) {
            channels[axis] = board_inputs_->isRcConnected() ? board_inputs_->readChannel(params_->rc.channels[axis])
                                                            : 0;
        }

        //set goal mode as per the switch position on RC
        updateGoalMode();
        updateAllowApiControl();

        //get any special action being requested by user such as arm/disarm
        RcRequestType rc_action = getActionRequest(channels);

        //state machine
        switch (vehicle_state_->getState()) {
        case VehicleStateType::Inactive:
            //comm_link_->log(std::string("State:\t ").append("Inactive state"));

            if (rc_action == RcRequestType::ArmRequest) {
                comm_link_->log(std::string("State:\t ").append("Inactive state, Arm request received"));
                request_duration_ += dt;

                if (request_duration_ > params_->rc.arm_duration) {
                    vehicle_state_->setState(VehicleStateType::BeingArmed);
                    request_duration_ = 0;
                }
            }
            //else ignore
            break;
        case VehicleStateType::BeingArmed:
            comm_link_->log(std::string("State:\t ").append("Being armed"));

            //start the motors
            goal_ = Axis4r::zero(); //neural activation while still being armed
            goal_.throttle() = params_->Params::min_armed_throttle();

            //we must wait until sticks are at neutral or we will have random behavior
            if (rc_action == RcRequestType::NeutralRequest) {
                request_duration_ += dt;

                if (request_duration_ > params_->rc.neutral_duration) {
                    //TODO: this code should be reused in OffboardApi
                    vehicle_state_->setState(VehicleStateType::Armed, state_estimator_->getHomeGeoPoint());
                    comm_link_->log(std::string("State:\t ").append("Armed"));
                    request_duration_ = 0;
                }
            }
            //else ignore
            break;
        case VehicleStateType::Armed:
            //unless disarm is being requested, set goal from stick position
            if (rc_action == RcRequestType::DisarmRequest) {
                comm_link_->log(std::string("State:\t ").append("Armed state, disarm request received"));
                request_duration_ += dt;

                if (request_duration_ > params_->rc.disarm_duration) {
                    vehicle_state_->setState(VehicleStateType::BeingDisarmed);
                    request_duration_ = 0;
                }
            }
            else {
                request_duration_ = 0; //if there was spurious disarm request
                updateGoal(channels);
            }
            break;
        case VehicleStateType::BeingDisarmed:
            comm_link_->log(std::string("State:\t ").append("Being state"));

            //TODO: this code should be reused in OffboardApi
            goal_.setAxis3(Axis3r::zero()); //neutral activation while being disarmed
            vehicle_state_->setState(VehicleStateType::Disarmed);
            request_duration_ = 0;

            break;
        case VehicleStateType::Disarmed:
            comm_link_->log(std::string("State:\t ").append("Disarmed"));

            goal_ = Axis4r::zero(); //neutral activation while being disarmed
            vehicle_state_->setState(VehicleStateType::Inactive);
            request_duration_ = 0;

            break;
        default:
            throw std::runtime_error("VehicleStateType has unknown value for RemoteControl::update()");
        }
    }

    virtual const Axis4r& getGoalValue() const override
    {
        return goal_;
    }

    virtual const GoalMode& getGoalMode() const override
    {
        return goal_mode_;
    }

    bool allowApiControl()
    {
        return allow_api_control_;
    }

    float getMotorOutput()
    {
        return board_inputs_->getAvgMotorOutput();
    }

private:
    enum class RcRequestType
    {
        None,
        ArmRequest,
        DisarmRequest,
        NeutralRequest
    };

    void updateGoalMode()
    {
        if (!board_inputs_->isRcConnected()) {
            //TODO: is it good idea to keep the last mode?
            //if (!goal_mode_.equals4(GoalMode::getUnknown()))
            //    goal_mode_ = GoalMode::getUnknown();

            //For angle as well as rate mode, keep only throttle
            goal_.setAxis3(Axis3r());

            return;
        }

        //set up RC mode as level or rate
        angle_mode_ = board_inputs_->readChannel(params_->rc.rate_level_mode_channel);
        if (last_angle_mode_ != angle_mode_) {
            //for 3 way switch, 1/3 value for each position
            if (angle_mode_ < params_->rc.max_angle_level_switch)
                goal_mode_ = GoalMode::getStandardAngleMode();
            else
                goal_mode_ = GoalMode::getAllRateMode();

            last_angle_mode_ = angle_mode_;
        }
    }

    void updateAllowApiControl()
    {
        bool allow = params_->rc.allow_api_always;
        allow |= board_inputs_->isRcConnected() ? board_inputs_->readChannel(params_->rc.allow_api_control_channel) > 0.1f
                                                : params_->rc.allow_api_when_disconnected;

        if (allow_api_control_ != allow)
            comm_link_->log(std::string("API control enabled:\t").append(std::to_string(allow_api_control_)));

        allow_api_control_ = allow;
    }

    void updateGoal(const Axis4r& channels)
    {
        //for 3 way switch, 1/3 value for each position
        if (angle_mode_ < params_->rc.max_angle_level_switch) {
            //we are in control-by-level mode
            goal_ = channels.colWiseMultiply4(params_->angle_level_pid.max_limit);
        }
        else { //we are in control-by-rate mode
            goal_ = channels.colWiseMultiply4(params_->angle_level_pid.max_limit);
        }

        //if throttle is too low then set all motors to same value as throttle because
        //otherwise values in pitch/roll/yaw would get clipped randomly and can produce random results
        //in other words: we can't do angling if throttle is too low
        if (channels.throttle() < params_->rc.min_angling_throttle)
            goal_.throttle() = params_->rc.min_angling_throttle;
    }

    static bool isInTolerance(TReal val, TReal tolerance, TReal center = TReal())
    {
        return val <= center + tolerance && val >= center - tolerance;
    }

    RcRequestType getActionRequest(const Axis4r& channels)
    {
        TReal tolerance = params_->rc.action_request_tolerance;
        TReal stick_min = 1 - tolerance;

        bool yaw_action_positive = channels.yaw() >= stick_min;
        bool yaw_action_negative = channels.yaw() <= -stick_min;
        bool throttle_action = channels.throttle() <= tolerance;

        bool roll_action_positive = channels.roll() >= stick_min;
        bool roll_action_negative = channels.roll() <= -stick_min;
        TReal normalized_pitch = (channels.pitch() + 1) / 2; //-1 to 1 --> 0 to 1
        bool pitch_action = normalized_pitch >= stick_min;

        if (yaw_action_positive && throttle_action && roll_action_negative && pitch_action)
            return RcRequestType::ArmRequest;
        else if (yaw_action_negative && throttle_action && roll_action_positive && pitch_action)
            return RcRequestType::DisarmRequest;
        else if (isInTolerance(channels.roll(), tolerance) && isInTolerance(channels.pitch(), tolerance) && isInTolerance(channels.yaw(), tolerance))
            return RcRequestType::NeutralRequest;
        else
            return RcRequestType::None;
    }

private:
    const Params* params_;
    const IBoardClock* clock_;
    const IBoardInputPins* board_inputs_;
    VehicleState* vehicle_state_;
    IStateEstimator* state_estimator_;
    ICommLink* comm_link_;

    Axis4r goal_;
    GoalMode goal_mode_;

    uint64_t last_rec_read_;
    TReal angle_mode_, last_angle_mode_;
    bool allow_api_control_;

    uint64_t request_duration_;
};

} //namespace