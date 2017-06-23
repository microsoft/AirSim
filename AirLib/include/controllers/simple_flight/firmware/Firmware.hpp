#pragma once

#include "Board.hpp"
#include "CommLink.hpp"
#include "Params.hpp"
#include "RemoteControl.hpp"
#include "Stabilizer.hpp"
#include "CommonStructs.hpp"
#include "Mixer.hpp"

namespace simple_flight {

class Firmware {
public:
    Firmware(Board* board, CommLink* comm_link, const Params* params)
        : board_(board), comm_link_(comm_link), params_(params), 
          rc_(board, params), mixer_(params)
    {
    }

    void reset()
    {
        board_->reset();
        comm_link_->reset();
        rc_.reset();
    }

    void update()
    {
        board_->update();
        comm_link_->update();

        //get latest from RC
        rc_.update();

        ////use RC values as set point for stabilizer
        //const auto& desired_controls = rc_.getDesiredRates();
        //stabilizer_.setDesiredRates(desired_controls);

        ////update measured controls
        //board_->readGyro(gyro_readout);
        //measured_controls_.pitch = gyro_readout

        //const auto& output_controls = stabilizer_.getOutputControls();
        //mixer_.getMotorOutput(output_controls, );
    }

private:
    //objects we use
    Board* board_;
    CommLink* comm_link_;

    float gyro_readout[3];
    Controls measured_controls_;

    const Params* params_;
    RemoteControl rc_;
    Stabilizer stabilizer_;
    Mixer mixer_;
};


} //namespace

