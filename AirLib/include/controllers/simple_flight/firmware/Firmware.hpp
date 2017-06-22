#pragma once

#include "Board.hpp"
#include "CommLink.hpp"
#include "Params.hpp"
#include "RemoteControl.hpp"

namespace simple_flight {

class Firmware {
public:
    Firmware(Board* board, CommLink* comm_link, const Params* params)
        : board_(board), comm_link_(comm_link), params_(params), rc_(board, params)
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
        rc_.update();
    }

private:
    //objects we use
    Board* board_;
    CommLink* comm_link_;

    const Params* params_;
    RemoteControl rc_;
};


} //namespace

