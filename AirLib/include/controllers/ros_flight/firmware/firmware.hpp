#pragma once

#if defined(_MSC_VER)
__pragma(warning(push))	
__pragma(warning( disable : 4268))					  
#endif

#include "estimator.hpp"
#include "mode.hpp"
#include "param.hpp"
#include "sensors.hpp"
#include "controller.hpp"
#include "mixer.hpp"
#include "rc.hpp"
#include "board.hpp"
#include "commlink.hpp"
#include "commonstate.hpp"


namespace ros_flight {

class Firmware {
public:
    Firmware(Board* _board, CommLink* _comm_link);

    void setup();
    void loop();

    //getters
    Board* get_board() { return board; }
    CommLink* get_commLink() { return comm_link; }
    Estimator* get_estimator() { return &estimator; }
    Sensors* get_sensors() { return &sensors; }
    Mux* get_mux() { return &mux; }
    Mixer* get_mixer() { return &mixer; }
    Controller* get_controller() { return &controller; }
    RC* get_rc() { return &rc; }
    Mode* get_mode() { return &mode; }

private:
    //params and shared state
    Params params;
    CommonState common_state;

    //objects we use
    Board* board;
    CommLink* comm_link;
    Estimator estimator;
    Sensors sensors;
    Mux mux;
    Mixer mixer;
    Controller controller;
    RC rc;
    Mode mode;
    
    //variables to real IMU
    vector_t accel, gyro;
    uint64_t imu_time;
};


/************************************************** Implementation ***************************************************************/

Firmware::Firmware(Board* _board, CommLink* _comm_link)
    : board(_board), comm_link(_comm_link)
{
}

void Firmware::setup()
{
    board->init();

    //initialize parameters source such as EPROM
    params.init(board, comm_link);

    // Initialize communication stack such as MavLink
    comm_link->init();


    // Initialize Estimator
    // mat_exp <- greater accuracy, but adds ~90 us
    // quadratic_integration <- some additional accuracy, adds ~20 us
    // accelerometer correction <- if using angle mode, this is required, adds ~70 us
    estimator.init(&params, false, false, true);

    // Initialize Sensors
    sensors.init(&common_state, board, &estimator, &params, comm_link);

    mux.init(&common_state, board, &params);

    // Initialize Motor Mixing
    mixer.init(&common_state, board, &params);

    controller.init(&common_state, board, &mux, &mixer, &estimator, &params, comm_link);

    rc.init(&common_state, board, &mux, &params, comm_link);

    mode.init(board, comm_link, &common_state, &sensors, &rc, &params);

    comm_link->log_message("ROSFlight firmware initialized", 0);
}

void Firmware::loop()
{
    /*********************/
    /***  Control Loop ***/
    /*********************/
    if (sensors.update_sensors()) // 595 | 591 | 590 us
    {
        // If I have new IMU data, then perform control
        sensors.get_imu_measurements(accel, gyro, imu_time);
        estimator.run_estimator(accel, gyro, imu_time); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)
        controller.run_controller(); // 278 | 271
        mixer.mix_output(); // 16 | 13 us
    }

    /*********************/
    /***  Post-Process ***/
    /*********************/
    //Let communication stack send and recieve messages
    comm_link->update(); // 165 | 27 | 2

                         // update the armed_states, an internal timer runs this at a fixed rate
    mode.check_mode(board->micros()); // 108 | 1 | 1

                                      // get RC, an internal timer runs this every 20 ms (50 Hz)
    rc.receive_rc(board->micros()); // 42 | 2 | 1

                                    // update commands (internal logic tells whether or not we should do anything or not)
    mux.mux_inputs(); // 6 | 1 | 1
}



} //namespace


#if defined(_MSC_VER)
__pragma(warning(pop))					  
#endif
