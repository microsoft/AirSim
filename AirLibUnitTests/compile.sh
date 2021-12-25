#!/bin/bash
compile(){
    debug_flag="$1"
    if [[ $debug_flag == 'd' ]]; then
        cd ../build_debug/AirLibUnitTests
        make
        cd ../../AirLibUnitTests
        echo "Debug mode, built only"
    else
        cd ../build_release/AirLibUnitTests
        make
        cd ../output/bin
        ./AirLibUnitTests
        cd ../../../AirLibUnitTests
        echo "Release mode, built and run"
    fi
}

