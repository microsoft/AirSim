#!/bin/bash
cleanup(){
    debug_flag="$1"
    # in_file="$1"
    # data_file="$2"
    # log_file="$3"
    # cat "$in_file" | sed -E '/^[^0-9]*$/d' > "$data_file"
    # cat "$in_file" | sed -E '/^[0-9]+.*$/d' > "$log_file"
    if [[ $debug_flag == 'd' ]]; then
        cat "log.txt" | sed -E '/^[^0-9]*$/d' > "data.csv"
        cat "log.txt" | sed -E '/^[0-9]+.*$/d' > "other.txt"
    else
        cat "../build_release/output/bin/log.txt" | sed -E '/^[^0-9]*$/d' > "data.csv"
        cat "../build_release/output/bin/log.txt" | sed -E '/^[0-9]+.*$/d' > "other.txt"
    fi
    
}
