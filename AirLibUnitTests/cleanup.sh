#!/bin/bash
cleanup(){
    in_file="$1"
    data_file="$2"
    log_file="$3"
    cat "$in_file" | sed -E '/^[^0-9]*$/d' > "$data_file"
    cat "$in_file" | sed -E '/^[0-9]+.*$/d' > "$log_file"
}
