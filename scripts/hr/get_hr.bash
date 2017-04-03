#!/usr/bin/env bash

trap 'rm -f /tmp/hr' SIGINT SIGTERM EXIT

get_hr() {
    local url=https://raw.githubusercontent.com/hansonrobotics/HEAD/master/scripts/hr/hr
    local file=/tmp/hr
    curl -sLo $file ${url}
    bash_flag=$(file $file | grep bash | wc -l)
    if [[ $bash_flag != "1" ]]; then
        echo "Can't get hr"
        exit 1
    fi
    chmod +x /tmp/hr
    /tmp/hr install hr
    rm -f /tmp/hr
}

get_hr
