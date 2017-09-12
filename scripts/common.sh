COLOR_INFO='\033[32m'
COLOR_WARN='\033[33m'
COLOR_ERROR='\033[31m'
COLOR_RESET='\033[0m'
info() {
    printf "${COLOR_INFO}[INFO] ${1}${COLOR_RESET}\n"
}
warn() {
    printf "${COLOR_WARN}[WARN] ${1}${COLOR_RESET}\n"
}
error() {
    printf "${COLOR_ERROR}[ERROR] ${1}${COLOR_RESET}\n"
}

check_disk_usage() {
    local available
    available=$(df / | grep / | awk '{print $4}')
    if (( $available < 3000000 )); then
        if (( $available < 1000000 )); then
            error "Extreamly low disk space. Please add more space"
            return 1
        else
            warn "Low disk space"
        fi
    fi
}

check_ports() {
    local ports=$@
    local process
    local ret
    for port in ${ports[@]}; do
        process=$(netstat -tuplen 2>/dev/null|awk '{print $4 " " $9}'|grep ":$port"|cut -d' ' -f2|cut -d/ -f1|sort -u)
        if [[ ! -z $process ]]; then
            error "Process $process is using port $port"
            ret=1
        fi
    done
    return $ret
}
