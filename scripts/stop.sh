#!/usr/bin/env bash

BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $BASEDIR/common.sh
PIDFILE=/tmp/hr.pid

if [[ -f $PIDFILE ]]; then
    pids=$(<"$PIDFILE")
    kill $pids
    rm $PIDFILE
fi

ns=$NAME

supervisorctl -c $BASEDIR/supervisord.conf shutdown >/dev/null
sleep 0.5

if [[ $(tmux ls 2>/dev/null) ]]; then
    # TODO: don't send command when stopping pololu
    # cd $HR_WORKSPACE/HEAD/src/tools/src/testing_tools && python live_off.py && cd -
    # rostopic pub -1 /behavior_switch std_msgs/String "btree_off"
    # rosservice call /${ns}/head_pau_mux/select /${ns}/no_pau
    # rosservice call /${ns}/neck_pau_mux/select /${ns}/cmd_neck_pau
    # rostopic pub -1 /${ns}/point_head basic_head_api/PointHead 0 0 0
    # rostopic pub -1 /${ns}/make_face_expr basic_head_api/MakeFaceExpr Neutral 0
    curl -G localhost:8001/v1.1/dump_history >/dev/null 2>&1
    # bash $HR_WORKSPACE/hr-solr/stop.sh
    tmux kill-session
fi

check_ports 1024 4000 8000 8001 9090 10001
info "Stopped $?"
