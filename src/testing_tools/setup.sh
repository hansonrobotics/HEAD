#!/usr/bin/env bash

CWD=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
TARGET=$CWD/../../devel/lib/python2.7/dist-packages/
if [[ ! -d $TARGET ]]; then
    mkdir -p $TARGET
fi
rm -rf $TARGET/testing_tools*
pip install -t $TARGET $CWD
