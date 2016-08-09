#!/usr/bin/env bash

set -e

if [[ $# != 1 ]];then
    echo "Usage: bash $0 <PREFIX>"
    exit 1
fi

BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

HR_PREFIX=${1}
HR_CACHE=~/.hr/cache
HR_BUILD_ENVFILE=~/.hr/build_env.sh
MANYEARSLIB_PREFIX=$HR_PREFIX/manyears

set_env() {
cat <<EOF >$HR_BUILD_ENVFILE
export MANYEARSLIB_PREFIX=$HR_PREFIX/manyears
EOF
}

wget_cache() {
    local url=$1
    local ofile=${2-${url##*/}}
    [[ -f ${HR_CACHE}/${ofile} ]] || wget ${url} -O ${HR_CACHE}/${ofile}
}

install_marytts_deps() {
    wget_cache https://github.com/marytts/marytts/releases/download/v5.1.2/marytts-5.1.2.zip
    unzip -od $HR_PREFIX $HR_CACHE/marytts-5.1.2.zip
}

install_manyears_deps() {
    wget_cache https://github.com/hansonrobotics/manyears-C/archive/v1.0.1.tar.gz manyears.v1.0.1.tar.gz
    local manyears_dir=/tmp/manyears-C-1.0.1
    rm -rf ${manyears_dir}
    tar zxf $HR_CACHE/manyears.v1.0.1.tar.gz -C /tmp
    cd ${manyears_dir} && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=${MANYEARSLIB_PREFIX} .. && make && make install
    rm -rf ${manyears_dir}
}

mkdir -p $HR_CACHE
mkdir -p $HR_PREFIX

set_env
pip2 install -t $HR_PREFIX/lib/python2.7/dist-packages -r $BASEDIR/requirements
install_marytts_deps
install_manyears_deps
