#!/usr/bin/env bash

set -e
version=0.1.17
curl -sLo /tmp/head-hr_${version}_amd64.deb https://github.com/hansonrobotics/hrtool/releases/download/v${version}/head-hr_${version}_amd64.deb
if [[ -f /usr/local/bin/hr-base ]]; then
    sudo rm /usr/local/bin/hr
    sudo rm /usr/local/bin/hr-base
    sudo rm /usr/local/bin/hr-ext
fi
sudo dpkg -i /tmp/head-hr_${version}_amd64.deb
rm /tmp/head-hr_${version}_amd64.deb
