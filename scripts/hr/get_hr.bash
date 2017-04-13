#!/usr/bin/env bash

version=0.1.8
curl -sLo /tmp/head-hr_${version}_amd64.deb https://github.com/hansonrobotics/hrtool/releases/download/v${version}/head-hr_${version}_amd64.deb
sudo dpkg -i /tmp/head-hr_${version}_amd64.deb
