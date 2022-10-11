#!/usr/bin/env bash

set -ex

# use home directory by default, passed
# directory otherwise
if [ $# == 0 ]; then
   working_dir=$HOME
else
   working_dir=$1
fi

echo "Unpacking dxc for Linux in directory ${working_dir}"
cd $working_dir
wget https://drive.google.com/uc\?export\=download\&id\=1dF6cX5q-3tB3e5zVcZIL_Fa1W8-WRpOq -O dxc-artifacts.tar.gz
tar xf dxc-artifacts.tar.gz
