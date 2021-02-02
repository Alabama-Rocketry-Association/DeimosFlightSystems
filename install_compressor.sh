#!/usr/bin/env bash

git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
vcpkg install lz4
python3 -m pip install lz4
echo "FINISHED!"