#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
cd ${BASE_DIR}

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
make install
cd ..
rm -r build