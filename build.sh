#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
cd ${BASE_DIR}

mkdir build
cd build
cmake .. -D Protobuf_PROTOC_EXECUTABLE=/usr/bin/protoc
make -j8
make install
cd ..
rm -r build