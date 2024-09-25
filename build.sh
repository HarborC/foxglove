#!/bin/bash

BASE_DIR=$(cd $(dirname $0);pwd)
cd ${BASE_DIR}

rm -r build
rm -r ./src/proto/foxglove/*.pb.cc
rm -r ./src/proto/foxglove/*.pb.h

mkdir build
cd build
cmake .. -D Protobuf_PROTOC_EXECUTABLE=/usr/local/bin/protoc
make -j8 
make install
cd ..

rm -r build
rm -r ./src/proto/foxglove/*.pb.cc
rm -r ./src/proto/foxglove/*.pb.h