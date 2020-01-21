#!/bin/sh
if [ ! -d "./Build" ]; then
    echo "create directory build"
    mkdir Build
fi
echo "Build exist"
if [ ! -d "./cross-config.cmake" ]; then
    python create_crossconfig.py
fi

cd ./Build
# rm -r ./*
# sync
echo "configure project"
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cross-config.cmake
echo "make project"
make -j4
echo "Make Done"
