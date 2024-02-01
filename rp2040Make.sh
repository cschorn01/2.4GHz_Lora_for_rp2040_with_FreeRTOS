#!/bin/bash

echo rm -r build
rm -r build

echo mkdir build
mkdir build

echo cmake -D PICO_SDK_PATH=/Absolute/Path/To/pico-sdk -B ./build
cmake -D PICO_SDK_PATH=/Absolute/Path/To/pico-sdk -B ./build

echo make -C build
make -C build
