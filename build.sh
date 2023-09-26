#!/bin/sh

set -e

mkdir -p build

build_type="Ninja"

cd build
rm -rf lightkraken2_debug*
mkdir -p lightkraken2_debug
cd lightkraken2_debug
cmake -G "$build_type" -DCMAKE_SYSTEM_NAME=Generic -DCMAKE_TOOLCHAIN_FILE=../../cmake/arm-gcc-toolchain.cmake -DCMAKE_BUILD_TYPE=Debug ../..
cmake --build .
cd ..

cd build
rm -rf lightkraken2_release*
mkdir -p lightkraken2_release
cd lightkraken2_release
cmake -G "$build_type" -DCMAKE_SYSTEM_NAME=Generic -DCMAKE_TOOLCHAIN_FILE=../../cmake/arm-gcc-toolchain.cmake -DCMAKE_BUILD_TYPE=Release  ../..
cmake --build .
cd ..
