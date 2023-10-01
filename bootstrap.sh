#!/bin/sh
git submodule init
git submodule update
cd vue3 
yarn install
cd ..
