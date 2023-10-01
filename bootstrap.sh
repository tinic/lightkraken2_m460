#!/bin/sh
pip3 install FATtools
pip3 install npm
npm install yarn
git submodule init
git submodule update
cd vue3 
npm yarn install
