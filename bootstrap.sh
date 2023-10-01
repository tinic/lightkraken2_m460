#!/bin/sh
python -m venv .venv
source .venv/bin/activate
pip3 install FATtools
pip3 install npm
npm install yarn
git submodule init
git submodule update
cd vue3 
npm yarn install
