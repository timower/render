#!/bin/sh
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=YES ..
cmake --build .
