#!/bin/bash

catkin_make -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_CXX_FLAGS="-std=c++14" -DCATKIN_BLACKLIST_PACKAGES="gpar_nanook"
