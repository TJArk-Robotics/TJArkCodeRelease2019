#!/usr/bin/python

import os

# Get cunrrent dir path
homedir = os.getcwd()

# Create cross chain dir path
ctc_system_root_dir = homedir + '/ctc/yocto-sdk/sysroots/core2-32-sbr-linux'
tools_dir = homedir + '/ctc/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux'
index_cmake_sysroot = "set(CMAKE_SYSROOT " + ctc_system_root_dir + ')\n'
index_tools = "set(tools " + tools_dir + ')\n'

# create cross-config.cmake
with open('./cross-config.cmake', 'w') as f:
    f.write("set(CMAKE_SYSTEM_NAME Linux)\n")
    f.write("set(CMAKE_SYSTEM_PROCESSOR i686)\n")
    f.write(index_cmake_sysroot)
    f.write(index_tools)
    f.write('set(CMAKE_C_COMPILER "${tools}/i686-sbr-linux-gcc")\n')
    f.write('set(CMAKE_CXX_COMPILER "${tools}/i686-sbr-linux-g++")\n')
    f.write('set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)\n')
    f.write('set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)\n')
    f.write('set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)\n')
    f.write('set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)')

