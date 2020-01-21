# TJArk Code Release 2019

## Introduction
### This is the open source code of TJArk framework used on NAO V6 robots in RoboCup 2019.

#### Architecture

- Bin: executable binaries of the code
- Config: configuration files
- Src: source code

#### Features

- Five threads are used: Upper-cognition, Lower-cognition, Motion, Behavior, Debug
- Optimized version of OpenCV


Some third-party libraries/codes are included in this code release, such as LolaConnector from Nao-Team HTWK, CABSL(iostream version) from B-Human, and OpenCV. 
 
Their license can be found in *3rdPartyLicense* folder. We highly respect and appreciate their contribution. 

## How to use on NAO v6

- Use installCrossToolChain.sh <ctc-file-name> to install cross toolchain

- Run ```path-to-toolchain/yocto-sdk/relocate_qitoolchain.sh```
  
  Caution: After this the toolchain can't be moved or copied to other devices anymore!
 
- Run ```create_crossconfig.py``` to create cross-config.cmake. This is the cmake cross chain file.
 
- Run ```ctc/yocto-sdk/relocate_sdk.py``` to relocate sdk.

- Run ```sh Make.sh``` to build this program.


If you are interested in other modules or our dataset or pre-trained weights, please contact us [TJArk.Official@gmail.com](tjark.official@gmail.com).
