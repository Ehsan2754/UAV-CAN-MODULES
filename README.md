# UAV-CAN-MODULES

## Description
This repository contains the CAN-nodes I designed for  [UAV-CAN](https://uavcan.org) platform. 
Each node project in repository contains X categories of content as following :
* Schematics, PCB (Altium Designer) and BoM.
* Fabrication and GERBER files.
* 3D model of the node (```.STEP```)
* Frimware (```Embedded C + assembly```)

## Prerequesties
* GNU ARM
* STM32Fxxx HAL
* Altium Designer
* STMCUBEMX
* UAV-CAN

## [Node](https://github.com/Ehsan2754/UAV-CAN-MODULES/tree/main/PATTERN-GENERATOR-NODE) Structures
Each node directory can have the following structure:
![Project structure](https://i.ibb.co/jkwgZJt/Screenshot-2021-08-18-113708.png)
1. Directories named with ```FW``` keyword.  > these directories contain the framework following ```STM32CUBEMX``` and [```PlatformIO```](https://platformio.org/) structure under the STM32 hardware abstraction layer (HAL). Besides, It contains ```.elf``` and ```.bin``` frimware. 
2. Directories named ```Output``` contain fabrication and GERBER files, 3D model of the node (```.STEP```) and the bill of materials (BoM)
3. The node directory contains Schematics, PCB  and Altium Designer project file```.prjpcb ``` (Altium Designer)

