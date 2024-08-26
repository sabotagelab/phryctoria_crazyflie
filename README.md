# Phryctoria for Crazyflies
This is an application for Crazyflie 2.1 (*Crazyflie*) by Bitcraze, implementing a decentralized algorithm published in the paper "Decentralized Predicate Detection over Partially Synchronous Continuous-Time Signals".

## Background and Prerequisites
This tutorial assumes that the user has knowledge of operating Crazyflies and their development environment including firmware and python client. 

### Hardware
- Each Crazyflie in the swarm is equipped with a Micro SD card deck and a Lighthouse positioning deck. The indoor testbed with lighthouse positioning system is used. For more information about lighthouse positioning system, please refer to the [link](https://www.bitcraze.io/documentation/system/positioning/ligthouse-positioning-system/). Other positioning systems can be used, but it would require modifying codes manually.
- Crazyradio 2.0 (for flashing)

### Software
Install the Crazyflie firmware and the Python client. For users who are relatively new to this platform, please read documenation carefully.
- [Firmware](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/). For your information, the application worked well with the firmware version of 2023.11.
- [Python client](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/)

### Framework to operate a decentralized swarm of Crazyflies
The framework to operate a decentralized swarm of Crazyflies is from a [demo](https://github.com/bitcraze/crazyflie-firmware-experimental/tree/icra-2023/examples/demos/decentralized_swarm) by Bitcraze. Please check README carefully. 
- Please make sure that URIs of pilots are from `E7E7E7E701` to `E7E7E7E709` depending on the size of the swarm, and the one for the sniffer is `E7E7E7E700`.

## Install
- Clone this repository under the `example` folder of the firmware.

## Build
- Building and flashing should be repeated twice; one for pilots and one for the sniffer. The order does not matter.
- You can compile the application by `make` command.
- Make sure the right application is complied, which is controlled by the flags in `choose_app.h`.
- It is needed to specify the number of lighthouse beacons in `app-config`.

## Flash
- Pilots: Run `cload-all.sh`. Modify the script depending on the size of swarm.
- Sniffer: Execute `CLOAD_CMDS="-w E7E7E7E700" make cload` 
- Make sure the appropriate Crazyflie(s) are turned on. 

## Logging
- Pilots: Copy files in `usd/configs/process` to the micro SD card for each Crazyflie.
- Sniffer: Copy files in `usd_configs/sniffer` to the micro SD card.
- Increase the maximum number of log events to 40 by modifying [this](https://github.com/bitcraze/crazyflie-firmware/blob/ac565120e6ca2858d2e4a73f204775502e2d7233/src/deck/drivers/src/usddeck.c#L94).
- See [documentation](https://www.bitcraze.io/products/micro-sd-card-deck/) about the logging framework.

## Change the size of swarm
Changing the size of swarm requires to modify the code and run the previous steps (building and flashing) again. 
- Change `NUM_AGENTS` in `phryctoria.h`.
- Modify the name of config file in the deck [firmware](https://github.com/bitcraze/crazyflie-firmware/blob/ac565120e6ca2858d2e4a73f204775502e2d7233/src/deck/drivers/src/usddeck.c#L569) appropriately.