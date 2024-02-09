# OGC.Engineering
### ogc-dev-rp2040 - Exploration of the RP2040 on a Raspberry Pi Pico W
developer contact - dustin ( at ) ogc.engineering

--

## Repository Information
* Purpose
    - Exploration of MicroPython and C based development on the RP2040 provided by the Raspberry Pi Pico W
* Development Steps
    - Clone
```
git clone https://github.com/OGC-dustin/ogc-dev-rp2040.git
git checkout <branch of interest>
```
* Branches
    - Micro Python based development
        - master-p - master branch for Micro Python based development
        - rc-p - release candidate branch for testing Micro Python based developments
        - dev-p - development branch for Micro Python based developments
        - various feature branches as needed
    - C based development
        - master-c - master branch for C based development
        - rc-c - release candidate branch for testing C based developments
        - dev-c - development branch for C based developments
        - various feature branches as needed
* Filesystem Layout - Layered Development
    * documentation - it's documentation... not much else to say about it
    * projects - example projects showing one or more deployments, documentation, etc. that fully define a project
    * deployments - example deployments showing collections of software, firmware, and a hardware needed to create a deployable image
    * software
        * applications - applications quide device operations
        * libraries - hardware independent support logic
    * firmware - hardware dependent source files to meet software interface
        * hal - abstraction of hardware and firmware to meet the software interface
        * drivers - hardware specific support logic
        * csp - manufacturer chip support package(s), all the base level functionality buiilding blocks
    * hardware - hardware dependent defintions of capabilities and abstraction from csp defined pin/port names

## Hardware
* Development on the Raspberry Pi Pico W and DeskPi Pico Mate
    - https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html
    - https://wiki.deskpi.com/picomate/

## Firmware
* Micro Python
    - https://www.raspberrypi.com/documentation/microcontrollers/micropython.html
    - Make sure the firmware is updated for wireless support
        - https://projects.raspberrypi.org/en/projects/get-started-pico-w/1
* C SDK
    - https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html

## Development Environment
* Micro Python
    - Thonny - https://thonny.org/
* C SDK
    - CLI CMAKE project