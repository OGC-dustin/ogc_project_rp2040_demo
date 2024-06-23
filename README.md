# OGC.Engineering
### ogc_project_rp2040_demo - Exploration of the RP2040 on a Raspberry Pi Pico W
developer contact - dustin ( at ) ogc.engineering

---

## Repository Information
* Purpose
    - Exploration of MicroPython and C based development on the RP2040 provided by the Raspberry Pi Pico W
* Development Steps
    - Clone
```
git clone https://github.com/OGC-dustin/ogc_project_rp2040_demo.git
git checkout <branch of interest>
git submodule update --init
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
    * deployment(s) - collections of software, firmware, and a hardware needed to create a deployable image
        * software
            * applications - applications quide device operations
            * libraries - hardware independent support logic
        * firmware - hardware dependent source files to meet software interface
            * hal - abstraction of hardware and firmware to meet the software interface
                * "hal.h" - generic redirection to hal layers
                * "ogc_fw_hal_rp_2040_button" - Button ( GPIO )
                * "ogc_fw_hal_rp_2040_encoder" - Rotary Encoder ( GPIO )
                * "ogc_fw_hal_rp_2040_buzzer" - Buzzer ( Generic PWM )
            * drivers - hardware specific support logic
                * "ogc_fw_drv_zts6531s" - Microphone [ ZTS6531S ] ( PDM )
                * "ogc_fw_drv_ws2812" - RGB Lights [ WS2812 ] ( GPIO )
                * "ogc_fw_drv_ssd1315" - OLED Display 128x64 [ SSD1315 ] ( I2C0, addr 0x3C )
                * "ogc_fw_drv_lsm6ds3tr_c" - Gyroscope Sensor [ LSM6DS3TR-C ] ( I2C1, addr 0x6A )
                * "ogc_fw_drv_mmc5603nj" - 3-Axis Magnetometer Sensor [ MMC5603NJ ] ( I2C1, addr 0x30 )
                * "ogc_fw_drv_ltr_381rgb_01" - Photosensitive Sensor [ LTR-381RGB-01 ] ( I2C1, addr 0x53 )
                * "ogc_fw_drv_sht30_dis" - Temperature and Humidity Sensor [ SHT30-DIS ] ( I2C1, addr 0x44 )
                * "ogc_fw_drv_as312" - PIR Sensor [ AS312 ] ( GPIO )
                * "ogc_fw_drv_infineon_43439" - 2.4GHz 802.11n WiFi/BLE
            * csp - manufacturer chip support package(s), all the base level functionality buiilding blocks
                * "https://github.com/raspberrypi/pico-sdk"
                    * https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
        * hardware - hardware dependent defintions of capabilities and abstraction from csp defined pin/port names
            * "hw_description.h" - generic redirection to hw layers
            * "ogc_hw_rp_2040_pico_w" - Raspberry Pi Pico-W Processor Board definitions
            * "ogc_hw_deskpi_picomate" - Raspberry Pi Pico Development Baseboard definitions

## Hardware

### Development on the Raspberry Pi Pico W and DeskPi Pico Mate
- https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html
- https://wiki.deskpi.com/picomate/

### Power options
```
Micro-USB B port for Power and Programming
VBAT-IN ( 1.8 - 5.5V )
```

### Pin Resources
```
RP2040 ( 2MB Flash, 3 pin Arm Serial Wire Debug ( SWD ) )
      UART0_TX, I2C0_SDA, SPI0_RX, GP0 - [ 01 ][ 40 ] - VBUS
     UART0_RX, I2C0_SCL, SPI0_CSn, GP1 - [ 02 ][ 39 ] - VSYS
                                   GND - [ 03 ][ 38 ] - GND
               I2C1_SDA, SPI0_SCK, GP2 - [ 04 ][ 37 ] - 3V3_EN
                I2C1_SCL, SPI0_TX, GP3 - [ 05 ][ 36 ] - 3V3( OUT )
      UART1_TX, I2C0_SDA, SPI0_RX, GP4 - [ 06 ][ 35 ] - ADC_VREF
     UART1_RX, I2C0_SCL, API0_CSn, GP5 - [ 07 ][ 34 ] - GP28, ADC2
                                   GND - [ 08 ][ 33 ] - GND, AGND
               I2C0_SDA, SPI0_SCK, GP6 - [ 09 ][ 32 ] - GP27, ADC1, I2C1_SCL
                I2C1_SCL, SPI0_TX, GP7 - [ 10 ][ 31 ] - GP26, ADC0, I2C_1_SDA
      UART1_TX, I2C0_SDA, SPI1_RX, GP8 - [ 11 ][ 30 ] - RUN
     UART1_RX, I2C0_SCL, SPI1_CSn, GP9 - [ 12 ][ 29 ] - GP22
                                   GND - [ 13 ][ 28 ] - GND
              I2C1_SDA, SPI1_SCK, GP10 - [ 14 ][ 27 ] - GP21, I2C0_SCL
               I2C1_SCL, SPI1_TX, GP11 - [ 15 ][ 26 ] - GP20, I2C0_SDA
     UART0_TX, I2C0_SDA, SPI1_RX, GP12 - [ 16 ][ 25 ] - GP19, SPI0_TX, I2C1_SCL
    UART0_RX, I2C0_SCL, API1_CSn, GP13 - [ 17 ][ 24 ] - GP18, SPI0_SCK, I2C1_SDA
                                   GND - [ 18 ][ 23 ] - GND
              I2C1_SDA, SPI1_SCK, GP14 - [ 19 ][ 22 ] - GP17, SPI0_CSn, I2C0_SCL, UART0_RX
               I2C1_SCL, SPI1_TX, GP15 - [ 20 ][ 21 ] - GP16, SPI0_RX, I2C0_SDA, UART0_TX

Infineon 43439 ( 2.4GHz 802.11n, part number: CYW43439 )
LED on WL_GPIO0
```
```
All expansion ports have 3V3 and Ground along with the following GPIO ( see Pin Resource for additional capabilities )
Counter clockwise
1 - GP0, GP1
2 - GP2, GP3
3 - GP4, GP5
4 - GP10, GP11
5 - GP12, GP13
6 - GP14, GP15
7 - GP16, GP17
8 - GP18, GP19
9 - GP20, GP21
10 - GP22, GP26
11 - GP27, GP29
```

### DeskPi PicoMate Features
```
Built-in battery connector for on-the-go projects

User Interface - Inputs
    Button ( GPIO )
        GP26
    Rotary Encoder ( GPIO )
        A - GP7
        B - GP6
    Microphone [ ZTS6531S ] ( PDM )
        CLK - GP9
        DATA - GP8

User Interface - Outputs
    RGB Lights [ WS2812 ] ( GPIO )
        DIN - GP22
    Buzzer ( Generic PWM )
        CTRL - GP27
    OLED Display 128x64 [ SSD1315 ] ( I2C0, addr 0x3C )
        SCL - GP17
        SDA - GP16

Environmental Sensors
    Gyroscope Sensor [ LSM6DS3TR-C ] ( I2C1, addr 0x6A )
        SCL - GP15
        SDA - GP14
    3-Axis Magnetometer Sensor [ MMC5603NJ ] ( I2C1, addr 0x30 )
        SCL - GP15
        SDA - GP14
    Photosensitive Sensor [ LTR-381RGB-01 ] ( I2C1, addr 0x53 )
        SCL - GP15
        SDA - GP14
    Temperature and Humidity Sensor [ SHT30-DIS ] ( I2C1, addr 0x44 )
        SCL - GP15
        SDA - GP14
    PIR Sensor [ AS312 ] ( GPIO )
        DOUT - GP28
```

## Firmware/Software

### Micro Python Development
- https://www.raspberrypi.com/documentation/microcontrollers/micropython.html
- Make sure the firmware is updated for wireless support
    - https://projects.raspberrypi.org/en/projects/get-started-pico-w/1
* Micro Python IDE options
    - Thonny - https://thonny.org/

### C SDK Development
- https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
* If development is going to be done on a Raspberry Pi, a setup script is available
    * run at your own risk on other systems otherwise follow instructions at website
    * Raspberry Pi instructions can be skipped with certain definitions or just local setup script editing
```
./pico_setup.sh
```
* The script installs a tool called picotool
```
SYNOPSIS:
    picotool info [-b] [-p] [-d] [-l] [-a] [--bus <bus>] [--address <addr>] [-f] [-F]
    picotool info [-b] [-p] [-d] [-l] [-a] <filename> [-t <type>]
    picotool load [-n] [-N] [-u] [-v] [-x] <filename> [-t <type>] [-o <offset>] [--bus <bus>] [--address <addr>] [-f] [-F]
    picotool save [-p] [--bus <bus>] [--address <addr>] [-f] [-F] <filename> [-t <type>]
    picotool save -a [--bus <bus>] [--address <addr>] [-f] [-F] <filename> [-t <type>]
    picotool save -r <from> <to> [--bus <bus>] [--address <addr>] [-f] [-F] <filename> [-t <type>]
    picotool verify [--bus <bus>] [--address <addr>] [-f] [-F] <filename> [-t <type>] [-r <from> <to>] [-o <offset>]
    picotool reboot [-a] [-u] [--bus <bus>] [--address <addr>] [-f] [-F]
    picotool version [-s]
    picotool help [<cmd>]

COMMANDS:
    info      Display information from the target device(s) or file.
              Without any arguments, this will display basic information for all connected RP2040 devices in BOOTSEL mode
    load      Load the program / memory range stored in a file onto the device.
    save      Save the program / memory stored in flash on the device to a file.
    verify    Check that the device contents match those in the file.
    reboot    Reboot the device
    version   Display picotool version
    help      Show general help or help for a specific command

Use "picotool help <cmd>" for more info

```
* Develompent environments include standard command line operations and IDEs that support CMAKE like vscode
* Project examples are CMAKE projects with board and wifi definitions being given as follows:
```
cd <sdk_root>/pico/pico-examples/build/
cmake .. -DPICO_BOARD=pico_w -DWIFI_SSID="<your network>" -DWIFI_PASSWORD="<your password>"
cd pico-w/wifi/<project_of_interest>/
make
```
* output of a build will be a .uf2 file that can be put (drag-drop or scp) on the Pico W when in bootload mode

