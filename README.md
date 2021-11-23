# Introduction

The source code is based on the Nordic nRF52832 development board that supports NFC and BLE functionaility. The SDK version is 17.0.2 and the softdevice is s132.

We developed it following on the sample code provided by Nordic SDK, which can be found in the path `examples/ble_peripheral/experimental/ble_nfc_pairing_reference`.

# Linux Installation 

1) Download and install the [Segger Embedded Studio](https://www.segger.com/downloads/embedded-studio).


2) Download the [nrf sdk 17.02](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs) and unzip the content in your home directory. 

3) Install the [Segger J-Link software](https://www.segger.com/downloads/jlink/)

4) In segger studio go to tools/options/building/global macros and paste:

    `nRF_SDK_ROOT=../../../../../../../nRFSDK1702`

5) Compile the uECC.h library

    After properly setting the nRF_SDK_ROOT global macro and start compiling you will get the following error:
    ```
    7> /home/pavlos/nRFSDK1702/components/libraries/ecc/ecc.c:55:10: fatal error: uECC.h: No such file or directory
    ```
    Open a terminal:
    ```sh
    sudo apt-get update -y
    sudo apt-get install -y gcc-arm-none-eabi
    arm-none-eabi-gcc --version
    which arm-none-eabi-gcc 
    ```
    To set your toolchain directories, edit $(nRF_SDK_ROOT)/components/toolchain/gcc/Makefile.posix
    ```
    GNU_INSTALL_ROOT ?= /usr/bin/
    GNU_VERSION ?= 9.2.1
    GNU_PREFIX ?= arm-none-eabi
    ```
    Open the terminal again and type in:
    ```sh
    $ cd external/micro-ecc/nrf52hf_armgcc/armgcc/
    $ make
    Makefile:82: Cannot find source file: ../../micro-ecc/uECC.c
    Makefile:82: Cannot find include folder: ../../micro-ecc
    mkdir _build
    cd _build && mkdir micro_ecc_lib
    Compiling file: uECC.c
    arm-none-eabi-gcc: error: ../../micro-ecc/uECC.c: No such file or directory
    arm-none-eabi-gcc: fatal error: no input files
    compilation terminated.
    make: *** [../../../../components/toolchain/gcc/Makefile.common:272: _build/micro_ecc_lib/uECC.c.o] Error 1
    $ cd ../..
    $ chmod a+x build_all.sh 
    $ sudo apt install dos2unix
    $ dos2unix build_all.sh 
    $ ./build_all.sh 
    $ find -iname uECC.h
    ./micro-ecc/uECC.h
    ```
    Now the  firmware compilation should work.


6) Create a new project. 

    You can open the project file in a text editor that supports "search and replace" and replace the relative paths with a variable which you can be defined later in SES. 
    ```sh
    eg. `$(nRF_SDK_ROOT)/components` instead of `../../../../../../components/`
    ```


# Debugging

The firmware exposes the the nrf log reports on pin IO 0.6. Connect an FTDI chip and then use:
sudo picocom -b 115200 /dev/ttyUSB0


# BLE Service Explanation

The basic way to implement the communcations via BLE between the board and the device is to create a new customised service in GATT. We define a series of string to tell the board when and how to control the relay to lock or unlock a eletronic locker. Th string consists of command, access code, phone id and timestamp :

`|cmd | Length of access code | access codes | Phone ID | timestamp |`

# File

* `audio_control.c` :  Handle audio events
* `ble_control.c` : Handle Bluetoot Low Energy (BLE) events and communication
* `button_control.c` : Handle button events
* `gpio_control.c` : Handle GPIO events
* `led_control.c` : Handle LED events
* `lock_mech_events.c` : Handle locking mechanism events
* `log_control.c` : Handle events logger for debug
* `main.c` : Main file
* `nfc_control.c` : Handle NFC events and communication
* `peer_control.c` : Handle peer events
* `persistent_storage.c` : Handle persistent storage events
* `power_management_control.c` : Handle power management events
* `security.c` : Handle security events
* `smart_lock.c` : Top level Smart Lock control
* `smart_lock_core.c` : Smart Lock core functionality
* `smart_lock_gatt_service` : Custom GATT service for Smart Lock
* `timer_control.c` : Handle timer events

# References

[Nordic SDK 17.0.2](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fstruct_sdk%2Fstruct%2Fsdk_nrf5_latest.html)

[Nordic DevZone](https://devzone.nordicsemi.com/)
