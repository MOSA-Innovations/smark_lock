# Introduction

The source code is based on the Nordic RF52840 development board that supports NFC and BLE functionaility and the lastest SDK version 16.0.

We developed it following on the sample code provided by Nordic SDK, which can be found in the path `examples/ble_peripheral/experimental/ble_nfc_pairing_reference`.

# Explanation

The basic way to implement the communcations via BLE between the board and the device is to create a new customised service in GATT. We define a series of string to tell the board when and how to control the relay to lock or unlock a eletronic locker. Th string consists of command, access code, phone id and timestamp :

`|cmd | Length of access code | access code | Phone ID | timestamp |`

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
