# RP2040-FreeRTOS try 1.0.0

This repo contains project for [FreeRTOS](https://freertos.org/) on the Raspberry Pi RP2040 microcontroller

## Project Structure

```
/FreeRTOS-CW-I2C
|___/App-CW-I2C-Slave/main_I2C_Slave_PCF8575.c    // Source code for App-CW-I2C-Slave adding a PCF8575 GPIO extender
|___/Common                 // Source code common to applications - pico
|___CMakeLists.txt          // Top-level project CMake config file
|___/App-CW-I2C-Slave/I2C_Slave-RingBuf-input.cpp  // RPi code for I2C Master reading TXT files
|___README.md
|___LICENSE.md
```

## Prerequisites

To use the code in this repo, your system must be set up for RP2040 C/C++ and FreeRTOS development. See [this blog post of "smittytone"](https://blog.smittytone.net/2021/02/02/program-raspberry-pi-pico-c-mac/) for setup details.

## Usage

1. Clone the repo: `git clone https://github.com/smittytone/RP2040-FreeRTOS`.
1. Enter the repo: `cd FreeRTOS-PICO`.
1. Install the submodules: `git submodule update --init --recursive`.
1. Edit `CMakeLists.txt` and `/<Application>/CMakeLists.txt` to rename the project.
1. Optionally, manually configure the build process: `cmake -S . -B build/`.
1. Optionally, manually build the app: `cmake --build build`.
1. Connect your device so it’s ready for file transfer.
1. Install the app (I use the Drag and Drop process described in the pico-sdk documentation)

## The App

This App exercises the I2C Slave software provided by **Valentin Milea <valentin.milea@gmail.com>** and included in the Pico SDK codebase.

## The I2C Functionality

My original problem was to find a way to read a Text file into the Pico which would then read each character and send its equivalent as Morse Code, (CW).  I did not have a file system on the Pico so bringing the characters in using the I2C interface looked doable.  I read the text file with a RPi Zero, then send each character to the front of a Ring Buffer on the Pico.  A FreeRTOS Queue reads the tail of the Ring File and offers the character to the CW task which picks it up the next time it needs a character.
The Ring Buffer is formed on the data structure provided for the I2C Slave interface.

## Supporting Functionality


The I2C functionality feeds a CW task which accepts ASCII characters from the I2C Master and outputs Morse Code Dits and Dahs.
An A/D task reads a variable resister value and converts it to sixteen levels.  These sixteen levels are converted to timer delay values to give code speeds of 10 WPM (Words per Minute) to 26 WPM. These code speed values are transfered to the Timer generating the dot delay times.
A Switch Debounce task provides a start signal to initiate reading the text and sending the Morse code. The start signal is sent from the I2C Slave back to the I2C Master to initiate Text reading.
A PCF8575 GPIO Extender is added on the Pico as a second I2C interface to provide more switch inputs and an LED CODE Speed indication.
An application, **I2C-Slave-RingBuf-input.cpp** is provided to run on the RPi to provide the I2C Master.


## Credits

This work has as its foundation the code provided by [smittytone/RP2040-FreeRTOS project](https://github.com/smittytone/RP2040-FreeRTOS).


## Copyright and Licences

Application source © 2022, Calvin McCarthy and licensed under the terms of the [MIT Licence](./LICENSE.md).

Application source © 2022, Tony Smith and licensed under the terms of the [MIT Licence](./LICENSE.md).

[FreeRTOS](https://freertos.org/) © 2021, Amazon Web Services, Inc. It is also licensed under the terms of the [MIT Licence](./LICENSE.md).

The [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) is © 2020, Raspberry Pi (Trading) Ltd. It is licensed under the terms of the [BSD 3-Clause "New" or "Revised" Licence](https://github.com/raspberrypi/pico-sdk/blob/master/LICENSE.TXT).
