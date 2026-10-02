# RP2040-FreeRTOS I2C Slave 1.0.0

This repo contains project for [FreeRTOS](https://freertos.org/) on the Raspberry Pi RP2040 microcontroller

## Project Structure

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

My original problem was to find a way to read a Text file into the Pico which would then read each character and send its equivalent as Morse Code, **CW**.  I did not have a file system on the Pico so bringing the characters in using the I2C interface looked doable.  I read the text file with a **RPi Zero**, then send each character to the front of a Ring Buffer on the Pico.  A FreeRTOS Queue reads the tail of the Ring Buffer and offers the character to the CW task which picks it up the next time it needs a character.
The Ring Buffer is formed on the global data structure provided for the I2C Slave interface.
I provide code for both a Pico and Pimoroni Tiny.

## Supporting Functionality

The I2C Slave functionality feeds a CW task which accepts ASCII characters from the I2C Master and outputs Morse Code Dits and Dahs.

An application, **RPi-Text-Reader-Cmd.cpp is provided to run on the RPi to create the I2C Master. **RPi-TEXT-Reader-Cmd.cpp** is loaded with the 
name of the file to be read and presents a table of Code speeds with which you can select the code speed (10 WPM to 25 WPM) on starting.

You can use the switch of the Tiny or the Boot switch of the Tiny to lengthen the space between characters.

I feed the Audio output into an ordinary PC external Audio module.

Load the Pico or Tiny with its Application code, then start the RPi with ./RPi-Text-Reader-Cmd Code-Groups.txt. As soon as you select a Code Speed you
will have Morse character tones on the Audio output GPIO.

Various Text files are provided for Code Reading Practice.

The Pico and Tiny Apps use the SDK pio command **pio_sm_set_enabled** to switch the pio Square Wave **OFF** and **ON** to create the 5oo Hz CW Audio signal. 

This work has as its foundation the code provided by [smittytone/RP2040-FreeRTOS project](https://github.com/smittytone/RP2040-FreeRTOS).


## Copyright and Licences

Application source © 2022, Calvin McCarthy and licensed under the terms of the [MIT Licence](./LICENSE.md).

Application source © 2022, Tony Smith and licensed under the terms of the [MIT Licence](./LICENSE.md).

[FreeRTOS](https://freertos.org/) © 2021, Amazon Web Services, Inc. It is also licensed under the terms of the [MIT Licence](./LICENSE.md).

The [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) is © 2020, Raspberry Pi (Trading) Ltd. It is licensed under the terms of the [BSD 3-Clause "New" or "Revised" Licence](https://github.com/raspberrypi/pico-sdk/blob/master/LICENSE.TXT).
