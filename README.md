# SX1278-example
Example for STM32 HAL driver for LoRa SX1278 module 

This is an example project for SX1278 LoRa wireless communication module.
It includes the diver. The driver itself can be found in [SX1278](https://github.com/wdomski/SX1278) 
repository. For more information on the driver including 
what LoRa modules are supported please refer to the README file 
in [SX1278](https://github.com/wdomski/SX1278) repository.

First version of the example was prepared with SW4STM32 IDE 
for STM32F103C8T6 development board 
and can be easily ported to any other device. Also STM32CubeMX configuration 
file is included. 
The current state of the project was prepared for Nucleo-L476RG development board.

# Installation

First download this project. Preferably with git clone.
Remeber to download submodules for this repo.
```
git clone git@github.com:wdomski/SX1278-example.git
cd SX1278-example
git submodule update --init --recursive
```

# How to use it

## Prerequisite

You need two boards, preferably identical, and two SX1278 modules.
One of the boards will work as master and the other one will work as slave.
Master is sending data to slave. Also switching operation 
mode is possible by pressing user button available at the board.

Debugger -- ST-Link V2 with SWV capabilities.
*Please remember to upgrade the firmware in ST-Link*

## Connecting module

The module has to be connected with the MCU board 
with following pins:

| MCU pin | SX1278 pin | Description                                      |
| ------- | ---------- | -------------------------------------------------|
| PA4     | NSS        | SPI chip--select                                 |
| PB10    | SCK        | SPI SCK                                          |
| PC2     | SO         | SPI MISO                                         |
| PC3     | SI         | SPI MOSI                                         |
| PB0     | DIO0       | LoRa interrupt indicator pin (for receiver mode) |
| PB1     | RST        | LoRa reset                                       |
| ------- | ---------- | -------------------------------------------------|
| VDD     | VCC        | +3V3                                             |
| VSS     | GND        | GND                                              |

## Compilation
You have to compile it using STM32CubeIDE.

## Run
After flashing the image you should mind the PC13 port. To this a blue push 
button is connected. It is configured 
as input and depending on its state the board boots as a master or as a slave.
For master mode the PC13 has to be pulled high (button not pressed).
For slave mode the PC13 has to be pulled low (button pressed).

All the communication is going through redirected *printf* to SWV and USART2.
This is duplicated due to more robustness. You can use either.
To read the communicates you have to have ST-Link Utility installed. 
There you can chose **Printf via SWO viewer**.
When using serial interface any serial terminal is suitable. The 
configuration is standard 115200 8N1 (115200 bps, 8 bits for frame, no 
parity and a single STOP bit).


The master board is constantly sending messages while the slave is constantly 
reading them. To change the operation simply push the blue button and hold 
it for a moment.

# Final remarks

You can visit [my blog](http://blog.domski.pl) to read more about the SX1278 modules or my other projects.

