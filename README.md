# SX1278-example
Example for STM32 HAL driver for LoRa SX1278 module 

This is an example project for SX1278 LoRa wireless communication module.
It includes the diver. The driver itself can be found in [SX1278](https://github.com/wdomski/SX1278) 
repository.

The example was prepared under SW4STM32 for STM32F103C8T6 developement board 
and can be easily ported to any other device. Also STM32CubeMX configuration 
file is included.

# How to use it

## Prerequisite
You need two boards equiped with STM32F103C8T6 MCU and two SX1278 modules.
One of the boards will work as master and the other one will work as slave.
Master is sending data to slave.

Debugger -- ST-Link V2 with SWV capabilities.

## Compilation
You have to compile it under SW4STM32 or import it into i.e. TrueSTUDIO - Atollic.

## Run
After flashing the image you should mind the PB2 port. It is configured 
as input and depending on its state the board boots as a master or as a slave.
For master mode the PB2 has to be pulled high.
For slave mode the PB2 has to be pulled low.

All the comunication is going through redirected printf to SWV.
To read the communicates you have to have ST-Link Utility installed. 
There you can chose **Printf via SWO viewer**.

The master board is constantly sending messages while the slave is constantly 
reading them.

# Final remarks

You can visit [my blog](http://blog.domski.pl) to read more about the SX1278 modules.

