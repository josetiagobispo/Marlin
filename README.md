# Marlin Duo 3D Printer Firmware

[![Build Status](https://travis-ci.org/esenapaj/Marlin.svg?branch=Duo)](https://travis-ci.org/esenapaj/Marlin)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/2224/badge.svg)](https://scan.coverity.com/projects/2224)

<img align="top" width=175 src="buildroot/share/pixmaps/logo/marlin-250.png" />

## Marlin Duo 1.1.0-RCBugFix - 26 July 2016

## Information
Marlin Duo is modified version of [Marlin](https://github.com/MarlinFirmware/Marlin/tree/RCBugFix).
This branch is compatible with both Arduino MEGA and Due. 
  - based on [original Marlin 1.1.0-RCBugFix](https://github.com/MarlinFirmware/Marlin/tree/RCBugFix)<br>
  - Due part was ported [from MK4duo 4.2.4 - 4.2.87](https://github.com/MagoKimbra/MK4duo) and [Marlin4Due 1.0.3 dev](https://github.com/Wurstnase/Marlin4Due)<br>

__Not for production use – use with caution!__

## Current Status: Experimental

### known issues
 - DRV8825 does not work on Due
 - MOTOR_CURRENT_PWM (_XY, _Z, _E) does not work on Due
 - M100_FREE_MEMORY_WATCHER does not work on Due
 - FAST_PWM_FAN does not work on Due
 - TX_BUFFER_SIZE does not work on Due
 - EMERGENCY_PARSER does not work on Due
 - Makefile needs update

### Special thanks
 - [MagoKimbra](https://github.com/MagoKimbra)
 - [Wurstnase](https://github.com/Wurstnase)
 - [bobc](https://github.com/bobc)
 - [ivanseidel](https://github.com/ivanseidel)
 - [developers of original Marlin](https://github.com/MarlinFirmware)

## License

Marlin is published under the [GPL license](/LICENSE) because we believe in open development.<br>
The GPL comes with both rights and obligations.<br>
Whether you use Marlin firmware as the driver for your open or closed-source product,<br>
you must keep Marlin open, and you must provide your compatible Marlin source code to end users upon request.<br>
The most straightforward way to comply with the Marlin license is to make a fork of Marlin on Github,<br>
perform your modifications, and direct users to your modified fork.<br>

While we can't prevent the use of this code in products (3D printers, CNC, etc.)<br>
that are closed source or crippled by a patent,<br>
we would prefer that you choose another firmware or, better yet, make your own.<br>
