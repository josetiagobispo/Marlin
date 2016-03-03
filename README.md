# Marlin Duo 3D Printer Firmware
<img align="top" width=175 src="Documentation/Logo/Marlin%20Logo%20GitHub.png" />

## Marlin Duo 1.1.0-RC3 - 05 February 2016

## Information
Marlin Duo is modified version of [Marlin](https://github.com/MarlinFirmware/Marlin/tree/RCBugFix).
This branch is compatible with both Arduino MEGA and Due. 

__Not for production use – use with caution!__

## Recent Changes
RC3 - 05 Feb 2016<br>
      First release<br>
	  based on [original Marlin RCBugFix 1.1.0-RC3](https://github.com/MarlinFirmware/Marlin/tree/RCBugFix)<br>
      Due part was ported [from Marlinkimbra4due 4.2.4 dev](https://github.com/MagoKimbra/MarlinKimbra4due) and [Marlin4Due 1.0.3 dev](https://github.com/Wurstnase/Marlin4Due)<br>

## Current Status: Experimental

### known issues
* HEATER_0_USES_MAX6675 does not work on Due
* M100_FREE_MEMORY_WATCHER does not work on Due
* FAST_PWM_FAN does not work on Due
* WATCHDOG_RESET_MANUAL does not work on Due
* ADVANCE does not work on Due
* Makefile needs update

### Special thanks
* [MagoKimbra](https://github.com/MagoKimbra)
* [Wurstnase](https://github.com/Wurstnase)
* [bobc](https://github.com/bobc)
* [developers of original Marlin](https://github.com/MarlinFirmware)

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

[![Flattr this git repo](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/MarlinFirmware/Marlin&title=Marlin&language=&tags=github&category=software)
