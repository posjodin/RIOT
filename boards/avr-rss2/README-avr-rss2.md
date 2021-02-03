Getting Started with RIOT-OS using avr-rss2
===========================================

Document version
----------------
V1.5 2021-02-03

This guide's aim is to help you start using RIOT-OS on RSS2 boards
* Based on MCU AtMega256RFR2 (Board. Rev2.3 and Rev2.4)

This guide assumes that you have basic understanding of how to use the 
command line and can perform basic admin tasks on UNIX family OSs. You
should also have understanding about the RIOT-OS design as well have C
programming skills. Having some skill with practical electronics as
connecting cables, sensors and power is expected.

Board Features
---------------
* On PCB Antenna. Supercardiod coverage. (Top of PCB)
* Robust radio performance. Up to 300 meter line-of-sight.
* Unique EUI64 address for (6LowPAN, ip6) via chip combined with EEPROM
* Rev 2.3 Onboard temp sensor DS18B20,
* Rev 2.4 Onboard BME280 Temp/RH/Pressure
* Light Sensor.
* 32kHz RTC clock xtal
* Comparator chip and input. 
* Progammable power FET, (relay) for power on/off of sensors.
* DC input via LDO up to 25V.
* Standard. 6-pin TTL-USB header for FTDI cable for USART.
  Or USB-TTL adaptor.
* Pin headers for I2C, SPI, ADC, GPIO
* PCB formfactor for cheap project box G.40X IP54
* Power/current use:
  * RX ~10mA (Full RPC AtMegaXXRFR2). 
  * Sleep ~45uA @ 16MHz XTAL
  * Sleep ~15uA @  8MHz using internal oscillator
* Preprogammed Atmel standard bootloader stk500v2 @ 115200 bps 
* CE certified by test institute.

USART 
-----
The board has one USART via the 6-pin TTL-USB adapter, The default
baudrate is 115200 bps. It's possible to use higher speeds as is 250k
and 500k baud which gives 0% Error with 16MHz clock. 
An addtional 2:nd USART is on the chip can be used after HW modification.

RIOT-OS Port Features
---------------------
Atmel AVR is well supported in RIOT-OS. Atmel AVR MCU's is also used
in many of the Arduino boards.

The board has the following features:
* Standard, E64 address from built-in chip. Chip also has a 128bit ID.
* Support for Atmel RPC (Reduced Power Consumption) for AtMegaXXXRFR2. 
* Stable port since many years.

Toolchain alternative I
-----------------------
The Atmel toolcahin is available in most operating systems.

### For Linux
For a full robust toolchain and easy installation on Ubuntu:\
apt-get install gcc-avr avr-libc avrdude

Toolchain alternative II
------------------------
Otherwise if OS toolchain packages do not support the AtMega256RfR2
MCU, another option is to download the C compiler toolchain from
Microchip.

### For Linux and MacOS

1. Download the proper 8-bit C compiler AVR toolchain, 32 or 64-bit,
   [from Microchip](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers).
2. Unpack under `/usr/local`.
3. Add to your search PATH. For example add to `.bashrc`: `export PATH=$PATH:/usr/local/avr8-gnu-toolchain-linux_x86_64/bin` (for 64-bit systems) or `export PATH=$PATH:/usr/local/avr8-gnu-toolchain-linux_x86/bin` (for 32-bit systems).
4. For flash programming, you need `avrdude`. On Ubuntu Linux, it can be installed with the command
`apt-get install avrdude`. For MacOS, there is a [HomeBrew
package](https://formulae.brew.sh/formula/avrdude#default) that can be
installed with the command `brew install avrdude`. (You first need to
[install HomeBrew](https://brew.sh).)

The RIOT build system relies heavily on GNU Make. On Mac, GNU Make is not the default "make" program,
so you may need to install it as well. See [HomeBrew Formulae make](https://formulae.brew.sh/formula/make).

Toolchain Windows
-----------------
MIT has an updated instruction. You need to be root.\
http://fab.cba.mit.edu/classes/863.16/doc/projects/ftsmin/windows_avr.html

Microchip repo:
---------------
https://www.microchip.com/en-us/development-tools-tools-and-software/gcc-compilers-avr-and-arm

avrdude programming tool
------------------------
http://download.savannah.gnu.org/releases/avrdude/

RIOT-OS board name
------------------
avr-rss2

RIOT-OS build BOARD
--------------------
make BOARD=avr-rss2

Flashing/Programming hardware
------------------------------
Programming using avrdude using serial bootloader. (TTL-USB cable)
Press the RESET button. The bootloader waits for boot commands for 3 seconds.
During that time the flash command should be given.

Flashing commnad line example:\
avrdude -V -p m256rfr2 -c stk500v2  -P /dev/ttyUSB0 -b 115200 -e -U flash:w:bin/avr-rss2/hello-world.elf


RIOT-OS applications
--------------------
RIOT-OS has huge amount of examples and tests for different
protocols and MCU, boards and devices. Many of the applications
can be used with avr-rss2 board. Also many of the tests are
appropriate. Those are found in the test directory.

Example of tested applications
------------------------------
* `examples/hello-world`
* `examples/default`
* `examples/gnrc_minimal`
* `examples/gnrc_networking`
* `tests/...`

Wireshark (Contiki)
-------------------
Wireshark in-air realtime protocol decoding in Contiki.
examples/sensniff with sensniff

python script creating a named pipe as wireshark input. Be aware of 
restrictions in 6LoWPAN compression format.


Board approvals
---------------
 Summary: CE approved Radio Equipment Directive (RED) 2014/53/EU

Rev 2.4
* Safety: IEC 60950-1:2005 2nd Edition +Am 1:2009 +Am 2:2013
* RF: ETSI EN 300 328 V2.1.1 (2016-11)
* EMC: Draft ETSI EN 301 489-1 V2.2.0 (2017-03), 
  Draft ETSI EN 301 489-17 V3.2.0 (2017-03)
* EMF: EN 62479:2010
* Human exposure to electromagnetic fields: EN 62479:2010 

Rev 2.3
* R&TTE 73/23/EEC, 89/336/EEC and 99/5/EC
* Safety: EN 60950-1:2006 + A12: 2011
* RF: ETSI EN 300 328 V1.7.1 (2006-10)
* EMC: ETSI EN 301 489-1 V1.9.2 (2011-09), ETSI EN 301 489-17 V2.2.1 (2012-09)
* EMF: EN 62479:2010
* Human exposure to electromagnetic fields: EN 62479:2010 

Commercial availability
------------------------
Through vendor and though resellers. Note board is will only available 
were CE approval is covered. This may be further restricted by WEEE.
Contact vendor. For research legislation is more relaxed in most 
countries.

References
----------
AtMega64/128/256/RFR2 chip documentation available via Atmel.
Schematics and boards description. Available via Radio-Sensors
Smart Reduced Power Consumption Techniques. AT02594 available via Atmel.

Board (Rev2.4) anatomy with connectors:

http://radio-sensors.com/pictures/s2-2.4-front-edited.jpg

http://radio-sensors.com/pictures/s2-2.4-back-port-edited.jpg

Vendor info
-----------
http://radio-sensors.com/

Maintainer
----------
Robert Olsson <robert@radio-sensors.com>
