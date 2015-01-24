# Cheap and simple DDS 0-8 MHz based on ATTiny13a, HD44780 and AD9850 

## Intro

You can buy nice and cheap AD9850 DDS boards on ebay, they often sold with 125 MHz crystal, and can go up to 62.5 MHz.
I've had couple ATTiny13a's and LCD screen, so I've decided to create simple generator with them.

## Features

You can watch demo video [here](http://youtu.be/lxxVBcAxBDc).

- Two Modes:
	- 0 - 1020 Hz
	- 0 - 8160 KHz
- Switch between modes by turning to leftmost position.

## Schematics

![](http://i.imgur.com/uszv575.png)

## Code

Main limitation is ATTiny13a's 1 Kb of Flash memory because we need to do bit banging to control boards. I've found couple libs to control lcd and dds but they are so big, so I've decided to copy-paste main parts from libs and optimize code for optimal size.

For compilation I've used this settings :

	avr-gcc.exe -g -Os -fwhole-program -pedantic -mmcu=attiny13 -std=c99 -o attiny13a_dds.o -c attiny13a_dds
	avr-gcc.exe -g -Os -fwhole-program -pedantic -Wl,--gc-sections -mmcu=attiny13 -std=c99 -o attiny13a_dds.elf attiny13a_dds.o
	avr-objcopy.exe -O ihex attiny13a_dds.elf attiny13a_dds.hex

And for upload (note, I'm using default fuses) :

	avrdude.exe -c %YOUR_ISP% -P %YOUR_PORT% -p attiny13 -v -U lfuse:w:0x6a:m -U hfuse:w:0xff:m -U flash:w:attiny13a_dds.hex
 
Final binary size is 1020 bytes.