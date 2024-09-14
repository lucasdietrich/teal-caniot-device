.PHONY: build flash menuconfig clean reports build_f072 build_f429 build_caniot build_h745

all: build

build:
	west build

build_f072:
	west build -b teal_f072

build_caniot:
	west build -b teal_caniot

build_h745:
	west build -b nucleo_h745zi_q/stm32h745xx/m7

build_f429:
	west build -b nucleo_f429zi

build_nucleo_l452:
	west build -b nucleo_l452re

flash:
	west flash

menuconfig:
	west build -t menuconfig

reports:
	west build -t ram_report > build/ram_report.txt
	west build -t rom_report > build/rom_report.txt

clean:
	rm -rf build