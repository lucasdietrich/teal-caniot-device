.PHONY: build flash debugserver debug menuconfig clean reports teal nucleo_l452 nucleo_f072 nucleo_h745 nucleo_f429

all: build

build:
	west build

# board specific build
teal:
	west build -b teal_caniot

nucleo_l452:
	west build -b nucleo_l452re

nucleo_f072:
	west build -b teal_f072

nucleo_h745:
	west build -b nucleo_h745zi_q/stm32h745xx/m7

nucleo_f429:
	west build -b nucleo_f429zi

# tools
flash:
	west flash

debugserver:
	west debugserver

debug:
	west debug

menuconfig:
	west build -t menuconfig

reports:
	west build -t ram_report > build/ram_report.txt
	west build -t rom_report > build/rom_report.txt

clean:
	rm -rf build