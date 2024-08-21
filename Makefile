.PHONY: build build_f072 menuconfig clean reports

all: build

build_f072:
	west build -b teal_f072

build:
	west build -b teal_caniot

menuconfig:
	west build -t menuconfig

reports:
	west build -t ram_report > build/ram_report.txt
	west build -t rom_report > build/rom_report.txt

clean:
	rm -rf build