# Default recipe
default := "build"

# Build targets
build:
    west build

teal:
    west build -b teal_caniot

teal-rust:
    west build -b teal_caniot -- -DCONF_FILE=prj_rust.conf

teal-test:
    west build -b teal_caniot -- -DCONF_FILE=prj_test.conf

qemu:
    west build -b qemu_cortex_m3

run:
    west build -t run

nucleo-l452:
    west build -b nucleo_l452re

nucleo-f072:
    west build -b teal_f072

nucleo-h745:
    west build -b nucleo_h745zi_q/stm32h745xx/m7

nucleo-f429:
    west build -b nucleo_f429zi

# Tools
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

# Serial monitoring
usb0:
    python3 -m serial.tools.miniterm /dev/ttyUSB0 115200

acm0:
    python3 -m serial.tools.miniterm /dev/ttyACM0 115200

acm1:
    python3 -m serial.tools.miniterm /dev/ttyACM1 115200

miniterm DEV="/dev/ttyUSB0":
    python3 -m serial.tools.miniterm {{ DEV }} 115200