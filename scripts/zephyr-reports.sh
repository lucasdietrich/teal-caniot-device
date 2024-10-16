#!/usr/bin/bash

# 
function zephyr_report() {
    if [ ! -d "$1" ]; then
        echo "Usage: zephyr_report <dir_path>"
        return 1
    fi

    # get relative path
    local dir_path=$(realpath $1)

    west build -d $dir_path -t ram_report > $dir_path/ram_report.txt
	west build -d $dir_path -t rom_report > $dir_path/rom_report.txt
}

# run the function
zephyr_report "build/teal-caniot-device"
zephyr_report "build/mcuboot"
zephyr_report "build"