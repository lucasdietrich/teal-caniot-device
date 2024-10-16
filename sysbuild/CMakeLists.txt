# This file is required if you don't build with
# `west build -b $(BOARD) --sysbuild -- -DBOARD_ROOT=.`

# Get the parent directory
get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(PARENT_DIR ${CURRENT_DIR} DIRECTORY)

# set board root
set(BOARD_ROOT ${PARENT_DIR})
find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
