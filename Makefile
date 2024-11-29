.PHONY: build flash debugserver debug menuconfig clean reports teal sysbuild

all: build teal flash debug menuconfig reports clean

# nucleo_l452re nucleo_f429zi
BOARD=teal_caniot

build: teal

# board specific build
teal:
	west build -b $(BOARD)

test:
	west build -b $(BOARD) -- -DFILE_SUFFIX=test

sysbuild_test:
	west build -b $(BOARD) --sysbuild -- -DFILE_SUFFIX=test

sysbuild:
	west build -b $(BOARD) --sysbuild

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
	./scripts/zephyr-report.sh

clean:
	rm -rf build