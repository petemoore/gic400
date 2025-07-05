#!/bin/bash -eu

set -xv

cd "$(dirname "${0}")"
rm -rf dist
mkdir dist

aarch64-none-elf-as -g -o gic400.o gic400.s
aarch64-none-elf-ld --section-start=.uart_base=0xfe215000 --section-start=.gpio_base=0xfe200000 --section-start=.gic_base=0xff841000 -Ttext=0x0 -o gic400.elf gic400.o
aarch64-none-elf-objcopy gic400.elf -O binary dist/kernel8.img

rm gic400.{o,elf}
wget -q -O dist/LICENCE.broadcom https://github.com/raspberrypi/firmware/blob/ae093e011da705b72c09da5e64db7617e8f3191a/boot/LICENCE.broadcom?raw=true
wget -q -O dist/bootcode.bin https://github.com/raspberrypi/firmware/blob/ae093e011da705b72c09da5e64db7617e8f3191a/boot/bootcode.bin?raw=true
wget -q -O dist/fixup4.dat https://github.com/raspberrypi/firmware/blob/ae093e011da705b72c09da5e64db7617e8f3191a/boot/fixup4.dat?raw=true
wget -q -O dist/start4.elf https://github.com/raspberrypi/firmware/blob/ae093e011da705b72c09da5e64db7617e8f3191a/boot/start4.elf?raw=true
cp config.txt dist/config.txt
