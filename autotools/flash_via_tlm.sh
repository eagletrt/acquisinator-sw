#! /bin/env bash
set -xe
HOST=control@192.168.1.104
bin2srec -a 0x8004000 -i ./build/acquisinator.bin -o ./binaries/acquisinator31.srec
# scp ./binaries/acquisinator31.srec ${HOST}:/home/control/can-flashing/binaries