#! /bin/env bash

set -xe

HOST=control@169.254.81.205

bin2srec -a 0x8004000 -i ./build/acquisinator.bin -o ./build/acquisinator.srec
scp build/acquisinator.srec ${HOST}:/home/control/can-flashing/binaries
