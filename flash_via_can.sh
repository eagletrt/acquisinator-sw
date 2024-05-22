#! /bin/env bash

bin2srec -a 0x8004000 -i ./build/acquisinator.bin -o ./build/acquisinator.srec
can-flashing ACQUISINATOR_3 ./build/acquisinator.srec
