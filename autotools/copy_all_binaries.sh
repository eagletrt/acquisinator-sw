#!/bin/bash

# Make sure that you are connected to fenice local network.

set -xe
HOST = control@control.local
mkdir -p backup_binaries
for i in {1..31}
do 
    scp ${HOST}:~/control/can-flashing/binaries/acquisinator${i}.srec ./backup_binaries
    scp ./binaries/acquisinator${i}.srec ${HOST}:~/control/can-flashing/binaries
done
