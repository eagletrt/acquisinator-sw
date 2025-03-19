#! /bin/sh

rm -r binaries && mkdir binaries

for iacquisinatore in {1..31}
do
    printf "\n\n --- BUILDING ACQUISINATOR ${iacquisinatore} --- \n\n"
    sed -Ei "s/#define ACQUISINATOR_ID \(ACQUISINATOR_ID_[0-9]+\)/#define ACQUISINATOR_ID \(ACQUISINATOR_ID_${iacquisinatore}\)/" ./Core/Inc/acquisinator_id.h
    rm -fR ./build
    make -j16 DEBUG=1 -f ./STM32Make.make
    bin2srec -a 0x8004000 -i ./build/debug/acquisinator.bin -o ./binaries/acquisinator${iacquisinatore}.srec
done

#read -p "Press enter to copy files to raspberry, make sure you're connected to fenice local network, this will overwrite the files in the raspberry. Press CTRL+C to cancel."
