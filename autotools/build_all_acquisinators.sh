#! /bin/sh

rm -r binaries && mkdir binaries

for iacquisinatore in {1..31}
do
    printf "\n\n --- BUILDING ACQUISINATOR ${iacquisinatore} --- \n\n"
    ./autotools/set_acquisinator_id.py ${iacquisinatore}
    make -j8 -f STM32Make.make >/dev/null && ./autotools/flash_via_tlm.sh
done


