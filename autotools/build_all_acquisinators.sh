#! /bin/sh

rm -r binaries && mkdir binaries

# TODO: use this instead of set_acquisinator_id.py
# sed -Ei "s/#define ACQUISINATOR_ID \(ACQUISINATOR_ID_[0-9]+\)/#define ACQUISINATOR_ID \(ACQUISINATOR_ID_${id}\)/"

for iacquisinatore in {1..31}
do
    printf "\n\n --- BUILDING ACQUISINATOR ${iacquisinatore} --- \n\n"
    ./autotools/set_acquisinator_id.py ${iacquisinatore}
    make -j8 -f STM32Make.make >/dev/null && ./autotools/flash_via_tlm.sh || printf "\033[0;31m\n\n --- FAILING ACQUISINATOR ${iacquisinatore} --- \n\n\033[0m"
done


