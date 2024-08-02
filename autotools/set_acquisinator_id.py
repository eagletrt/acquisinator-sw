#! /bin/python

import sys

ID_HEADER_FILE_PATH="Core/Inc/acquisinator_id.h"
FLASH_VIA_CAN_FILE_PATH="autotools/flash_via_can.sh"
FLASH_VIA_TLM_FILE_PATH="autotools/flash_via_tlm.sh"


if __name__ == "__main__":
    argc = len(sys.argv)
    if argc != 2:
        print("Usage: $0 <acquisinator-number> (must be greater than 0, e.g. 1, 5 or 18)")
        
    # Header file generation
    HEADER=f'''#ifndef ACQUISINATOR_ID_H
#define ACQUISINATOR_ID_H

// #define ACQUISINATOR_ID_0  (0U) // SHOULD NOT BE USED!!! IDs START FROM 1!!!
#define ACQUISINATOR_ID_1  (1U)
#define ACQUISINATOR_ID_2  (2U)
#define ACQUISINATOR_ID_3  (3U)
#define ACQUISINATOR_ID_4  (4U)
#define ACQUISINATOR_ID_5  (5U)
#define ACQUISINATOR_ID_6  (6U)
#define ACQUISINATOR_ID_7  (7U)
#define ACQUISINATOR_ID_8  (8U)
#define ACQUISINATOR_ID_9  (9U)
#define ACQUISINATOR_ID_10 (10U)
#define ACQUISINATOR_ID_11 (11U)
#define ACQUISINATOR_ID_12 (12U)
#define ACQUISINATOR_ID_13 (13U)
#define ACQUISINATOR_ID_14 (14U)
#define ACQUISINATOR_ID_15 (15U)
#define ACQUISINATOR_ID_16 (16U)
#define ACQUISINATOR_ID_17 (17U)
#define ACQUISINATOR_ID_18 (18U)
#define ACQUISINATOR_ID_19 (19U)
#define ACQUISINATOR_ID_20 (20U)
#define ACQUISINATOR_ID_21 (21U)
#define ACQUISINATOR_ID_22 (22U)
#define ACQUISINATOR_ID_23 (23U)
#define ACQUISINATOR_ID_24 (24U)
#define ACQUISINATOR_ID_25 (25U)
#define ACQUISINATOR_ID_26 (26U)
#define ACQUISINATOR_ID_27 (27U)
#define ACQUISINATOR_ID_28 (28U)
#define ACQUISINATOR_ID_29 (29U)
#define ACQUISINATOR_ID_30 (30U)
#define ACQUISINATOR_ID_31 (31U)

#define ACQUISINATOR_ID (ACQUISINATOR_ID_{sys.argv[1]})

#if ACQUISINATOR_ID == 0
#error INVALID ACQUISINATOR_ID == 0
#endif

#if ACQUISINATOR_ID == ACQUISINATOR_ID_0
#error INVALID ACQUISINATOR_ID == ACQUISINATOR_ID_0
#endif

#endif // ACQUISINATOR_ID_H'''
    try:
        outfile = open(ID_HEADER_FILE_PATH, 'w+')
    except FileNotFoundError:
        print("File not found", ID_HEADER_FILE_PATH)
    if outfile.write(HEADER) > 0:
        print("successfully written header " + ID_HEADER_FILE_PATH)
    else:
        print("failed to write to file", ID_HEADER_FILE_PATH)
    outfile.close()

    # Flashing via can script generation
    FLASH_VIA_CAN = f'''#! /bin/env bash
bin2srec -a 0x8004000 -i ./build/acquisinator.bin -o ./build/acquisinator.srec
can-flashing ACQUISINATOR_{sys.argv[1]} ./build/acquisinator.srec'''
    try:
        outfile = open(FLASH_VIA_CAN_FILE_PATH, 'w+')
    except FileNotFoundError:
        print("File not found", FLASH_VIA_CAN_FILE_PATH)
    if outfile.write(FLASH_VIA_CAN) > 0:
        print("successfully written header " + FLASH_VIA_CAN_FILE_PATH)
    else:
        print("failed to write to file", FLASH_VIA_CAN_FILE_PATH)
    outfile.close()
    

    # Flashing via can script generation
    FLASH_VIA_TLM = f'''#! /bin/env bash
set -xe
HOST=control@192.168.1.104
bin2srec -a 0x8004000 -i ./build/acquisinator.bin -o ./binaries/acquisinator{sys.argv[1]}.srec
scp ./binaries/acquisinator{sys.argv[1]}.srec ${{HOST}}:/home/control/can-flashing/binaries'''
    try:
        outfile = open(FLASH_VIA_TLM_FILE_PATH, 'w+')
    except FileNotFoundError:
        print("File not found", FLASH_VIA_TLM_FILE_PATH)
    if outfile.write(FLASH_VIA_TLM) > 0:
        print("successfully written " + FLASH_VIA_TLM_FILE_PATH)
    else:
        print("failed to write to file", FLASH_VIA_TLM_FILE_PATH)
    outfile.close()
