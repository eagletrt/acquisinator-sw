import cantools

from gs_usb_bus import open_bus

db = cantools.database.load_file("acquisinator.dbc")
DIAG_ID = db.get_message_by_name("ACQUISINATOR_DIAG").frame_id
BRIDGE_ID = db.get_message_by_name("ACQUISINATOR_BRIDGE").frame_id

bus = open_bus(bitrate=1_000_000)

print("In ascolto (DEBUG: stampo TUTTI i frame)...")
silent = 0
try:
    while True:
        message = bus.recv(timeout=1.0)
        if message is None:
            silent += 1
            print(f"[nessun frame da {silent} s]")
            continue
        silent = 0
        # dump grezzo di qualunque frame
        print(f"RAW id=0x{message.arbitration_id:X} dlc={message.dlc} data={message.data.hex()}")
        if message.arbitration_id == DIAG_ID:
            try:
                d = db.decode_message(message.arbitration_id, message.data)
                poll = {0: "OK", 1: "ERROR", 2: "BUSY", 3: "TIMEOUT"}.get(int(d['adc_poll']), int(d['adc_poll']))
                volts = d['adc_raw'] * 3.29 / 65536
                print(f"   -> v_out = {d['lockin_out'] * 1000:+9.3f} mV   adc_raw = {d['adc_raw']:5.0f} ({volts:.4f} V)   spi = {poll}")
            except Exception as e:
                print(f"   -> decode error: {e}")
        elif message.arbitration_id == BRIDGE_ID:
            try:
                d = db.decode_message(message.arbitration_id, message.data)
                flags = int(d['flags'])
                notes = [name for bit, name in ((0, "SATURA BASSO"), (1, "SATURA ALTO"), (2, "ERRORE SPI"), (3, "non calibrato"), (4, "NESSUNA MISURA")) if flags & (1 << bit)]
                print(f"   -> R = {d['resistance']:8.3f} ohm   fase = {d['phase']:+7.2f} deg   #{int(d['seq']):3d}   {', '.join(notes)}")
            except Exception as e:
                print(f"   -> decode error: {e}")
except KeyboardInterrupt:
    pass
finally:
    bus.shutdown()
