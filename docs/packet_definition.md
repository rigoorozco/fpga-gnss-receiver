# UART Packet Definition (Phase 2)

All packets are currently fixed to 24 bytes and start with:
- Byte 0: `0xA5`
- Byte 1: `0x5A`
- Byte 2: packet type
- Bytes 3..6: sample counter (LSB-first in this scaffold)
- Byte 23: XOR checksum of bytes 0..22

## Packet Types

## `0x10` Channel Status Packet
- Byte 7: channel index
- Byte 8: PRN (`[5:0]`)
- Byte 9: status flags
  - `[1:0]` tracking state (`00=IDLE, 01=PULLIN, 10=LOCKED`)
  - `[2]` code lock
  - `[3]` carrier lock
  - `[4]` nav bit valid
  - `[5]` nav bit value
- Bytes 10..11: Doppler estimate (signed 16)
- Bytes 12..13: code phase estimate (11-bit packed in 16)

## `0x20` Observables Packet
- Byte 7: valid observables count
- Byte 8: first valid PRN (`[5:0]`)
- Bytes 9..12: observables epoch counter
- Bytes 13..16: first valid corrected pseudorange estimate (unsigned 32, meters)

## `0x30` PVT Packet
- Byte 7: satellites used in last solve
- Bytes 8..11: latitude in `deg * 1e7` (signed 32)
- Bytes 12..15: longitude in `deg * 1e7` (signed 32)
- Bytes 16..19: height in `mm` (signed 32)
- Bytes 20..22: clock bias (upper 24 bits of signed 32 in this scaffold)

## Notes
- This is a Phase 2 bring-up packet format intended for debugging and incremental verification.
- Internal records are intentionally decoupled from UART encoding so packet layout can evolve without redesigning core blocks.
- Simulation-only pre-encoding logs print receiver runtime / epoch tags plus `Lat/Long/Height` in human-readable units.
