# Phase 2 Control/Status Register Map

All addresses are byte addresses on the assumed simple control bus.

## Control/Config
- `0x00` Control
  - `[0]` core enable
  - `[1]` soft reset pulse
  - `[2]` acquisition rescan pulse
  - `[3]` tracking enable
  - `[4]` UART enable
  - `[5]` nav store enable
  - `[6]` observables enable
  - `[7]` PVT enable
- `0x04` Channel config
  - `[G_NUM_CHANNELS-1:0]` channel enable mask
  - `[15:8]` preferred channel count
- `0x08` PRN range
  - `[5:0]` PRN start
  - `[13:8]` PRN stop
- `0x0C` detection threshold (unsigned 32)
- `0x10` Doppler min (signed 16)
- `0x14` Doppler max (signed 16)
- `0x18` Doppler step (signed 16)

## Status
- `0x40` global status
  - `[0]` acquisition done
  - `[1]` acquisition success
  - `[2]` PVT valid
  - `[3]` UART busy
- `0x44` acquisition result
  - `[5:0]` detected PRN
  - `[26:16]` detected code phase
- `0x48` acquisition Doppler (signed 16)
- `0x4C` channel allocation bitmap + observables count
  - `[G_NUM_CHANNELS-1:0]` allocation bitmap
  - `[23:16]` observables count
- `0x50` channel lock bitmap + last PVT satellites used
  - `[G_NUM_CHANNELS-1:0]` lock bitmap
  - `[23:16]` satellites used
- `0x54` ephemeris-valid PRN bitmap (low 32 PRNs)
- `0x58` last latitude (`deg * 1e7`, signed 32)
- `0x5C` last longitude (`deg * 1e7`, signed 32)
- `0x60` last height (`mm`, signed 32)
- `0x64` last PVT clock bias (signed 32)
