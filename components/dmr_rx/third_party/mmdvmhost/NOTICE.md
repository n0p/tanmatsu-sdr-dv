# MMDVMHost (DMR FEC and PDU classes)

- Upstream: https://github.com/g4klx/MMDVMHost
- Commit: 6c849abfec42ab42513fe43f3a04342764a1b18e
- Licence: GPL-2.0-or-later (see `LICENCE`; every vendored file carries the
  "either version 2 of the License, or (at your option) any later version"
  header). Used here under AGPL-3.0 via GPL-3.0.
- Copyright (C) Jonathan Naylor G4KLX and the contributors named in the files.

## Files

Unmodified copies:

`CRC.cpp` `CRC.h` `DMRDefines.h` `DMREMB.cpp` `DMREMB.h` `DMRLC.cpp` `DMRLC.h`
`DMRSlotType.cpp` `DMRSlotType.h` `Golay2087.cpp` `Golay2087.h` `Hamming.cpp`
`QR1676.cpp` `QR1676.h` `RS129.cpp` `RS129.h` `LICENCE`

Modified (each change is marked "Modified for dmr_rx" in the file):

| File | Change |
|---|---|
| `Defines.h` | Trimmed to what the vendored classes need. Only `USE_DMR` is defined. Adds `#define`s that prefix the class names with `DmrRx_` so that another component vendoring MMDVMHost cannot collide at link time. |
| `Hamming.h` | Includes `Defines.h` (class name prefix). |
| `BPTC19696.h` `BPTC19696.cpp` | Work buffers are fixed member arrays instead of `new[]` (no heap). |
| `DMRShortLC.h` `DMRShortLC.cpp` | Work buffers are fixed member arrays instead of `new[]`. `decodeErrorCheck()`: one mismatch in the column parity row is accepted (it is a bit error in the parity row itself, the Hamming rows are already corrected; the caller checks the CRC-8 of the payload). Upstream rejects the block. |
| `DMREmbeddedData.h` `DMREmbeddedData.cpp` | Work buffers are fixed member arrays instead of `new[]`. `getLC()` fills a caller-provided `CDMRLC` and returns `bool` instead of returning a heap object. `decodeEmbeddedData()`: one mismatch in the column parity row is accepted (the 5 bit checksum is verified afterwards). Upstream rejects the block. |
| `DMRFullLC.h` `DMRFullLC.cpp` | `decode()` fills a caller-provided `CDMRLC` and returns `bool` instead of returning a heap object. |

Not from MMDVMHost (written for dmr_rx, MIT):

| File | Why |
|---|---|
| `Log.h` | No-op replacement for upstream `Log.h` (which pulls in `std::string` and a JSON library). |
| `Utils.h` | Upstream `Utils.h` / `Utils.cpp` are "GPL version 2" only (no "or later") and therefore not usable in an AGPL-3.0 application. This is an independent replacement with the two bit/byte helpers the vendored classes call. |

The encoder halves of these classes are not used by the receiver; the host
test signal generator (`host/dmr_gen_lib.cpp`) uses them to build bursts.
