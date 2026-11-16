# DSD (Digital Speech Decoder) - DMR AMBE interleave schedule

- Upstream: https://github.com/szechyjs/dsd
- Commit: 59423fa46be8b41ef0bd2f3d2b45590600be29f0
- Licence: ISC (see `COPYRIGHT`)
- Copyright (C) 2010 DSD Author

## Files

| File | Change |
|---|---|
| `dmr_const.h` | Unmodified copy of `include/dmr_const.h`: the tables `rW`, `rX`, `rY`, `rZ` that map the 72 bits of an AMBE+2 voice frame on the air to the four code words handed to mbelib. Included once, with `_MAIN` defined, by `src/dmr_fec.cpp`. |
| `COPYRIGHT` | Unmodified. |

Nothing else of DSD is used.
