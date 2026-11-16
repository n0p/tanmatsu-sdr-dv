#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Unpack the ETSI EN 300 395-2 reference codec for the tetra_codec component.

  prepare.py <en_30039502v010301p0.zip> <output directory>

Only the C-CODE directory is taken. The files are DOS text with upper-case names: they are
written with lower-case names (the sources include each other that way), without carriage
returns and without the Ctrl-Z end-of-file mark. Nothing else is changed; the adaptations the
code needs to build here are all in src/tetra_codec.c.
"""
import hashlib
import os
import sys
import zipfile

KNOWN_SHA256 = "1fe18c4773c8ccb52ef23ca5b4a0b0841b38d54ddd5d9c7d86eb8b060f132f39"  # V1.3.1


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    zip_path, out_dir = sys.argv[1], sys.argv[2]
    with open(zip_path, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    if digest != KNOWN_SHA256:
        print(f"tetra_codec: {zip_path} is not the archive this was tested with (sha256 {digest})", file=sys.stderr)
    os.makedirs(out_dir, exist_ok=True)
    count = 0
    with zipfile.ZipFile(zip_path) as z:
        for info in z.infolist():
            parts = info.filename.replace("\\", "/").split("/")
            if info.is_dir() or len(parts) != 2 or parts[0].upper() != "C-CODE":
                continue
            name = parts[1].lower()
            if not name.endswith((".c", ".h", ".tab")):
                continue
            data = z.read(info).replace(b"\r", b"").replace(b"\x1a", b"")
            path = os.path.join(out_dir, name)
            try:
                with open(path, "rb") as f:
                    if f.read() == data:
                        count += 1
                        continue  # keep the time stamp: no rebuild
            except OSError:
                pass
            with open(path, "wb") as f:
                f.write(data)
            count += 1
    if count == 0:
        sys.exit(f"tetra_codec: no C-CODE directory in {zip_path}")
    print(f"tetra_codec: {count} reference codec files in {out_dir}")


if __name__ == "__main__":
    main()
