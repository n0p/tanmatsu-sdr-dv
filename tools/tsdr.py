#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Host side of the Tanmatsu SDR development console (USB-Serial-JTAG of the ESP32-P4).

  tsdr.py cmd stat                 send a command, print the '#' replies
  tsdr.py log 5                    print everything the device says for 5 s
  tsdr.py cap ch 2000 -o x.cs16    capture 2000 ms of a tap (raw|if|ch|fm|audio) to a file
  tsdr.py cap fm 3000 --trig -40   ... starting when the channel power exceeds -40 dBFS
  tsdr.py shot screen.png          screenshot of the device display
  tsdr.py cmd ls                   the app's files on the SD card: recordings, message logs, screenshots
  tsdr.py get <name> [-o file]     fetch one of them (the name as "ls" prints it, with its folder)

Captures get a .json sidecar with rate / format / frequency.
"""
import argparse
import base64
import glob
import json
import os
import struct
import sys
import time
import zlib

import serial

C6_SERIAL = "7C:2C:67:46:E1:18"  # the radio coprocessor's port: never ours


def find_port():
    ports = [p for p in glob.glob("/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*") if C6_SERIAL not in p]
    if not ports:
        sys.exit("no ESP32-P4 console port found (is the app running?)")
    return ports[0]


def open_port(port):
    s = serial.Serial()
    s.port = port or find_port()
    s.baudrate = 115200
    # DTR/RTS drive the chip's reset / boot straps. RTS alone means "reset", so the lines must
    # never differ: leave both asserted (the kernel raises and drops them together) and do not
    # let pyserial release them one after the other.
    s.dtr = True
    s.rts = True
    s.timeout = 0.2
    s.open()
    return s


def read_lines(s, deadline):
    buf = b""
    while time.time() < deadline():
        chunk = s.read(65536)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield line.rstrip(b"\r").decode("utf-8", "replace")


def send(s, text):
    # Terminate whatever is sitting in the device's line buffer first (boot log lines echoed back
    # by the host tty before the port was switched to raw mode end up there).
    s.write(b"\n")
    s.flush()
    time.sleep(0.15)
    s.reset_input_buffer()
    s.write((text + "\n").encode())
    s.flush()


def cmd(args):
    s = open_port(args.port)
    send(s, " ".join(args.words))
    last = [time.time()]
    got = False
    for line in read_lines(s, lambda: last[0] + (args.wait if not got else 0.4)):
        if line.startswith("#"):
            print(line)
            got = True
            last[0] = time.time()
        elif args.all:
            print(line)


def log(args):
    s = open_port(args.port)
    end = time.time() + args.seconds
    for line in read_lines(s, lambda: end):
        print(line)


def receive_blob(s, kind, timeout):
    """Returns (header dict, bytes). Raises on protocol or CRC errors."""
    last = [time.time()]
    header = None
    chunks = []
    for line in read_lines(s, lambda: last[0] + timeout):
        last[0] = time.time()
        if header is None:
            if line.startswith("#ERR"):
                raise RuntimeError(line)
            if line.startswith("#%s BEGIN" % kind):
                header = dict(kv.split("=", 1) for kv in line.split()[2:] if "=" in kv)
            elif line.startswith("#"):
                print(line, file=sys.stderr)
            continue
        if line.startswith("#%s END" % kind):
            data = b"".join(chunks)
            crc = int(line.split("crc32=")[1], 16)
            if len(data) != int(header["bytes"]):
                raise RuntimeError("short payload: %d of %s bytes" % (len(data), header["bytes"]))
            if zlib.crc32(data) & 0xFFFFFFFF != crc:
                raise RuntimeError("CRC mismatch")
            return header, data
        if line and not line.startswith("#"):
            chunks.append(base64.b64decode(line))
    raise RuntimeError("timed out waiting for %s data" % kind)


def cap(args):
    s = open_port(args.port)
    line = "cap %s %d" % (args.tap, args.ms)
    wait = 10.0
    if args.trig is not None:
        line += " trig %g %d" % (args.trig, args.wait)
        wait = args.wait + 10.0
    send(s, line)
    header, data = receive_blob(s, "CAP", wait + args.ms / 1000.0)
    with open(args.out, "wb") as f:
        f.write(data)
    with open(args.out + ".json", "w") as f:
        json.dump(header, f, indent=1)
    print("%s: %d bytes %s" % (args.out, len(data), header))


def write_png(path, rgb, width, height):
    def chunk(tag, payload):
        return struct.pack(">I", len(payload)) + tag + payload + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    rows = b"".join(b"\0" + rgb[y * width * 3:(y + 1) * width * 3] for y in range(height))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(rows, 6)))
        f.write(chunk(b"IEND", b""))


def shot(args):
    import numpy as np

    s = open_port(args.port)
    send(s, "shot")
    header, data = receive_blob(s, "SHOT", 10.0)
    raw_w, raw_h = int(header["w"]), int(header["h"])
    # Panel memory is raw_h columns of raw_w pixels (B,G,R); logical (x, y) = raw[x][raw_w-1-y].
    raw = np.frombuffer(data, dtype=np.uint8).reshape(raw_h, raw_w, 3)
    img = raw[:, ::-1, ::-1].transpose(1, 0, 2)
    write_png(args.out, np.ascontiguousarray(img).tobytes(), raw_h, raw_w)
    print("%s: %dx%d" % (args.out, raw_h, raw_w))


def get(args):
    """Fetches a file from the app's folder on the SD card (see "cmd ls")."""
    s = open_port(args.port)
    send(s, "get " + args.name)
    header, data = receive_blob(s, "FILE", 20.0)
    out = args.out or os.path.basename(args.name)
    with open(out, "wb") as f:
        f.write(data)
    print("%s: %d bytes" % (out, len(data)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    sub = ap.add_subparsers(dest="action", required=True)
    p = sub.add_parser("cmd")
    p.add_argument("words", nargs="+")
    p.add_argument("--wait", type=float, default=3.0)
    p.add_argument("--all", action="store_true", help="also print log lines")
    p.set_defaults(fn=cmd)
    p = sub.add_parser("log")
    p.add_argument("seconds", type=float, nargs="?", default=5.0)
    p.set_defaults(fn=log)
    p = sub.add_parser("cap")
    p.add_argument("tap", choices=["raw", "if", "ch", "fm", "audio"])
    p.add_argument("ms", type=int)
    p.add_argument("-o", "--out", required=True)
    p.add_argument("--trig", type=float)
    p.add_argument("--wait", type=int, default=60, help="seconds to wait for the trigger")
    p.set_defaults(fn=cap)
    p = sub.add_parser("shot")
    p.add_argument("out")
    p.set_defaults(fn=shot)
    p = sub.add_parser("get")
    p.add_argument("name")
    p.add_argument("-o", "--out")
    p.set_defaults(fn=get)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
