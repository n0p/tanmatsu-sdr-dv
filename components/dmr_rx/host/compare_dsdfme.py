#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Compare dmr_rx_file against a dsd-fme log of the same recording.

usage: compare_dsdfme.py [--bin build/dmr_rx_file] --wav DIR --ref DIR [name ...]

For every DIR/<name>.wav this runs `dmr_rx_file -v` and parses
REF/<name>.dsd-fme.log (dsd-fme -fs [-xr] -i <wav> -o null -Z). It prints, per
recording: colour code, burst counts per data type (CRC/FEC-good ones), voice
frame counts, how many AMBE frames (49 bit, after FEC) are bit-identical and in
the same order in both decoders, and the link-control address sets.

Run it with `python3 -I` when the logs come from an untrusted place.
"""
import argparse
import collections
import difflib
import os
import re
import subprocess
import sys

FME_TYPES = {
    "VLC": "Voice LC Header", "TLC": "Terminator with LC", "PI": "PI Header", "CSBK": "CSBK",
    "IDLE": "Idle", "DATA": "Data Header", "R12U": "Rate 1/2 Data", "R12C": "Rate 1/2 Data",
    "R34U": "Rate 3/4 Data", "R34C": "Rate 3/4 Data", "R_1U": "Rate 1 Data", "R_1C": "Rate 1 Data",
    "MBCH": "MBC Header", "MBCC": "MBC Continuation", "USBD": "Unified Single Block Data",
}


def parse_fme(path):
    r = dict(cc=collections.Counter(), types=collections.Counter(), bad=collections.Counter(),
             ambe=[], ids=collections.Counter(), csbk=collections.Counter(), voice_slot=collections.Counter(),
             nosync=0, slots=collections.Counter())
    want_csbk = False
    slot = 1
    last = None  # (slot, type) of the last burst counted as good
    with open(path, "r", errors="replace") as f:
        for line in f:
            if "Sync:" not in line and last and ("CRC ERR" in line or "FEC ERR" in line or "Protected LC" in line):
                # dsd-fme prints the burst type first and the failed RS / CRC check of its PDU on a following line
                r["types"][last[1]] -= 1
                r["slots"][last] -= 1
                r["bad"][last[1]] += 1
                last = None
                want_csbk = False
                continue
            if "Sync:" in line:
                want_csbk = False
                last = None
                if "no sync" in line:
                    r["nosync"] += 1
                    continue
                m = re.search(r"\[(slot|SLOT)([12])\]", line)
                slot = int(m.group(2)) if m else 1
                parts = [p.strip() for p in line.split("|")]
                m = re.search(r"Color Code=(\d+)", line)
                tail = parts[-1]
                err = "ERR" in tail or "Err" in tail
                t = tail.split()[0] if tail else ""
                if m and not err:
                    r["cc"][int(m.group(1))] += 1
                if t.startswith("VC"):
                    continue
                if t in FME_TYPES:
                    if err:
                        r["bad"][FME_TYPES[t]] += 1
                    else:
                        r["types"][FME_TYPES[t]] += 1
                        r["slots"][(slot, FME_TYPES[t])] += 1
                        last = (slot, FME_TYPES[t])
                        want_csbk = t == "CSBK"
                else:
                    r["bad"]["(undecoded)"] += 1
                continue
            m = re.match(r"\s*AMBE ([0-9A-F]{14}) err = \[(\d+)\] \[(\d+)\]", line)
            if m:
                r["ambe"].append(m.group(1))
                r["voice_slot"][slot] += 1
                continue
            m = re.search(r"TGT=(\d+) SRC=(\d+) FLCO=0x([0-9A-F]+) FID=0x([0-9A-F]+) SVC=0x([0-9A-F]+)", line)
            if m:
                r["ids"][(int(m.group(1)), int(m.group(2)), int(m.group(3), 16), int(m.group(4), 16),
                          int(m.group(5), 16))] += 1
                continue
            if want_csbk and "DMR PDU Payload" in line:
                b = re.findall(r"\[([0-9A-F]{2})\]", line)
                if len(b) >= 2:
                    r["csbk"][(int(b[0], 16) & 0x3F, int(b[1], 16))] += 1
                want_csbk = False
    return r


def parse_mine(text):
    r = dict(cc=collections.Counter(), types=collections.Counter(), bad=collections.Counter(),
             ambe=[], ids=collections.Counter(), csbk=collections.Counter(), voice_slot=collections.Counter(),
             slots=collections.Counter(), summary=[], enc_calls=0, calls=0)
    in_sum = False
    for line in text.splitlines():
        if line.startswith("== summary"):
            in_sum = True
        if in_sum:
            r["summary"].append(line)
            continue
        m = re.match(r"\s*([\d.]+) TS(\d)(.) CC(-?\d+)\s+(\S+)\s*(.*)", line)
        if not m:
            continue
        slot, cc, kind, rest = int(m.group(2)), int(m.group(4)), m.group(5), m.group(6)
        if kind == "DATA":
            mm = re.match(r"(.*?)\s+st=(\d) pl=(\d)", rest)
            name, st, pl = mm.group(1).strip(), int(mm.group(2)), int(mm.group(3))
            if st and cc >= 0:
                r["cc"][cc] += 1
            checked = name in ("Voice LC Header", "Terminator with LC", "PI Header", "CSBK", "Data Header",
                               "MBC Header", "MBC Continuation", "Idle", "Rate 1/2 Data")
            if st and (pl or not checked):
                r["types"][name] += 1
                r["slots"][(slot, name)] += 1
            else:
                r["bad"][name if st else "(undecoded)"] += 1
        elif kind == "VOICE":
            mm = re.search(r"ambe=([0-9A-F]{14}),([0-9A-F]{14}),([0-9A-F]{14})", rest)
            r["ambe"].extend(mm.groups())
            r["voice_slot"][slot] += 3
            if cc >= 0:
                r["cc"][cc] += 1
        elif kind == "LC":
            mm = re.search(r"FID=([0-9A-F]{2}) svc=([0-9A-F]{2}) (TG|ID) dst=(\d+) src=(\d+).*\[([0-9A-F]{18})\]", rest)
            if mm:
                flco = int(mm.group(6)[:2], 16) & 0x3F
                r["ids"][(int(mm.group(4)), int(mm.group(5)), flco, int(mm.group(1), 16),
                          int(mm.group(2), 16))] += 1
        elif kind == "CSBK":
            mm = re.search(r"op=([0-9A-F]{2}) FID=([0-9A-F]{2})", rest)
            r["csbk"][(int(mm.group(1), 16), int(mm.group(2), 16))] += 1
        elif kind == "CALL_START":
            r["calls"] += 1
        if kind in ("CALL_START", "CALL_UPDATE", "CALL_END") and "ENCRYPTED" in rest and kind == "CALL_END":
            r["enc_calls"] += 1
    return r


def common_in_order(a, b):
    sm = difflib.SequenceMatcher(None, a, b, autojunk=False)
    return sum(blk.size for blk in sm.get_matching_blocks())


def voice_ids(c):
    """(dst, src) of group / unit-to-unit voice LCs (FLCO 0 and 3)."""
    return sorted({(k[0], k[1]) for k in c if k[2] in (0, 3)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "dmr_rx_file"))
    ap.add_argument("--wav", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--details", action="store_true")
    ap.add_argument("names", nargs="*")
    a = ap.parse_args()
    names = a.names or sorted(f[:-4] for f in os.listdir(a.wav) if f.endswith(".wav"))
    order = ["Voice LC Header", "Terminator with LC", "PI Header", "CSBK", "Data Header", "Rate 1/2 Data",
             "Rate 3/4 Data", "Idle"]
    short = ["VLC", "TLC", "PI", "CSBK", "DHDR", "R12", "R34", "IDLE"]
    print("columns are dsd-fme/dmr_rx. AMBE: frames decoded by each, 'same' = bit-identical 49 bit frames in the "
          "same order.")
    hdr = "%-24s %-7s %-22s %-11s " % ("recording", "CC", "AMBE frames (same)", "voice TS") + " ".join(
        "%-9s" % s for s in short) + " ids"
    print(hdr)
    worse = 0
    for n in names:
        ref = os.path.join(a.ref, n + ".dsd-fme.log")
        wav = os.path.join(a.wav, n + ".wav")
        if not os.path.exists(ref):
            print("%-24s (no reference log)" % n)
            continue
        out = subprocess.run([a.bin, "-v", wav], capture_output=True, text=True).stdout
        f, m = parse_fme(ref), parse_mine(out)
        fcc = f["cc"].most_common(1)[0][0] if f["cc"] else -1
        mcc = ",".join(str(k) for k, v in sorted(m["cc"].items()) if v >= 3) or "-"
        same = common_in_order(f["ambe"], m["ambe"])
        fv = "+".join(str(k) for k in sorted(f["voice_slot"])) or "-"
        mv = "+".join(str(k) for k in sorted(m["voice_slot"])) or "-"
        cols = " ".join("%-9s" % ("%d/%d" % (f["types"][t], m["types"][t])) for t in order)
        fi, mi = voice_ids(f["ids"]), voice_ids(m["ids"])
        ids = "same" if fi == mi else "DIFF fme=%s mine=%s" % (fi, mi)
        if fi:
            ids += " " + ";".join("%d>%d" % (s, d) for d, s in mi)
        print("%-24s %-7s %-22s %-11s %s %s" % (n, "%d/%s" % (fcc, mcc), "%d/%d (%d)" % (
            len(f["ambe"]), len(m["ambe"]), same), "%s/%s" % (fv, mv), cols, ids))
        if len(m["ambe"]) < len(f["ambe"]) or any(m["types"][t] < f["types"][t] for t in order):
            worse += 1
        if a.details:
            print("    fme  bad:", dict(f["bad"]), "no-sync:", f["nosync"])
            print("    mine bad:", dict(m["bad"]))
            print("    fme  csbk:", {("%02X/%02X" % k): v for k, v in sorted(f["csbk"].items())})
            print("    mine csbk:", {("%02X/%02X" % k): v for k, v in sorted(m["csbk"].items())})
            print("    fme  lc:", dict(f["ids"]))
            print("    mine lc:", dict(m["ids"]))
    print("recordings where dmr_rx decoded fewer good bursts of some type than dsd-fme: %d" % worse)
    return 0


if __name__ == "__main__":
    sys.exit(main())
