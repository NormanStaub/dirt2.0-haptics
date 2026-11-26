"""
Replay a recorded stage into the running bridge, so you can feel haptics
changes on a real stage without driving it again.

  python replay.py                         newest recording, whole stage
  python replay.py --start 150 --len 60    a section (seconds into the stage)
  python replay.py recordings\\stage_X.bin

Pause DiRT (or sit in the menus) while replaying, so the two don't mix.
"""

import argparse
import glob
import os
import socket
import struct
import time


def load(path):
    raw = open(path, "rb").read()
    out, i = [], 0
    while i + 12 <= len(raw):
        t, n = struct.unpack_from("<dI", raw, i)
        i += 12
        if i + n > len(raw):
            break
        out.append((t, raw[i:i + n]))
        i += n
    # drop stationary packets at the start (menus / start line)
    moving = [k for k, (_, d) in enumerate(out)
              if len(d) >= 32 and struct.unpack_from("<f", d, 28)[0] > 1.0]
    return out[moving[0]:] if moving else out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser()
    p.add_argument("file", nargs="?")
    p.add_argument("--start", type=float, default=0.0, help="seconds into the stage")
    p.add_argument("--len", type=float, default=0.0, help="seconds to play (0 = to the end)")
    p.add_argument("--port", type=int, default=20777)
    a = p.parse_args()

    path = a.file or max(glob.glob(os.path.join(here, "recordings", "*.bin")), key=os.path.getmtime)
    pk = load(path)
    t0 = pk[0][0]
    pk = [(t - t0, d) for t, d in pk if t - t0 >= a.start and (not a.len or t - t0 < a.start + a.len)]
    if not pk:
        raise SystemExit("nothing in that range")
    print(f"replaying {os.path.basename(path)}  {pk[0][0]:.0f}s -> {pk[-1][0]:.0f}s")

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.perf_counter() - pk[0][0]
    next_mark = pk[0][0]
    for t, d in pk:
        while (wait := t - (time.perf_counter() - start)) > 0:
            time.sleep(min(wait, 0.005))
        s.sendto(d, ("127.0.0.1", a.port))
        if t >= next_mark:
            speed = struct.unpack_from("<f", d, 28)[0] * 3.6
            print(f"  {t:5.0f}s  {speed:4.0f} km/h", flush=True)
            next_mark += 10
    print("done")


if __name__ == "__main__":
    main()
