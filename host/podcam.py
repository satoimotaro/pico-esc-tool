#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""podcam — WHAT IS ACTUALLY MOVING, seen rather than inferred.

WHY THIS EXISTS. On 2026-08-15 the encoder said the azimuth axis was turning at 1428 rpm while both
ESCs were commanded and delivering the SAME speed in solid 6-step — which the pod's geometry
(azimuth = (w_A + w_B)/2, so equal-and-opposite holds the angle) says is impossible. Hours of
electrical measurement could not separate "the pod really is spinning" from "the encoder is not on
the azimuth axis", because every instrument in the loop shares the same encoder or the same
telemetry. A camera shares neither.

It does not need a good exposure, and the bench is backlit. It captures a burst of frames while the
motors run and reports the PER-PIXEL TEMPORAL VARIANCE: whatever is moving lights up, whatever is
still does not. A rest burst is taken first as the baseline, so sensor noise and room flicker are
subtracted rather than mistaken for motion.

Read the output like this:
  - the POD BODY lights up          -> the azimuth axis really is rotating; the encoder is honest.
  - only a PROPELLER DISC lights up -> the pod is still and the encoder is reading the prop shaft,
                                       which would explain every "impossible" number on record.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc.link import EscHost                                   # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")


def burst(cap, secs):
    """Grab frames for `secs` and return them stacked as float32 greyscale."""
    frames, t0 = [], time.time()
    while time.time() - t0 < secs:
        ok, f = cap.read()
        if ok:
            frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype(np.float32))
    return np.stack(frames) if frames else None


def var_map(stack):
    """Temporal standard deviation per pixel — the 'what moved' image."""
    return stack.std(axis=0) if stack is not None and len(stack) > 2 else None


def save(img, path, note=""):
    v = img.copy()
    hi = np.percentile(v, 99.5)
    v = np.clip(v / max(hi, 1e-6), 0, 1)
    out = cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    if note:
        cv2.putText(out, note, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.imwrite(path, out)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--idx-a", type=int, default=0)
    p.add_argument("--idx-b", type=int, default=1)
    p.add_argument("--rpm", type=float, default=800.0, help="matched target for BOTH motors")
    p.add_argument("--rpm-b", type=float, default=None, help="override B's target")
    p.add_argument("--slew", type=float, default=2000.0)
    p.add_argument("--secs", type=float, default=3.0, help="seconds of capture while spinning")
    p.add_argument("--rest-secs", type=float, default=2.0, help="baseline burst before arming")
    p.add_argument("--arm-secs", type=float, default=4.5)
    p.add_argument("--settle", type=float, default=1.5, help="after the target, before capturing")
    p.add_argument("--name", default="pod")
    args = p.parse_args()

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        sys.exit(f"camera {args.cam} did not open")
    d = EscHost(args.port)
    os.makedirs(OUT, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")

    print("# baseline burst (motors idle)")
    rest = var_map(burst(cap, args.rest_secs))
    spin = None
    try:
        for i in (args.idx_a, args.idx_b):
            d.cmd(f"arm {i} bidir", timeout=10)
            d.cmd(f"gain {i} slew {args.slew}", timeout=5)
        time.sleep(args.arm_secs)
        rb = args.rpm if args.rpm_b is None else args.rpm_b
        print(f"# spin: A -> {args.rpm:.0f}, B -> {rb:.0f} rpm")
        d.cmd(f"rpm {args.idx_a} {args.rpm}", timeout=5)
        d.cmd(f"rpm {args.idx_b} {rb}", timeout=5)
        time.sleep(args.settle)
        spin = var_map(burst(cap, args.secs))
    finally:
        for _ in range(3):
            for i in (args.idx_a, args.idx_b):
                for c in (f"rpm {i} 0", f"thrust {i} 0"):
                    try:
                        d.cmd(c, timeout=3)
                    except Exception:
                        pass
        for i in (args.idx_a, args.idx_b):
            try:
                d.cmd(f"disarm {i}", timeout=3)
            except Exception:
                pass
        cap.release()

    if rest is None or spin is None:
        sys.exit("not enough frames captured")
    # Subtract the idle baseline so sensor noise and room flicker are not read as motion.
    moved = np.clip(spin - rest, 0, None)
    for img, tag in ((rest, "rest"), (spin, "spin"), (moved, "moved")):
        path = os.path.join(OUT, f"cam_{args.name}_{tag}_{stamp}.png")
        save(img, path, f"{tag}  A={args.rpm:.0f} B={args.rpm if args.rpm_b is None else args.rpm_b:.0f}")
        print(f"# wrote {path}")
    h, w = moved.shape
    print(f"\n# motion energy by image third (rows x cols), baseline-subtracted:")
    for ri, rs in enumerate(("top   ", "middle", "bottom")):
        row = []
        for ci in range(3):
            blk = moved[ri * h // 3:(ri + 1) * h // 3, ci * w // 3:(ci + 1) * w // 3]
            row.append(f"{blk.mean():7.2f}")
        print(f"#   {rs} " + " ".join(row))


if __name__ == "__main__":
    main()
