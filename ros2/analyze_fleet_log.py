#!/usr/bin/env python3
"""Analyze fleet-brain DRIVE_CSV (20 Hz paired cmd + IMU gz)."""
from __future__ import annotations

import argparse
import csv
import math
import sys


def load(path: str) -> list[dict]:
    rows: list[dict] = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append(
                {
                    't': float(row['t']),
                    'lin': float(row['lin']),
                    'ang': float(row['ang']),
                    'gz': float(row['gz']),
                    'range_m': float(row['range_m']) if row['range_m'] else float('nan'),
                    'mode': row['mode'],
                    'pan': float(row['pan']),
                }
            )
    return rows


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('csv')
    p.add_argument('--gyro-sign', type=float, default=-1.0)
    args = p.parse_args()

    rows = load(args.csv)
    if len(rows) < 10:
        print(f'Too few samples ({len(rows)})', file=sys.stderr)
        sys.exit(1)

    t0, t1 = rows[0]['t'], rows[-1]['t']
    dur = t1 - t0

    yaw = 0.0
    yaw_series = [0.0]
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        if 0 < dt < 0.15:
            yaw += math.degrees(a['gz'] * args.gyro_sign) * dt
        yaw_series.append(yaw)

    total_deg = abs(yaw_series[-1] - yaw_series[0])
    gz_phys = [r['gz'] * args.gyro_sign for r in rows]
    gz_abs = sorted(abs(g) for g in gz_phys)

    d_ang = [abs(rows[i]['ang'] - rows[i - 1]['ang']) for i in range(1, len(rows))]
    d_lin = [abs(rows[i]['lin'] - rows[i - 1]['lin']) for i in range(1, len(rows))]
    big_steps = sum(1 for i in range(1, len(rows)) if d_lin[i - 1] > 0.12 or d_ang[i - 1] > 0.15)

    pivots = sum(1 for r in rows if abs(r['lin']) < 0.05 and abs(r['ang']) > 0.4)
    arcs = sum(1 for r in rows if r['lin'] > 0.1 and abs(r['ang']) > 0.12)
    avoids = sum(1 for r in rows if r['lin'] > 0.1 and abs(r['ang']) > 0.2 and r['range_m'] < 0.45)

    # cmd_ang vs gz when moving forward
    fwd = [r for r in rows if r['lin'] > 0.08]
    if fwd:
        err = sorted(abs(r['ang'] - r['gz'] * args.gyro_sign) for r in fwd)

    print(f'=== Fleet drive log: {args.csv} ===')
    print(f'Duration:     {dur:.1f}s  samples: {len(rows)} ({len(rows)/dur:.1f} Hz)')
    print()
    print('--- Rotation ---')
    print(f'IMU integrated heading change: {total_deg:.0f}° ({total_deg/360:.2f} turns)')
    print(f'|gz| median / p95: {gz_abs[len(gz_abs)//2]:.3f} / {gz_abs[int(len(gz_abs)*0.95)]:.3f} rad/s')
    print(f'High spin samples: {sum(1 for g in gz_abs if g > 0.35)} (>0.35 rad/s)')
    print()
    print('--- Jerk (command steps @ 20Hz) ---')
    print(f'|Δang| max: {max(d_ang):.3f}  |Δlin| max: {max(d_lin):.3f}')
    print(f'Big steps: {big_steps}  (Δlin>0.12 or Δang>0.15 between ticks)')
    print(f'Pivot cmds: {pivots}  forward+steer arcs: {arcs}')
    print()
    print('--- Commands ---')
    lins = [r['lin'] for r in rows]
    angs = [r['ang'] for r in rows]
    print(f'lin min/mean/max: {min(lins):+.2f} / {sum(lins)/len(lins):+.2f} / {max(lins):+.2f}')
    print(f'ang min/mean/max: {min(angs):+.3f} / {sum(angs)/len(angs):+.3f} / {max(angs):+.3f}')
    if fwd:
        print(f'fwd |cmd_ang - gz| median: {err[len(err)//2]:.3f} rad/s')
    modes = {}
    for r in rows:
        modes[r['mode']] = modes.get(r['mode'], 0) + 1
    print(f'modes: {modes}')
    print()
    print('--- Sample timeline (every ~5s) ---')
    last = t0
    for r in rows:
        if r['t'] - last >= 5.0:
            print(
                f"  t={r['t']-t0:5.1f}s  lin={r['lin']:+.2f} ang={r['ang']:+.3f} "
                f"gz={r['gz']*args.gyro_sign:+.3f} rng={r['range_m']:.2f}m {r['mode']}"
                if not math.isnan(r['range_m'])
                else f"  t={r['t']-t0:5.1f}s  lin={r['lin']:+.2f} ang={r['ang']:+.3f} "
                f"gz={r['gz']*args.gyro_sign:+.3f} rng=nan {r['mode']}"
            )
            last = r['t']


if __name__ == '__main__':
    main()
