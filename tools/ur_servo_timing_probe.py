"""Stage 0a for UR5e joint trajectories: timing only, no motion.

Read-only. Opens an RTDE *receive* interface (like the observer does) and
never the control interface, so it cannot move the arm or take the
controller's program slot; run it with the service up, under its normal
load. Measures:

1. the receive stream: how often the controller timestamp advances when
   polled, and the gaps between new packets;
2. the jitter of an absolute-deadline loop at each rate, paced exactly as
   the trajectory executor paces itself (sleep in slices of at most 5 ms,
   GC disabled), reading the newest packet on every tick.

The controller's answer time for a control call needs the owned control
interface, so it is measured in stage 0b with the session open.

    python tools/ur_servo_timing_probe.py --host 192.168.254.49 --seconds 10 --out probe.json

See src/docs/UR5E_TRAJECTORY_PLAN.md (section 3).
"""

from __future__ import annotations

import argparse
import gc
import json
import platform
import statistics
import sys
import time


def pct(values, p):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(p / 100 * (len(ordered) - 1))))]


def ms(value):
    return None if value is None else round(value * 1000, 3)


def receive_stream(rtde, seconds):
    """Poll as fast as possible; record when the controller timestamp moves."""
    gaps, last_ts, last_new, polls = [], None, None, 0
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        ts = rtde.getTimestamp()
        now = time.perf_counter()
        polls += 1
        if ts != last_ts:
            if last_new is not None:
                gaps.append(now - last_new)
            last_ts, last_new = ts, now
        time.sleep(0.0002)
    return {
        "polls": polls,
        "packets": len(gaps) + 1,
        "packet_rate_hz": round(len(gaps) / seconds, 1),
        "gap_ms": {"p50": ms(pct(gaps, 50)), "p99": ms(pct(gaps, 99)), "max": ms(max(gaps) if gaps else None)},
    }


def deadline_loop(rtde, rate, seconds):
    """The executor's pacing: absolute deadlines, sleep slices <= 5 ms."""
    dt = 1.0 / rate
    lateness, reads, stale = [], [], 0
    last_ts = None
    gc_was = gc.isenabled()
    gc.disable()
    try:
        start = time.perf_counter() + 0.05
        n = int(seconds * rate)
        for i in range(n):
            deadline = start + i * dt
            while True:
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.005))
            woke = time.perf_counter()
            lateness.append(woke - deadline)
            ts = rtde.getTimestamp()
            q = rtde.getActualQ()
            reads.append(time.perf_counter() - woke)
            if ts == last_ts:
                stale += 1
            last_ts = ts
            if len(q) != 6:
                raise RuntimeError("getActualQ did not return six joints")
    finally:
        if gc_was:
            gc.enable()
    over = lambda k: sum(1 for v in lateness if v > k * dt)  # noqa: E731
    return {
        "rate_hz": rate,
        "ticks": len(lateness),
        "late_ms": {
            "mean": ms(statistics.fmean(lateness)),
            "p50": ms(pct(lateness, 50)),
            "p99": ms(pct(lateness, 99)),
            "p999": ms(pct(lateness, 99.9)),
            "max": ms(max(lateness)),
        },
        "late_over_1_period": over(1),
        "late_over_5_periods": over(5),  # the executor's constrained-stop threshold
        "read_ms": {"p50": ms(pct(reads, 50)), "p99": ms(pct(reads, 99)), "max": ms(max(reads))},
        "same_packet_ticks": stale,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", required=True)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--rates", default="125,250,500")
    parser.add_argument("--frequency", type=float, default=500.0, help="RTDE receive frequency, Hz")
    parser.add_argument("--out")
    args = parser.parse_args()

    import rtde_receive

    rtde = rtde_receive.RTDEReceiveInterface(args.host, args.frequency, ["timestamp", "actual_q", "robot_mode", "safety_mode"])
    try:
        result = {
            "probe": "ur_servo_timing_probe",
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "host": platform.node(),
            "python": sys.version.split()[0],
            "platform": platform.platform(),
            "robot": args.host,
            "receive_frequency_hz": args.frequency,
            "robot_mode": rtde.getRobotMode(),
            "safety_mode": rtde.getSafetyMode(),
            "receive": receive_stream(rtde, min(args.seconds, 5.0)),
            "loops": [deadline_loop(rtde, float(r), args.seconds) for r in args.rates.split(",")],
        }
    finally:
        rtde.disconnect()
    text = json.dumps(result, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(text)
    print(text)


if __name__ == "__main__":
    main()
