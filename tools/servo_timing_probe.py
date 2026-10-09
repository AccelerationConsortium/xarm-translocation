#!/usr/bin/env python3
"""Stage 0a of SERVOJ_TRAJECTORY_PLAN.md: measure timing without moving the arm.

Measures, from the PC the control box is cabled to:

1. the round trip of a read command (``get_servo_angle``), back to back;
2. the jitter of an absolute-deadline sleep loop at several rates, with
   no SDK traffic;
3. the same loop with one read command per tick, the closest no-motion
   stand-in for a servo loop;
4. the push rate of the controller's report streams (normal 30001,
   rich 30002, real-time 30003), read from raw sockets.

Read-only by construction:
- it never calls set_mode, set_state, motion_enable or any move;
- the SDK's connect would call clean_warn when a warning is active, so
  that call is replaced with a no-op before connecting;
- the report streams are only read, and nothing is sent on them.

Prints one JSON document; ``--out`` also writes it to a file.
"""
import argparse
import json
import platform
import socket
import statistics
import sys
import threading
import time

from xarm.wrapper import XArmAPI


def stats_ms(samples_s):
    """Summary statistics in milliseconds for a list of durations in seconds."""
    if not samples_s:
        return None
    ms = sorted(s * 1000.0 for s in samples_s)

    def pct(p):
        return round(ms[min(len(ms) - 1, int(p / 100.0 * len(ms)))], 3)

    return {
        'n': len(ms),
        'mean': round(statistics.fmean(ms), 3),
        'p50': pct(50),
        'p90': pct(90),
        'p99': pct(99),
        'p99_9': pct(99.9),
        'max': round(ms[-1], 3),
    }


def read_round_trips(arm, count):
    durations, failures = [], 0
    for _ in range(count):
        t0 = time.perf_counter()
        code, _ = arm.get_servo_angle()
        durations.append(time.perf_counter() - t0)
        if code != 0:
            failures += 1
    result = {'round_trip_ms': stats_ms(durations), 'nonzero_codes': failures}
    for bound_ms in (1, 2, 5, 10):
        result[f'over_{bound_ms}ms'] = sum(1 for d in durations if d * 1000 > bound_ms)
    return result


def paced_loop(rate_hz, seconds, work=None):
    """Absolute-deadline loop: tick k is due at t0 + k / rate_hz.

    Returns wake lateness against the deadline, the interval between
    ticks, and (when ``work`` is given) its duration per tick.
    """
    period = 1.0 / rate_hz
    ticks = int(rate_hz * seconds)
    lateness, intervals, work_durations = [], [], []
    late_ticks = 0
    t0 = time.perf_counter() + 0.05
    previous = None
    for k in range(ticks):
        deadline = t0 + k * period
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        woke = time.perf_counter()
        lateness.append(max(0.0, woke - deadline))
        if previous is not None:
            interval = woke - previous
            intervals.append(interval)
            if interval > 1.5 * period:
                late_ticks += 1
        previous = woke
        if work is not None:
            w0 = time.perf_counter()
            work()
            work_durations.append(time.perf_counter() - w0)
    result = {
        'rate_hz': rate_hz,
        'period_ms': round(period * 1000, 3),
        'ticks': ticks,
        'interval_ms': stats_ms(intervals),
        'lateness_ms': stats_ms(lateness),
        'intervals_over_1_5_periods': late_ticks,
    }
    if work is not None:
        result['work_ms'] = stats_ms(work_durations)
        result['work_over_period'] = sum(1 for d in work_durations if d > period)
    return result


def count_report_frames(host, port, seconds, out, key):
    """Count report frames on one push stream. Frames start with a 4-byte
    big-endian total length. Nothing is ever sent on the socket."""
    frames, arrivals = 0, []
    try:
        with socket.create_connection((host, port), timeout=3) as sock:
            sock.settimeout(1.0)
            buffer = b''
            end = time.perf_counter() + seconds
            while time.perf_counter() < end:
                try:
                    chunk = sock.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk
                while len(buffer) >= 4:
                    size = int.from_bytes(buffer[:4], 'big')
                    if size <= 0 or size > 65536:
                        out[key] = {'error': f'unexpected frame size {size}'}
                        return
                    if len(buffer) < size:
                        break
                    buffer = buffer[size:]
                    frames += 1
                    arrivals.append(time.perf_counter())
        intervals = [b - a for a, b in zip(arrivals, arrivals[1:])]
        out[key] = {
            'port': port,
            'frames': frames,
            'rate_hz': round(frames / seconds, 2),
            'interval_ms': stats_ms(intervals),
        }
    except OSError as exc:
        out[key] = {'port': port, 'error': str(exc)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('host', help='control box IP, e.g. 192.168.1.237')
    parser.add_argument('--round-trips', type=int, default=3000)
    parser.add_argument('--seconds', type=float, default=10.0, help='per paced-loop run')
    parser.add_argument('--report-seconds', type=float, default=5.0)
    parser.add_argument('--out', help='also write the JSON here')
    args = parser.parse_args()

    result = {
        'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'host': args.host,
        'python': sys.version,
        'platform': platform.platform(),
        'perf_counter': str(time.get_clock_info('perf_counter')),
        'monotonic': str(time.get_clock_info('monotonic')),
    }

    # Report streams first, before our own SDK connection adds traffic.
    reports = {}
    threads = [
        threading.Thread(target=count_report_frames,
                         args=(args.host, port, args.report_seconds, reports, name))
        for name, port in (('normal', 30001), ('rich', 30002), ('real', 30003))
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    result['report_streams'] = reports

    # Pure timing loops: the PC's scheduling alone.
    result['sleep_loop'] = [paced_loop(rate, args.seconds) for rate in (50, 100, 200, 250)]

    arm = XArmAPI(args.host, do_not_open=True, is_radian=False)
    # Read-only: never clear a controller warning from this probe.
    arm._arm.clean_warn = lambda *a, **k: 0
    arm.connect()
    try:
        if not arm.connected:
            raise SystemExit('could not connect to the control box')
        # The SDK starts with placeholder state 4 / mode 0 until the first
        # report (5 Hz on the default stream) arrives; wait for real values.
        time.sleep(1.0)
        result['controller'] = {
            'version': arm.version,
            'state': arm.state,
            'mode': arm.mode,
            'error_code': arm.error_code,
            'warn_code': arm.warn_code,
        }
        result['read_round_trip'] = read_round_trips(arm, args.round_trips)
        read = lambda: arm.get_servo_angle()  # noqa: E731
        result['read_loop'] = [paced_loop(rate, args.seconds, work=read) for rate in (100, 200)]
        result['controller_after'] = {
            'state': arm.state,
            'mode': arm.mode,
            'error_code': arm.error_code,
            'warn_code': arm.warn_code,
        }
    finally:
        arm.disconnect()

    result['finished_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    text = json.dumps(result, indent=1)
    print(text)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as handle:
            handle.write(text)


if __name__ == '__main__':
    main()
