#!/usr/bin/env python3
"""Stage 0b of SERVOJ_TRAJECTORY_PLAN.md: compare ServoJ and mode 6 on the arm.

THIS MOVES THE ARM. Run it only with someone at the robot, the E-stop in
reach, and the arm parked at a clear graph node away from the hood and the
instruments. The ``xarm`` service must not be connected to the arm (stop
it, or Disconnect in the panel), so that this is the only program
commanding it.

It drives the same executor the service uses (``core.trajectory_executor``)
through the real xArm backends, with nothing in between: no claim, no
graph or sash interlock. Each run is a small, slow, rest-to-rest wiggle
around the arm's *current* pose: J1 and J5 go out, back through it to the
other side, and return, a set number of times.

Without ``--execute`` it only connects, reads the current pose, builds and
validates every run, and prints the plan. Nothing moves.

Scenarios, in order:
- ``mode1``: ServoJ at each ``--mode1-rates``, the whole trajectory.
- ``mode6``: online planning at each ``--mode6-rates``, the whole trajectory.
- ``cancel``: one run per mode, cancelled at ``--stop-at`` (25 %, mid-swing at
  peak speed); the constrained stop.
- ``stop``: one run per mode, STOP (emergency stop) at ``--stop-at``, then the probe
  recovers the arm like Clear errors does (after you confirm).

For each run it writes the executor's full record (every tick and every
real-time report frame) to ``--out`` and prints a summary:
- send timing: period, lateness, round trip and late ticks;
- lag: how far the arm runs behind the commands;
- tracking: the error that remains after removing that lag;
- smoothness: over 50 ms windows;
- holds: single 10 ms frames where a moving joint did not move.

Run from a checkout of the branch with this tool, using the service's
Python, for example:

    .venv\\Scripts\\python.exe -I tools\\servo_mode_probe.py 192.168.1.237
    .venv\\Scripts\\python.exe -I tools\\servo_mode_probe.py 192.168.1.237 --execute
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from xarm.wrapper import XArmAPI  # noqa: E402

from core.joint_trajectory import JointTrajectory, JointPoint  # noqa: E402
from core.trajectory_executor import (  # noqa: E402
    RealtimeJointMonitor, TrajectoryError, TrajectoryManager, TrajectorySettings, make_backend,
)

# xArm5 limits from src/settings/safety.yaml (model 5).
XARM5_LIMITS = [(-360, 360), (-118, 120), (-225, 11), (-97, 180), (-360, 360)]
REPORT_PERIOD_S = 0.01   # the real-time report is 100 Hz


def wiggle_points(q0, amp_j1, amp_j5, speed, cycles, spacing_s=0.0105):
    """Rest-to-rest min-jerk segments 0 -> +A -> -A -> ... -> 0 on J1 and J5,
    sampled with positions, velocities and accelerations."""
    targets = []
    for _ in range(cycles):
        targets += [1.0, -1.0]
    targets.append(0.0)
    knots = [(0.0, 0.0)]   # (time, phase)
    t = 0.0
    phase = 0.0
    for target in targets:
        distance = max(abs(target - phase) * amp_j1, abs(target - phase) * amp_j5)
        duration = 1.875 * distance / speed          # min-jerk peak speed = 1.875 D / T
        t += duration
        knots.append((t, target))
        phase = target
    points = []
    # Points at least spacing_s apart: the validator rejects points closer
    # than one command period (10 ms at 100 Hz).
    for (t0, p0), (t1, p1) in zip(knots, knots[1:]):
        h = t1 - t0
        n = max(2, int(h / spacing_s))
        for k in range(n):
            s = k / n
            blend = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            dblend = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / h
            ddblend = (60 * s - 180 * s ** 2 + 120 * s ** 3) / (h * h)
            phase = p0 + (p1 - p0) * blend
            vel = (p1 - p0) * dblend
            acc = (p1 - p0) * ddblend
            q = list(q0)
            qd = [0.0] * len(q0)
            qdd = [0.0] * len(q0)
            q[0] += amp_j1 * phase
            q[4] += amp_j5 * phase
            qd[0], qd[4] = amp_j1 * vel, amp_j5 * vel
            qdd[0], qdd[4] = amp_j1 * acc, amp_j5 * acc
            points.append({"t": round(t0 + h * s, 9), "joints_deg": q,
                           "velocities_deg_s": qd, "accelerations_deg_s2": qdd})
    final = list(q0)
    points.append({"t": round(knots[-1][0], 9), "joints_deg": final,
                   "velocities_deg_s": [0.0] * len(q0), "accelerations_deg_s2": [0.0] * len(q0)})
    return points


def _interp(ts, vs, t):
    i = bisect.bisect_right(ts, t) - 1
    if i < 0:
        return vs[0]
    if i >= len(ts) - 1:
        return vs[-1]
    return vs[i] + (vs[i + 1] - vs[i]) * (t - ts[i]) / (ts[i + 1] - ts[i])


def analyse(points, record, num_joints, moving_joints=(0, 4)):
    """Compare what the arm reported with what was commanded, on one clock.

    Report frames are sampled every ~10 ms by the controller but arrive in
    ~50 ms bursts, so their sample times are reconstructed: a straight line
    through the last (least delayed) frame of each burst. The commanded
    series is then shifted by the lag that best matches the reported one:
    - lag_ms: how far the arm runs behind the commands, plus the delivery
      delay of the freshest report frame, which cannot be separated out. It
      is therefore an upper bound on the arm's own lag;
    - tracking: the remaining error;
    - smoothness: over 50 ms windows (10 ms differences are dominated by
      report timing noise);
    - holds: 10 ms frames where a moving joint did not move.
    """
    ticks = (record or {}).get("ticks") or {}
    reported = (record or {}).get("reported") or {}
    sent = ticks.get("sent_s") or []
    commanded = ticks.get("commanded_deg") or []
    received = reported.get("received_s") or []
    frames = reported.get("joints_deg") or []
    if len(frames) < 20 or len(sent) < 3:
        return {"frames": len(frames), "note": "too few real-time frames or commands to analyse"}
    last_in_burst = [i for i in range(len(received))
                     if i == len(received) - 1 or received[i + 1] - received[i] > 0.005]
    n = len(last_in_burst)
    mx = sum(last_in_burst) / n
    my = sum(received[i] for i in last_in_burst) / n
    denom = sum((x - mx) ** 2 for x in last_in_burst) or 1.0
    period = sum((x - mx) * (received[x] - my) for x in last_in_burst) / denom
    times = [my + period * (i - mx) for i in range(len(frames))]

    def errors(lag):
        out = []
        for i, t in enumerate(times):
            tc = t - lag
            if sent[0] <= tc <= sent[-1]:
                out.append(max(abs(_interp(sent, commanded[j], tc) - frames[i][j]) for j in moving_joints))
        return out

    best = None
    for lag_ms in range(0, 801, 5):
        e = errors(lag_ms / 1000.0)
        if e:
            rms = (sum(x * x for x in e) / len(e)) ** 0.5
            if best is None or rms < best[0]:
                best = (rms, lag_ms, max(e))
    w = 5
    speeds = [max(abs(frames[i + w][j] - frames[i][j]) for j in moving_joints) / (times[i + w] - times[i])
              for i in range(len(frames) - w)]
    accs = sorted(abs(speeds[i + w] - speeds[i]) / (times[i + w] - times[i]) for i in range(len(speeds) - w))
    holds = sum(1 for i in range(1, len(frames) - 1)
                if max(abs(frames[i + 1][j] - frames[i][j]) for j in moving_joints) < 1e-4
                and max(abs(frames[i][j] - frames[i - 1][j]) for j in moving_joints) > 0.02)
    return {
        "frames": len(frames),
        "frame_period_ms": round(period * 1000, 3),
        "lag_ms": best[1] if best else None,
        "tracking_deg": None if not best else {"rms": round(best[0], 4), "max": round(best[2], 4)},
        "smoothness_50ms": {"peak_speed_deg_s": round(max(speeds), 2),
                            "acc_p99_deg_s2": round(accs[int(0.99 * (len(accs) - 1))], 1) if accs else None,
                            "acc_max_deg_s2": round(accs[-1], 1) if accs else None},
        "single_frame_holds": holds,
    }


def service_connected():
    try:
        with urllib.request.urlopen("http://127.0.0.1:8000/status", timeout=2) as resp:
            status = json.load(resp)
        return status.get("equipment_status") not in (None, "requires_init")
    except Exception:  # noqa: BLE001 - not running is fine
        return False


def recover(arm):
    """What Clear errors does: back to mode 0, state 0."""
    for name, call in (("clean_error", arm.clean_error), ("clean_warn", arm.clean_warn),
                       ("motion_enable", lambda: arm.motion_enable(enable=True)),
                       ("set_mode(0)", lambda: arm.set_mode(0)), ("set_state(0)", lambda: arm.set_state(0))):
        code = call()
        print(f"    {name}: {code}")
        time.sleep(0.2)


def one_run(arm, host, args, backend_name, rate, points, scenario, out_dir, run_id):
    settings = TrajectorySettings.from_mapping({
        "enabled": True, "backend": backend_name,
        "servo_rate_hz": rate if backend_name == "servoj" else 100.0,
        "online_rate_hz": rate if backend_name == "online_planning" else 50.0,
        "max_joint_acc_deg_s2": args.max_acc, "log_dir": str(out_dir),
    })
    mgr = TrajectoryManager(settings)
    generation = mgr.stop_generation()
    session = mgr.create(key=None, owner="servo_mode_probe", num_joints=5, rate_hz=rate)
    limits = mgr.validation_limits(session, XARM5_LIMITS, args.max_speed)
    mgr.add_chunk(session.id, 0, None, points, final=True, limits=limits)
    code, angles = arm.get_servo_angle(is_radian=False)
    if code != 0:
        raise SystemExit(f"could not read joints (code {code})")
    backend = make_backend(backend_name, arm, settings.command_timeout_s)
    monitor = RealtimeJointMonitor(host, 5)
    mgr.start(session.id, None, limits=limits, start_joints=list(angles)[:5], backend=backend,
              watch=lambda: None, on_finish=lambda s: None, monitor=monitor,
              stop_generation=generation)
    if scenario in ("cancel", "stop"):
        while session.state in ("starting", "running") and \
                session.t_exec < args.stop_at * (session.duration_s or 0):
            time.sleep(0.01)
        if scenario == "cancel":
            mgr.cancel(session.id, None)
        else:
            mgr.notify_hard_stop("stop")
            arm.emergency_stop()
    # Wait for the executor thread itself, so the terminal state, the
    # record and the log are all in place before reading them.
    if not mgr.wait_idle(timeout_s=(session.duration_s or 0) + 30):
        arm.emergency_stop()
        raise SystemExit("the run did not end in time; emergency stop sent")
    status = session.status()
    result = {
        "run": run_id, "backend": backend_name, "rate_hz": rate, "scenario": scenario,
        "state": status["state"], "reason": status["reason"],
        "servo": status["servo"], "final": status["final"],
        "analysis": analyse(points, session.record, 5),
        "stop_at": args.stop_at if scenario in ("cancel", "stop") else None,
        "log": session.log_path,
    }
    print(json.dumps({k: result[k] for k in ("run", "backend", "rate_hz", "scenario", "state", "reason")}))
    servo = status["servo"] or {}
    print(f"    period {servo.get('period_ms')}\n    lateness {servo.get('lateness_ms')}\n"
          f"    round trip {servo.get('round_trip_ms')}\n    late ticks {servo.get('late_ticks')}, "
          f"delay {servo.get('cumulative_delay_ms')} ms, sdk errors {servo.get('sdk_errors')}")
    print(f"    analysis {json.dumps(result['analysis'])}")
    return result


class _Tee:
    """Copy everything printed (and typed answers) to a console log, so the
    run can be reviewed from elsewhere."""

    def __init__(self, stream, path):
        self.stream = stream
        self.file = open(path, "a", encoding="utf-8")

    def write(self, text):
        self.stream.write(text)
        self.file.write(text)
        self.file.flush()
        return len(text)

    def flush(self):
        self.stream.flush()
        self.file.flush()


def confirm(prompt):
    answer = input(prompt)
    print(f"[typed: {answer.strip()!r}]")
    return answer.strip().upper() == "YES"


def main():
    os.makedirs(os.path.join("logs", "servo_mode_probe"), exist_ok=True)
    sys.stdout = _Tee(sys.stdout, os.path.join(
        "logs", "servo_mode_probe", time.strftime("console-%Y%m%dT%H%M%SZ.txt", time.gmtime())))
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("host", help="control box IP, e.g. 192.168.1.237")
    parser.add_argument("--execute", action="store_true", help="actually move the arm")
    parser.add_argument("--amp-j1", type=float, default=5.0, help="J1 amplitude, deg (max 10)")
    parser.add_argument("--amp-j5", type=float, default=10.0, help="J5 amplitude, deg (max 20)")
    parser.add_argument("--speed", type=float, default=20.0, help="peak joint speed, deg/s (max 30)")
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--mode1-rates", default="100,200")
    parser.add_argument("--mode6-rates", default="20,50")
    parser.add_argument("--scenarios", default="mode1,mode6,cancel,stop")
    parser.add_argument("--max-speed", type=float, default=60.0, help="validation speed limit, deg/s")
    parser.add_argument("--max-acc", type=float, default=500.0, help="acceleration limit, deg/s^2")
    parser.add_argument("--stop-at", type=float, default=0.25,
                        help="fraction of the trajectory at which cancel/stop runs act; 0.25 is "
                             "mid-swing at peak speed for the default wiggle")
    parser.add_argument("--out", default=os.path.join("logs", "servo_mode_probe"))
    args = parser.parse_args()
    if not (0 < args.amp_j1 <= 10 and 0 < args.amp_j5 <= 20 and 0 < args.speed <= 30):
        raise SystemExit("amplitudes and speed are capped for this test: J1 <= 10, J5 <= 20 deg, 30 deg/s")
    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    mode1 = [float(r) for r in args.mode1_rates.split(",") if r.strip()]
    mode6 = [float(r) for r in args.mode6_rates.split(",") if r.strip()]

    if service_connected():
        raise SystemExit("The xarm service is connected to the arm. Disconnect it in the panel or stop "
                         "the service first, so only this probe commands the arm.")

    arm = XArmAPI(args.host, is_radian=False)
    try:
        time.sleep(1.0)    # first report
        if arm.error_code or arm.warn_code or arm.state in (4, 5):
            raise SystemExit(f"arm not ready: state {arm.state}, error {arm.error_code}, warn {arm.warn_code}")
        code, angles = arm.get_servo_angle(is_radian=False)
        if code != 0:
            raise SystemExit(f"could not read joints (code {code})")
        q0 = [float(a) for a in list(angles)[:5]]
        points = wiggle_points(q0, args.amp_j1, args.amp_j5, args.speed, args.cycles)
        for j, (lo, hi) in enumerate(XARM5_LIMITS):
            values = [p["joints_deg"][j] for p in points]
            if min(values) < lo or max(values) > hi:
                raise SystemExit(f"J{j + 1} would leave [{lo}, {hi}] from the current pose")
        runs = []
        if "mode1" in scenarios:
            runs += [("servoj", r, "full") for r in mode1]
        if "mode6" in scenarios:
            runs += [("online_planning", r, "full") for r in mode6]
        for scenario in ("cancel", "stop"):
            if scenario in scenarios:
                if mode1:
                    runs.append(("servoj", mode1[0], scenario))
                if mode6:
                    runs.append(("online_planning", mode6[-1], scenario))
        duration = points[-1]["t"]
        print(f"Controller {arm.version}; current joints {[round(v, 3) for v in q0]}")
        print(f"Trajectory: J1 +/-{args.amp_j1} deg, J5 +/-{args.amp_j5} deg, peak {args.speed} deg/s, "
              f"{args.cycles} cycles, {duration:.1f} s, {len(points)} points; returns to the start pose.")
        for i, (name, rate, scenario) in enumerate(runs, 1):
            print(f"  run {i}: {name} at {rate:g} Hz, {scenario}")

        # Validate every run up front, without moving.
        for name, rate, _ in runs:
            settings = TrajectorySettings.from_mapping({
                "enabled": True, "backend": name,
                "servo_rate_hz": rate if name == "servoj" else 100.0,
                "online_rate_hz": rate if name == "online_planning" else 50.0,
                "max_joint_acc_deg_s2": args.max_acc})
            mgr = TrajectoryManager(settings)
            session = mgr.create(key=None, owner="probe", num_joints=5, rate_hz=rate)
            try:
                mgr.add_chunk(session.id, 0, None, points, final=True,
                              limits=mgr.validation_limits(session, XARM5_LIMITS, args.max_speed))
            except TrajectoryError as exc:
                raise SystemExit(f"{name} at {rate:g} Hz does not validate: {json.dumps(exc.detail)[:2000]}")
        print("All runs validate.")
        if not args.execute:
            print("Dry run: nothing moved. Add --execute to run it with someone at the robot.")
            return

        if not confirm("The arm will move. Someone is at the robot with the E-stop in reach? Type YES: "):
            print("Not confirmed; nothing moved.")
            return
        out_dir = Path(args.out) / time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        out_dir.mkdir(parents=True, exist_ok=True)
        results = []
        for i, (name, rate, scenario) in enumerate(runs, 1):
            print(f"\nRun {i}/{len(runs)}: {name} at {rate:g} Hz, {scenario}")
            result = one_run(arm, args.host, args, name, rate, points, scenario, out_dir, i)
            results.append(result)
            planned_stop = scenario == "stop" and result["state"] == "stopped"
            if result["state"] not in ("completed", "cancelled") and not planned_stop:
                # A real fault (collision, controller error, timing): do not
                # clear it automatically. Stop here and look at it.
                print(f"  Unplanned outcome ({result['state']}: {result['reason']}). Stopping the probe; "
                      "inspect the arm and clear errors yourself.")
                break
            if planned_stop:
                print("  The arm is stopped (planned STOP). Recover it like Clear errors?")
                if not confirm("  Type YES to recover and continue: "):
                    break
                recover(arm)
                time.sleep(1.0)
            # Every run starts where the previous one ended; go back to the
            # start pose with an ordinary joint move if it is not there.
            code, angles = arm.get_servo_angle(is_radian=False)
            here = [float(a) for a in list(angles)[:5]]
            if code == 0 and max(abs(a - b) for a, b in zip(here, q0)) > 0.3:
                print("  Returning to the start pose (mode 0 joint move, 10 deg/s).")
                code = arm.set_servo_angle(angle=q0, speed=10, mvacc=200, wait=True, is_radian=False)
                if code != 0:
                    print(f"  The return move failed (code {code}). Stopping the probe.")
                    break
            elif code != 0:
                print(f"  Could not read the joints (code {code}). Stopping the probe.")
                break
            time.sleep(1.0)
        summary_path = out_dir / "summary.json"
        summary_path.write_text(json.dumps(results, indent=1), encoding="utf-8")
        print(f"\nSummary written to {summary_path}")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    try:
        main()
    except SystemExit as exc:
        if exc.code not in (None, 0):
            print(f"Stopped: {exc.code}")
        raise
    except BaseException:
        import traceback
        print(traceback.format_exc())
        raise
