"""Stage 0b for UR5e joint trajectories: a supervised servoJ motion probe.

THIS MOVES THE ARM. Run it only with someone at the robot, the E-stop in
reach, the pendant speed slider at 100 %, and the service's arm session
DISCONNECTED (the probe uploads its own control script; it refuses while
the service has one open).

It runs the production code path without HTTP: the service's ControlSession
(control script, RTDE watchdog, feedback sampler), the shared executor and
URServoJBackend with the commissioned motion limits as guards, and the
workspace check with controller forward kinematics. For each rate:

- full:   a small slow wiggle around the current pose, J1 +-amp and J6 +-amp
          (min-jerk, peak speed --speed), to the end;
- cancel: the same, cancelled at --stop-at (a constrained stop on the path);
- stop:   the same, STOP at --stop-at (stopJ, as POST /control/stop).

After cancel and stop the arm returns to the start pose with moveJ at 5
deg/s (after a typed YES following a STOP). Each run's record is analysed:
the lag of the arm behind the commands, the tracking error, and the
command timing inside this process.

    python tools/ur_servo_probe.py --config .state/robot-motion.local.json

Logs go to --out-dir (console and one JSON per run).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
import urllib.request
from pathlib import Path

from robot_motion.config import load_settings
from robot_motion.drivers.ur_control import ControlSession
from robot_motion.trajectory.executor import TERMINAL, TrajectoryManager
from robot_motion.trajectory.joint_trajectory import JointLimits, JointPoint, JointTrajectory, validate_joint_trajectory
from robot_motion.trajectory.routes import executor_settings
from robot_motion.trajectory.ur_backend import URServoJBackend, path_errors

try:
    from robot_motion.trajectory.ur_backend import FeedbackMonitor
except ImportError:  # a checkout older than the monitor (before it was committed)
    class FeedbackMonitor:
        def __init__(self, feedback, num_joints=6, poll_s=0.002):
            self.feedback, self.num_joints, self.poll_s = feedback, num_joints, poll_s
            self.latest, self.samples, self.dropped, self.error = None, [], 0, None
            self._stop = threading.Event()
            self._thread = None

        def start(self):
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

        def stop(self):
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=1.0)

        def _run(self):
            last = None
            while not self._stop.wait(self.poll_s):
                sample = self.feedback.latest()
                if not isinstance(sample, dict) or sample.get("controller_timestamp_s") == last:
                    continue
                last = sample.get("controller_timestamp_s")
                age = time.monotonic() - float(sample["received_monotonic_s"])
                entry = (time.perf_counter() - max(0.0, age), [float(v) for v in sample["joints_deg"]][:6],
                         sample.get("robot_mode"), sample.get("safety_mode"))
                self.latest = entry
                self.samples.append(entry)

KEY = "ur-servo-probe"


class Tee:
    def __init__(self, stream, path):
        self.stream, self.file = stream, open(path, "a", encoding="utf-8")

    def write(self, text):
        self.stream.write(text)
        self.file.write(text)
        self.file.flush()

    def flush(self):
        self.stream.flush()


def confirm(prompt):
    answer = input(prompt)
    print(f"[typed: {answer.strip()!r}]")
    return answer.strip().upper() == "YES"


def wiggle(q0, amps, speed, cycles=1, spacing_s=0.0105):
    """Rest-to-rest min-jerk segments 0 -> +1 -> -1 -> ... -> 0 (phase),
    each joint moving amps[j] * phase, with velocities and accelerations."""
    targets = [v for _ in range(cycles) for v in (1.0, -1.0)] + [0.0]
    lead = max(abs(a) for a in amps)
    knots, t, phase = [(0.0, 0.0)], 0.0, 0.0
    for target in targets:
        t += 1.875 * abs(target - phase) * lead / speed   # min-jerk peak speed = 1.875 D / T
        knots.append((t, target))
        phase = target
    points = []
    for (t0, p0), (t1, p1) in zip(knots, knots[1:]):
        h = t1 - t0
        n = max(2, int(h / spacing_s))
        for k in range(n):
            s = k / n
            b = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            db = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / h
            ddb = (60 * s - 180 * s ** 2 + 120 * s ** 3) / h ** 2
            d = p1 - p0
            points.append({
                "t": round(t0 + s * h, 6),
                "joints_deg": [q + a * (p0 + d * b) for q, a in zip(q0, amps)],
                "velocities_deg_s": [a * d * db for a in amps],
                "accelerations_deg_s2": [a * d * ddb for a in amps],
            })
    points.append({"t": round(knots[-1][0], 6), "joints_deg": list(q0),
                   "velocities_deg_s": [0.0] * 6, "accelerations_deg_s2": [0.0] * 6})
    return points


def analyse(record, moving):
    """Lag of the measured joints behind the commands, and the tracking
    error at that lag, on the moving joints (one perf_counter clock)."""
    ticks, rep = record.get("ticks") or {}, record.get("reported") or {}
    sent, cmd = ticks.get("sent_s") or [], ticks.get("commanded_deg") or []
    times, joints = rep.get("received_s") or [], rep.get("joints_deg") or []
    if len(sent) < 10 or len(times) < 10:
        return {"note": "too few samples"}

    def commanded(j, t):
        if t <= sent[0]:
            return cmd[j][0]
        if t >= sent[-1]:
            return cmd[j][-1]
        lo, hi = 0, len(sent) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if sent[mid] <= t:
                lo = mid
            else:
                hi = mid
        f = (t - sent[lo]) / (sent[hi] - sent[lo])
        return cmd[j][lo] + f * (cmd[j][hi] - cmd[j][lo])

    window = [i for i, t in enumerate(times) if sent[0] <= t <= sent[-1]]

    def errors(lag):
        return [abs(joints[i][j] - commanded(j, times[i] - lag)) for i in window for j in moving]

    best = None
    for lag_ms in range(0, 201, 2):
        e = errors(lag_ms / 1000)
        rms = math.sqrt(sum(v * v for v in e) / len(e))
        if best is None or rms < best[1]:
            best = (lag_ms, rms, max(e))
    raw = errors(0.0)
    return {
        "measured_samples": len(window),
        "lag_ms": best[0],
        "tracking_rms_deg_at_lag": round(best[1], 4),
        "tracking_max_deg_at_lag": round(best[2], 4),
        "following_error_max_deg": round(max(raw), 4),
        "note": "lag includes the feedback delivery delay; tracking is after removing the lag",
    }


def service_session_open(url):
    try:
        with urllib.request.urlopen(url, timeout=5) as resp:
            details = json.load(resp)["details"]
        return bool((details.get("control_session") or {}).get("open"))
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"Cannot read the service status at {url} ({exc}); refusing to start")


def move_home(session, q0, speed_deg_s, accel_deg_s2):
    session.control.moveJ([math.radians(v) for v in q0], math.radians(speed_deg_s),
                          math.radians(accel_deg_s2), False)


def one_run(session, motion, cfg, settings, points, rate, scenario, stop_at, lookahead, gain, out_dir):
    mgr = TrajectoryManager(settings)
    sample = session.feedback.latest()
    traj = mgr.create(key=KEY, owner="probe", num_joints=6, rate_hz=rate)
    limits = mgr.validation_limits(traj, list(zip(motion.joint_lower_deg, motion.joint_upper_deg)),
                                   motion.max_joint_speed_deg_s)
    mgr.add_chunk(traj.id, 0, KEY, points, True, limits)
    backend = URServoJBackend(
        session.control, session.feedback, period_s=1.0 / rate, num_joints=6,
        lookahead_s=lookahead, gain=gain, stop_decel_deg_s2=motion.stop_joint_decel_deg_s2,
        emergency_decel_deg_s2=motion.stop_joint_decel_deg_s2,
        max_feedback_age_s=motion.feedback_max_age_s, guard=motion,
    )
    done = threading.Event()
    mgr.start(traj.id, KEY, limits=limits, start_joints=list(sample["joints_deg"]), backend=backend,
              watch=lambda: None if session.watchdog_ok else "control_session_lost",
              on_finish=lambda _s=None: done.set(), monitor=FeedbackMonitor(session.feedback, 6))
    acted = None
    while traj.state not in TERMINAL:
        if scenario != "full" and acted is None and traj.duration_s and traj.t_exec >= stop_at * traj.duration_s:
            acted = round(traj.t_exec, 3)
            if scenario == "cancel":
                mgr.cancel(traj.id, KEY)
            else:
                mgr.notify_hard_stop("stop")
                session.stop(motion.stop_joint_decel_deg_s2)
        time.sleep(0.005)
    done.wait(10)
    mgr.wait_idle(10)
    status = traj.status()
    result = {
        "rate_hz": rate, "scenario": scenario, "lookahead_s": lookahead, "gain": gain,
        "state": status["state"], "reason": status["reason"], "acted_at_t": acted,
        "duration_s": status["executed"]["duration_s"], "t_exec": status["executed"]["t_exec"],
        "servo": status["servo"], "final": status["final"],
        "analysis": analyse(traj.record or {}, moving=(0, 5)) if traj.record else None,
        "peak_force_change_n": round(backend.peak_force_change_n, 2),
    }
    name = f"{time.strftime('%Y%m%dT%H%M%S')}-{int(rate)}hz-{scenario}.json"
    with open(os.path.join(out_dir, name), "w", encoding="utf-8") as handle:
        json.dump({"result": result, "record": traj.record}, handle)
    print(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True)
    parser.add_argument("--rates", default="125,250")
    parser.add_argument("--scenarios", default="full,cancel,stop")
    parser.add_argument("--amp-j1", type=float, default=5.0)
    parser.add_argument("--amp-j6", type=float, default=10.0)
    parser.add_argument("--speed", type=float, default=8.0, help="peak speed of the leading joint, deg/s")
    parser.add_argument("--stop-at", type=float, default=0.25)
    parser.add_argument("--lookahead", type=float, default=None)
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--status-url", default="http://100.64.254.18:8075/status")
    parser.add_argument("--out-dir", default=r"C:\Users\sdl2\ur_probe\logs")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    console = os.path.join(args.out_dir, time.strftime("console-%Y%m%dT%H%M%S.txt"))
    sys.stdout = Tee(sys.stdout, console)
    sys.stderr = Tee(sys.stderr, console)
    settings_all = load_settings(Path(args.config))
    motion, cfg = settings_all.control.motion, settings_all.control.trajectory
    if motion is None or cfg is None:
        raise SystemExit("The config needs control.motion and control.trajectory")
    lookahead = args.lookahead if args.lookahead is not None else cfg.lookahead_s
    gain = args.gain if args.gain is not None else cfg.gain
    rates = [float(r) for r in args.rates.split(",")]
    if any(r > 250 for r in rates):
        raise SystemExit("Rates above 250 Hz are not part of stage 0b")
    if args.speed > motion.max_joint_speed_deg_s:
        raise SystemExit(f"--speed {args.speed} is above the commissioned {motion.max_joint_speed_deg_s} deg/s")
    settings = executor_settings(cfg.model_copy(update={"enabled": True, "max_rate_hz": max(max(rates), cfg.rate_hz),
                                                       "log_dir": args.out_dir}), motion)

    if service_session_open(args.status_url):
        raise SystemExit("The service has the arm session open. Press Disconnect in the panel first.")
    print(f"stage 0b probe  {time.strftime('%Y-%m-%d %H:%M:%S')}  robot {settings_all.robot_host}")
    print(f"limits {motion.commissioning_id}: {motion.max_joint_speed_deg_s} deg/s, "
          f"{motion.joint_accel_deg_s2} deg/s^2; servoJ lookahead {lookahead} s, gain {gain}")

    session = ControlSession(settings_all)
    session.open()
    try:
        time.sleep(0.5)
        sample = session.feedback.latest()
        if sample is None:
            raise SystemExit("No control feedback")
        q0 = [round(v, 4) for v in sample["joints_deg"]]
        print(f"start pose (deg): {q0}")
        print(f"robot {sample['robot_mode']} / safety {sample['safety_mode']}, "
              f"speed slider {sample.get('speed_fraction')}")
        amps = [args.amp_j1, 0, 0, 0, 0, args.amp_j6]
        points = wiggle(q0, amps, args.speed)
        for rate in rates:
            limits = JointLimits(
                joint_limits_deg=list(zip(motion.joint_lower_deg, motion.joint_upper_deg)),
                max_joint_speed_deg_s=float(motion.max_joint_speed_deg_s),
                max_joint_acc_deg_s2=settings.max_joint_acc_deg_s2, servo_rate_hz=rate,
                max_duration_s=settings.max_duration_s, max_points=settings.max_points)
            parsed = [JointPoint.from_mapping(p) for p in points]
            report = validate_joint_trajectory(parsed, limits, start_joints=q0)
            if not report["valid"]:
                raise SystemExit(f"Trajectory invalid at {rate:g} Hz: {report['errors'][:3]}")
            print(f"{rate:g} Hz: trajectory valid; checking the path with controller kinematics")
            errors = path_errors(JointTrajectory(parsed), session.control, motion,
                                 step_deg=cfg.path_check_step_deg, max_checks=cfg.max_fk_checks, rate_hz=rate)
            if errors:
                raise SystemExit(f"Workspace check failed: {errors}")
        s = report["summary"]
        print(f"wiggle: J1 +-{args.amp_j1} deg, J6 +-{args.amp_j6} deg, {s['duration_s']:.1f} s, "
              f"peak {s['max_joint_speed_deg_s']:.1f} deg/s, {s['max_joint_acc_deg_s2']:.1f} deg/s^2; "
              f"workspace box OK")
        runs = [(r, sc) for r in rates for sc in args.scenarios.split(",")]
        print("runs: " + ", ".join(f"{r:g} Hz {sc}" for r, sc in runs))
        if not confirm("The arm will move. Type YES to start: "):
            print("Not confirmed; nothing moved.")
            return
        results = []
        for rate, scenario in runs:
            print(f"\n=== {rate:g} Hz  {scenario} ===")
            results.append(one_run(session, motion, cfg, settings, points, rate, scenario,
                                   args.stop_at, lookahead, gain, args.out_dir))
            if scenario == "stop":
                if not confirm("STOP done. Check the arm. Type YES to drive back to the start pose: "):
                    print("Not confirmed; ending here.")
                    break
            if scenario in ("cancel", "stop"):
                print("returning to the start pose at 5 deg/s")
                move_home(session, q0, 5.0, motion.joint_accel_deg_s2)
            time.sleep(0.5)
        print("\n=== summary ===")
        for r in results:
            a = r.get("analysis") or {}
            servo = r.get("servo") or {}
            print(f"{r['rate_hz']:g} Hz {r['scenario']:6s} {r['state']:9s} lag {a.get('lag_ms')} ms  "
                  f"tracking rms {a.get('tracking_rms_deg_at_lag')} max {a.get('tracking_max_deg_at_lag')} deg  "
                  f"late ticks {servo.get('late_ticks')}  lateness {servo.get('lateness_ms')}")
    finally:
        session.close()
        print("control session closed; Connect again in the panel")


if __name__ == "__main__":
    try:
        main()
    except SystemExit as exc:
        if exc.code not in (None, 0):
            print(f"REFUSED: {exc.code}")
        raise
    except BaseException:
        import traceback

        print("ERROR:\n" + traceback.format_exc())
        raise
