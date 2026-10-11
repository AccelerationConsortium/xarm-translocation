#!/usr/bin/env python3
"""Stage 3 of UR5E_TRAJECTORY_PLAN.md: the joint-trajectory HTTP API on the UR5e.

THIS MOVES THE ARM. Run it only with someone at the robot, the E-stop in
reach, the pendant speed slider at 100 %, and the arm CONNECTED in the
panel (Connect, nobody holding control). It talks to the robot exactly as
a remote client (Astra's digital twin) would, through the lab edge's
/ur5e/ route (the panel's own path): Caddy checks the sign-in cookie and
adds the operator identity and the device's edge secret; the client's
X-Claim-Token passes through untouched:

1. sign in with an emailed one-time code (kept in memory only; signed out
   at the end), take the claim, and keep it alive with heartbeats; every
   call carries the claim's X-Claim-Token;
2. read GET /limits and validate the stage 0b wiggle around the current
   pose (J1 +-5, J6 +-10 deg, peak 8 deg/s);
3. after a typed YES, three runs: full; cancel at --stop-at; STOP
   (POST /control/stop) at --stop-at, then after another YES: Clear errors
   and drive back with a joint move at 5 deg/s;
4. fetch each session's log and analyse it like the probe; release the
   claim and sign out.

    python tools/ur_trajectory_http_check.py --email you@utoronto.ca

Everything printed is also written to --out-dir.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ur_servo_probe import Tee, analyse, confirm, wiggle  # noqa: E402

EDGE = "http://100.64.254.6"
ROUTE = "/ur5e"   # the edge route to the UR service (panel and API)
DEVICE_STATUS = "http://100.64.254.18:8075/status"   # open, read-only
TRAJ = "/freehand/trajectory"


class Client:
    def __init__(self, base, route):
        self.base = base
        self.prefix = f"{route}/control"
        self.cookie = None
        self.token = None
        self.open_session = None
        self.last_headers = {}

    def raw(self, method, url, body=None, timeout=30):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(url, data=data, method=method)
        req.add_header("Content-Type", "application/json")
        if self.cookie:
            req.add_header("Cookie", self.cookie)
        if self.token:
            req.add_header("X-Claim-Token", self.token)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw, status = resp.read(), resp.status
                set_cookie = resp.headers.get("Set-Cookie")
                self.last_headers = dict(resp.headers)
        except urllib.error.HTTPError as exc:
            raw, status, set_cookie = exc.read(), exc.code, None
            self.last_headers = dict(exc.headers or {})
        try:
            payload = json.loads(raw) if raw else None
        except ValueError:
            payload = raw.decode(errors="replace")
        return status, payload, set_cookie

    def call(self, method, path, body=None, timeout=30):
        """A device control route, through the edge."""
        return self.raw(method, self.base + self.prefix + path, body, timeout)

    def edge(self, method, path, body=None, timeout=30):
        return self.raw(method, self.base + path, body, timeout)


def expect(status, payload, wanted, what):
    if status != wanted:
        raise SystemExit(f"{what}: expected HTTP {wanted}, got {status}: {json.dumps(payload)[:600]}")
    return payload


def device_status():
    with urllib.request.urlopen(DEVICE_STATUS, timeout=5) as resp:
        return json.load(resp)


def wait_terminal(client, sid, act=None, act_at=None, checks=None, timeout=60):
    deadline = time.time() + timeout
    acted = checked = False
    body = None
    while time.time() < deadline:
        _, body, _ = client.call("GET", f"{TRAJ}/{sid}")
        state = (body or {}).get("state")
        if state in ("running", "stopping"):
            if checks and not checked and body["executed"]["t_exec"] > 0.5:
                checked = True
                checks(body)
            if act and not acted and act_at is not None and body["executed"]["t_exec"] >= act_at:
                acted = True
                act()
        if state in ("completed", "cancelled", "stopped", "failed", "expired") and body.get("log_available"):
            return body
        time.sleep(0.1)
    raise SystemExit(f"session {sid} did not end in {timeout} s; last status {json.dumps(body)[:400]}")


def cancel_open(client):
    if client.open_session and client.token:
        status, _, _ = client.call("POST", f"{TRAJ}/{client.open_session}/cancel", {})
        print(f"  Cancelled the unfinished session {client.open_session}: HTTP {status}")
    client.open_session = None


def go_home(client, q0):
    # Right after a run the arm can still be settling (a few thousandths of
    # a degree per second), which the move check refuses as robot_moving.
    for _ in range(15):
        status, payload, _ = client.call("POST", "/freehand/joints", {"angles": q0, "speed": 5}, timeout=90)
        detail = payload.get("detail") if isinstance(payload, dict) else None
        if status == 412 and isinstance(detail, dict) and detail.get("error") == "robot_moving":
            time.sleep(0.3)
            continue
        break
    print(f"  Return to the start pose (moveJ, 5 deg/s): HTTP {status}")
    if status != 200:
        raise SystemExit(f"return failed: {json.dumps(payload)[:400]}")
    time.sleep(0.5)


def one_run(client, points, scenario, rate, stop_at, out_dir, results):
    print(f"\nRun: servoJ at {rate:g} Hz via the edge, {scenario}")
    session = expect(*client.call("POST", TRAJ, {"rate_hz": rate})[:2], 201, "create")
    sid = session["session_id"]
    client.open_session = sid
    expect(*client.call("POST", f"{TRAJ}/{sid}/chunks/0", {"points": points, "final": True}, timeout=60)[:2],
           200, "upload")
    observed = {}

    def checks(body):
        observed["measured_during_run"] = body.get("measured")
        st = device_status()
        det = st.get("details") or {}
        observed["status_trajectory_state"] = ((det.get("trajectory") or {}).get("session") or {}).get("state")
        observed["status_busy"] = (det.get("control_session") or {}).get("busy")
        observed["move_allowed_during_run"] = "arm.move_joints" in (st.get("allowed_actions") or [])

    act = None
    if scenario == "cancel":
        def act():
            observed["cancel_http"] = client.call("POST", f"{TRAJ}/{sid}/cancel", {})[0]
    elif scenario == "stop":
        def act():
            observed["stop_http"] = client.call("POST", "/stop", {})[0]
    started = expect(*client.call("POST", f"{TRAJ}/{sid}/start", {}, timeout=60)[:2], 200, "start")
    duration = started["executed"]["duration_s"]
    end = wait_terminal(client, sid, act=act, act_at=stop_at * duration if act else None, checks=checks,
                        timeout=duration + 30)
    client.open_session = None
    _, record, _ = client.call("GET", f"{TRAJ}/{sid}/log", timeout=90)
    (out_dir / f"{sid}.json").write_text(json.dumps(record), encoding="utf-8")
    result = {
        "scenario": scenario, "rate_hz": rate, "session_id": sid,
        "state": end["state"], "reason": end["reason"], "started_at_utc": end["started_at_utc"],
        "t_exec": end["executed"]["t_exec"], "servo": end["servo"], "tracking": end.get("tracking"),
        "final": end["final"], "observed": observed,
        "analysis": analyse(record, moving=(0, 5)) if isinstance(record, dict) else None,
    }
    results.append(result)
    print(json.dumps({k: result[k] for k in ("scenario", "state", "reason", "t_exec", "observed")}))
    servo = end["servo"] or {}
    print(f"    lateness {servo.get('lateness_ms')}\n    late ticks {servo.get('late_ticks')}, "
          f"sdk errors {servo.get('sdk_errors')}")
    print(f"    final {json.dumps(end['final'])}")
    print(f"    analysis {json.dumps(result['analysis'])}")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--email", required=True, help="your sign-in email (a one-time code is sent)")
    parser.add_argument("--rate", type=float, default=125.0)
    parser.add_argument("--stop-at", type=float, default=0.5)
    parser.add_argument("--scenarios", default="full,cancel,stop")
    parser.add_argument("--edge", default=EDGE)
    parser.add_argument("--out-dir", default=r"C:\Users\sdl2\ur_probe\http_check")
    args = parser.parse_args()
    stamp = time.strftime("%Y%m%dT%H%M%S")
    out_dir = Path(args.out_dir) / stamp
    out_dir.mkdir(parents=True, exist_ok=True)
    sys.stdout = Tee(sys.stdout, str(out_dir / "console.txt"))
    sys.stderr = Tee(sys.stderr, str(out_dir / "console.txt"))

    st = device_status()
    det = st.get("details") or {}
    session = det.get("control_session") or {}
    print(f"Robot: {st.get('equipment_status')}, {det.get('robotmode')}/{det.get('safetystatus')}, "
          f"session open {session.get('open')}, claimed by {det.get('claimed_by')}, "
          f"kinematics {session.get('kinematics_error') or 'verified'}")
    if not session.get("open"):
        raise SystemExit("The arm is not connected: press Connect in the panel first.")
    if det.get("claimed_by"):
        raise SystemExit("Someone holds control; release it in the panel first.")

    client = Client(args.edge, ROUTE)
    print(f"Sending a one-time sign-in code to {args.email} ...")
    status, payload, _ = client.edge("POST", "/auth/login", {"email": args.email})
    if status == 429:
        retry = client.last_headers.get("Retry-After")
        raise SystemExit(f"the sign-in service is rate-limiting codes for this address (retry after {retry} s)")
    if status not in (200, 202):
        raise SystemExit(f"request code: HTTP {status}: {json.dumps(payload)[:400]}")
    code = input("Type the code from the email: ").strip()
    status, payload, set_cookie = client.edge("POST", "/auth/verify-code", {"email": args.email, "code": code})
    expect(status, payload, 200, "verify code")
    if not set_cookie:
        raise SystemExit("signed in, but no session cookie came back")
    client.cookie = set_cookie.split(";", 1)[0]
    print("Signed in.")

    heartbeat_stop = threading.Event()
    results = []
    try:
        claim = expect(*client.call("POST", "/claim", {
            "owner": args.email, "session_id": f"ur-trajectory-check-{uuid.uuid4().hex[:8]}", "ttl_s": 60,
        })[:2], 200, "claim")
        client.token = claim["claim_token"]
        print("Claim taken (through the edge).")

        def beat():
            while not heartbeat_stop.wait(10):
                status, _, _ = client.call("POST", "/heartbeat", {})
                if status not in (200, 204):
                    print(f"  heartbeat: HTTP {status}")

        threading.Thread(target=beat, daemon=True).start()

        limits = expect(*client.call("GET", f"{TRAJ}/limits")[:2], 200, "limits")
        print(f"Limits: {limits['max_joint_speed_deg_s']} deg/s, {limits['max_joint_acc_deg_s2']} deg/s^2, "
              f"rate {limits['rate_hz']}, workspace {limits['workspace_mm']}, {limits['commissioning_id']}")
        q0 = [round(float(v), 4) for v in device_status()["details"]["current_joints"]]
        print(f"Current joints {q0}")
        points = wiggle(q0, [5.0, 0, 0, 0, 0, 10.0], 8.0)
        report = expect(*client.call("POST", f"{TRAJ}/validate", {"points": points, "rate_hz": args.rate},
                                     timeout=60)[:2], 200, "validate")
        s = report["summary"]
        print(f"Validated (incl. the workspace check): {s['points']} points, {s['duration_s']} s, "
              f"{s['interpolation']}, start error {report['start_state'].get('joint_error_deg')} deg.")
        scenarios = [x.strip() for x in args.scenarios.split(",") if x.strip()]
        print(f"Runs: {', '.join(scenarios)} (cancel and STOP at {args.stop_at:.0%}); J1 +-5, J6 +-10 deg.")
        if not confirm("The arm will move. Someone is at the robot with the E-stop in reach? Type YES: "):
            print("Not confirmed; nothing moved.")
            return
        wanted = {"full": "completed", "cancel": "cancelled", "stop": "stopped"}
        for scenario in scenarios:
            result = one_run(client, points, scenario, args.rate, args.stop_at, out_dir, results)
            if result["state"] != wanted[scenario]:
                print(f"  Unexpected outcome: {result['state']} ({result['reason']}); ending here.")
                return
            if scenario == "cancel":
                go_home(client, q0)
            if scenario == "stop":
                det = device_status()["details"]
                print(f"  After STOP: session open {det['control_session']['open']}, "
                      f"latched {det['control_session']['latched']}, "
                      f"watchdog ok {det['control_session']['watchdog_ok']}")
                if not confirm("  The arm is stopped. Type YES to Clear errors and drive back: "):
                    print("  Left stopped. Clear errors in the panel when ready.")
                    return
                print(f"  Clear errors: HTTP {client.call('POST', '/reset', {})[0]}")
                go_home(client, q0)
    finally:
        cancel_open(client)
        heartbeat_stop.set()
        (out_dir / "summary.json").write_text(json.dumps(results, indent=1), encoding="utf-8")
        if client.token:
            client.call("POST", "/release", {})
            print("Claim released.")
            client.token = None
        if client.cookie:
            client.edge("POST", "/auth/logout", {})
            client.cookie = None
            print("Signed out.")
        print(f"Results in {out_dir}")


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
