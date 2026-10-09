#!/usr/bin/env python3
"""Stage 3 of SERVOJ_TRAJECTORY_PLAN.md: the joint-trajectory HTTP API on the arm.

THIS MOVES THE ARM. Run it only with someone at the robot, the E-stop in
reach, the arm at a clear graph node (home), and the arm connected in the
panel. It talks to the xarm service on this PC (http://127.0.0.1:8000)
exactly as a client such as the digital twin would:

1. sign in with an emailed one-time code (kept in memory only; signed out
   at the end), take the claim, and keep it alive with heartbeats;
2. validate the trajectory: the same small J1 / J5 wiggle around the
   current pose as the stage 0b probe;
3. after a typed yes, three runs through the API:
   - full: create, upload, start, poll to the end. While running, check
     that /positions is refused (409 trajectory_running) and that
     /status shows the session;
   - cancel at --stop-at (25 %, mid-swing), then drive home at 10 deg/s;
   - STOP (POST /control/stop) at --stop-at, then after another typed
     yes: Clear errors and drive home.
4. fetch each session's log and analyse it like the probe; release the
   claim and sign out.

Everything printed is also written to logs/trajectory_http_check/.
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
from servo_mode_probe import _Tee, analyse, confirm, wiggle_points  # noqa: E402

BASE = "http://127.0.0.1:8000"


class Client:
    def __init__(self, base):
        self.base = base
        self.cookie = None
        self.token = None
        self.open_session = None
        self.last_headers = {}

    def call(self, method, path, body=None, timeout=30):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method=method)
        req.add_header("Content-Type", "application/json")
        if self.cookie:
            req.add_header("Cookie", self.cookie)
        if self.token:
            req.add_header("X-Claim-Token", self.token)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read()
                set_cookie = resp.headers.get("Set-Cookie")
                status = resp.status
                self.last_headers = dict(resp.headers)
        except urllib.error.HTTPError as exc:
            raw, set_cookie, status = exc.read(), None, exc.code
            self.last_headers = dict(exc.headers or {})
        try:
            payload = json.loads(raw) if raw else None
        except ValueError:
            payload = raw.decode(errors="replace")
        return status, payload, set_cookie


def expect(status, payload, wanted, what):
    if status != wanted:
        raise SystemExit(f"{what}: expected HTTP {wanted}, got {status}: {json.dumps(payload)[:600]}")
    return payload


def wait_terminal(client, sid, act=None, act_at=None, checks=None, timeout=60):
    """Poll the session at 10 Hz until it ends. ``act()`` runs once when
    t_exec passes ``act_at`` seconds; ``checks()`` once while running."""
    deadline = time.time() + timeout
    acted = checked = False
    body = None
    while time.time() < deadline:
        _, body, _ = client.call("GET", f"/control/freehand/trajectory/{sid}")
        state = body.get("state")
        if state in ("running", "stopping"):
            if checks and not checked and body["executed"]["t_exec"] > 0.5:
                checked = True
                checks()
            if act and not acted and act_at is not None and body["executed"]["t_exec"] >= act_at:
                acted = True
                act()
        if state in ("completed", "cancelled", "stopped", "failed", "expired") and body.get("log_available"):
            return body
        time.sleep(0.1)
    raise SystemExit(f"session {sid} did not end in {timeout} s; last status {json.dumps(body)[:400]}")


def cancel_open(client):
    """Never leave a session open: it would block the next client."""
    if client.open_session and client.token:
        status_code, _, _ = client.call("POST", f"/control/freehand/trajectory/{client.open_session}/cancel")
        print(f"  Cancelled the unfinished session {client.open_session}: HTTP {status_code}")
    client.open_session = None


def go_home(client, q0):
    status, payload, _ = client.call("POST", "/control/freehand/joints",
                                     {"angles": q0, "speed": 10, "wait": True}, timeout=60)
    print(f"  Return home (mode 0 joint move, 10 deg/s): HTTP {status}")
    if status != 200:
        raise SystemExit(f"return home failed: {json.dumps(payload)[:400]}")
    time.sleep(1.0)   # let the arm settle and its reported state catch up


def one_run(client, points, q0, scenario, rate, stop_at, out_dir, results):
    print(f"\nRun: ServoJ at {rate:g} Hz via the API, {scenario}")
    session = expect(*client.call("POST", "/control/freehand/trajectory", {"rate_hz": rate})[:2], 201, "create")
    sid = session["session_id"]
    client.open_session = sid     # cancelled in main's finally if this run gives up
    expect(*client.call("PUT", f"/control/freehand/trajectory/{sid}/chunks/0",
                        {"points": points, "final": True}, timeout=60)[:2], 200, "upload")
    observed = {}

    def checks():
        status, payload, _ = client.call("GET", "/positions")
        observed["positions_during_run"] = (status, (payload or {}).get("detail", {}).get("error")
                                            if isinstance(payload, dict) else None)
        _, st, _ = client.call("GET", "/status")
        traj = ((st or {}).get("details") or {}).get("trajectory") or {}
        observed["status_trajectory_state"] = (traj.get("session") or {}).get("state")
        observed["status_activity"] = (st or {}).get("activity")

    act = None
    if scenario == "cancel":
        def act():
            status, _, _ = client.call("POST", f"/control/freehand/trajectory/{sid}/cancel")
            observed["cancel_http"] = status
    elif scenario == "stop":
        def act():
            status, _, _ = client.call("POST", "/control/stop")
            observed["stop_http"] = status
    # The service's cached arm state updates at 5 Hz, so right after a move
    # it can still read "moving": retry the start briefly on arm_not_idle.
    for attempt in range(6):
        status_code, started, _ = client.call("POST", f"/control/freehand/trajectory/{sid}/start", timeout=60)
        detail = started.get("detail") if isinstance(started, dict) else None
        if status_code == 412 and isinstance(detail, dict) and detail.get("error") == "arm_not_idle":
            time.sleep(0.5)
            continue
        break
    started = expect(status_code, started, 200, "start")
    duration = started["executed"]["duration_s"]
    end = wait_terminal(client, sid, act=act, act_at=stop_at * duration if act else None, checks=checks,
                        timeout=duration + 30)
    client.open_session = None
    _, record, _ = client.call("GET", f"/control/freehand/trajectory/{sid}/log", timeout=60)
    (out_dir / f"{sid}.json").write_text(json.dumps(record), encoding="utf-8")
    after_status, _, _ = client.call("GET", "/positions")
    result = {
        "scenario": scenario, "rate_hz": rate, "session_id": sid,
        "state": end["state"], "reason": end["reason"],
        "started_at_utc": end["started_at_utc"], "t_exec": end["executed"]["t_exec"],
        "servo": end["servo"], "final": end["final"], "observed": observed,
        "positions_after_run_http": after_status,
        "analysis": analyse(points, record, 5) if isinstance(record, dict) else None,
    }
    results.append(result)
    print(json.dumps({k: result[k] for k in ("scenario", "state", "reason", "t_exec", "observed",
                                             "positions_after_run_http")}))
    servo = end["servo"] or {}
    print(f"    period {servo.get('period_ms')}\n    lateness {servo.get('lateness_ms')}\n"
          f"    round trip {servo.get('round_trip_ms')}\n    late ticks {servo.get('late_ticks')}, "
          f"sdk errors {servo.get('sdk_errors')}, stop rate {servo.get('stop_rate_per_s2')}")
    print(f"    final {json.dumps(end['final'])}")
    print(f"    analysis {json.dumps(result['analysis'])}")
    return result


def main():
    os.makedirs(os.path.join("logs", "trajectory_http_check"), exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    sys.stdout = _Tee(sys.stdout, os.path.join("logs", "trajectory_http_check", f"console-{stamp}.txt"))
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--email", required=True, help="your sign-in email (a one-time code is sent)")
    parser.add_argument("--no-login", action="store_true",
                        help="offline tests only: skip sign-in (a real service refuses with 401)")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--stop-at", type=float, default=0.25)
    parser.add_argument("--cycles", type=int, default=2, help="wiggle cycles per run (2 = 7.5 s)")
    parser.add_argument("--scenarios", default="full,cancel,stop",
                        help="comma-separated subset of full,cancel,stop, run in that order")
    parser.add_argument("--base", default=BASE)
    parser.add_argument("--auth-base", default="http://100.64.254.6",
                        help="the lab's shared sign-in service (edge); codes are requested there "
                             "with POST /auth/login")
    args = parser.parse_args()
    client = Client(args.base)

    _, status, _ = client.call("GET", "/status")
    det = (status or {}).get("details") or {}
    print(f"Service: {status.get('equipment_status')}, activity {status.get('activity')}, "
          f"claimed by {det.get('claimed_by')}, graph mode {(det.get('motion_graph') or {}).get('graph_mode')}")
    if status.get("equipment_status") not in ("ready",):
        raise SystemExit("The arm is not connected and ready in the service: press Connect in the panel first.")

    if not args.no_login:
        # The xarm service's own /auth/request-code forwards to a path the
        # shared sign-in service no longer has (404), so ask the shared
        # service directly. Verify through the xarm service; if it cannot
        # (a different sign-in backend), verify at the shared service.
        auth = Client(args.auth_base)
        print(f"Sending a one-time sign-in code to {args.email} ...")
        # Through the xarm service first (its route forwards to the shared
        # service's /auth/login since 2026-10-09); straight to the shared
        # service only if this is an older xarm service that answers 404.
        status_code, payload, _ = client.call("POST", "/auth/request-code", {"email": args.email})
        rate_source = client
        if status_code == 404:
            status_code, payload, _ = auth.call("POST", "/auth/login", {"email": args.email})
            rate_source = auth
        else:
            print("  (requested through the xarm service)")
        if status_code == 429:
            retry = rate_source.last_headers.get("Retry-After")
            when = (time.strftime("%H:%M:%S UTC", time.gmtime(time.time() + int(retry)))
                    if retry and str(retry).isdigit() else "later")
            raise SystemExit(f"the sign-in service is rate-limiting codes for this address; "
                             f"the next one is allowed at about {when}")
        if status_code not in (200, 202):
            raise SystemExit(f"request code: HTTP {status_code}: {json.dumps(payload)[:400]}")
        code = input("Type the code from the email: ").strip()
        status_code, payload, set_cookie = client.call("POST", "/auth/verify-code",
                                                       {"email": args.email, "code": code})
        if status_code != 200 or not set_cookie:
            print(f"  The xarm service did not accept the code (HTTP {status_code}); "
                  "verifying at the shared sign-in service.")
            status_code, payload, set_cookie = auth.call("POST", "/auth/verify-code",
                                                         {"email": args.email, "code": code})
            expect(status_code, payload, 200, "verify code")
        if not set_cookie:
            raise SystemExit("signed in, but no session cookie came back")
        client.cookie = set_cookie.split(";", 1)[0]
        me = expect(*client.call("GET", "/auth/me")[:2], 200, "auth/me")
        print(f"Signed in as {me.get('identity')}.")

    heartbeat_stop = threading.Event()
    out_dir = Path("logs") / "trajectory_http_check" / stamp
    out_dir.mkdir(parents=True, exist_ok=True)
    results = []
    try:
        claim = expect(*client.call("POST", "/control/claim",
                                    {"owner": args.email, "session_id": f"trajectory-check-{uuid.uuid4().hex[:8]}",
                                     "ttl_s": 60})[:2], 200, "claim")
        client.token = claim["claim_token"]
        print("Claim taken.")

        def beat():
            while not heartbeat_stop.wait(15):
                client.call("POST", "/control/heartbeat", {})

        threading.Thread(target=beat, daemon=True).start()

        positions = expect(*client.call("GET", "/positions")[:2], 200, "positions")
        joints = positions.get("joints") or positions.get("joint_angles") or positions
        q0 = [float(v) for v in (joints.get("angles") if isinstance(joints, dict) else joints)[:5]]
        print(f"Current joints {[round(v, 3) for v in q0]}")
        points = wiggle_points(q0, 5.0, 10.0, 20.0, args.cycles)
        report = expect(*client.call("POST", "/control/freehand/trajectory/validate",
                                     {"points": points, "rate_hz": args.rate}, timeout=60)[:2], 200, "validate")
        print(f"Validated: {report['summary']['points']} points, {report['summary']['duration_s']} s, "
              f"{report['summary']['interpolation']}, start error {report['start_state'].get('joint_error_deg')} deg.")
        scenarios = [x.strip() for x in args.scenarios.split(",") if x.strip()]
        print(f"Runs: {', '.join(scenarios)} (cancel and STOP at {args.stop_at:.0%}). "
              "Each is J1 +/-5, J5 +/-10 deg at 20 deg/s.")
        if not confirm("The arm will move. Someone is at the robot with the E-stop in reach? Type YES: "):
            print("Not confirmed; nothing moved.")
            return

        def attempt(scenario, wanted):
            # One sign-in covers retries: a refused or unexpected run is
            # cleaned up and, after a typed yes, tried again.
            while True:
                try:
                    result = one_run(client, points, q0, scenario, args.rate, args.stop_at, out_dir, results)
                    if result["state"] == wanted:
                        return True
                    print(f"  Unexpected outcome: {result['state']} ({result['reason']}).")
                except SystemExit as exc:
                    print(f"  The run did not complete: {exc.code}")
                cancel_open(client)
                if not confirm(f"  Fix the cause (e.g. Clear errors in the panel), then type YES to retry "
                               f"the {scenario} run, or anything else to end: "):
                    return False

        if "full" in scenarios and not attempt("full", "completed"):
            return
        if "cancel" in scenarios:
            if not attempt("cancel", "cancelled"):
                return
            go_home(client, q0)
        if "stop" not in scenarios:
            return
        if not attempt("stop", "stopped"):
            return
        print("  The arm is stopped (planned STOP).")
        if not confirm("  Type YES to Clear errors and drive home: "):
            print("  Left stopped. Clear errors in the panel when ready.")
            return
        status_code, _, _ = client.call("POST", "/control/clear_errors")
        print(f"  Clear errors: HTTP {status_code}")
        time.sleep(1.0)
        go_home(client, q0)
    finally:
        cancel_open(client)
        heartbeat_stop.set()
        (out_dir / "summary.json").write_text(json.dumps(results, indent=1), encoding="utf-8")
        if client.token:
            client.call("POST", "/control/release", {})
            print("Claim released.")
            client.token = None
        if client.cookie:
            client.call("POST", "/auth/logout", {})
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
