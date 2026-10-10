# Joint-trajectory API (xArm5): client guide

Upload a time-stamped joint trajectory, then start it. The xArm service
plays it to the arm with ServoJ at 100 Hz on its own clock, so network
delay does not affect the motion. It was tested on the arm on 2026-10-09:
tracking within about 0.1° with a 10–15 ms lag. Design and test results
are in [`SERVOJ_TRAJECTORY_PLAN.md`](SERVOJ_TRAJECTORY_PLAN.md).

## Access

- **Base URL:** the xArm service, `http://sdl2-pc-03-cytation:8000` on the
  lab Tailnet, or through the lab edge.
- **Sign-in:** every call needs a signed-in identity. Use the lab session
  cookie (`ac_auth_session`, from the emailed one-time code: `POST
  /auth/request-code {email}`, then `POST /auth/verify-code {email,
  code}`), or an `X-Api-Key` for a machine principal. Codes are limited to
  5 per address per hour, so sign in once and reuse the session.
- **Claim:** moving needs the claim. Take it with `POST /control/claim
  {owner, session_id, ttl_s}`, send the returned token as `X-Claim-Token`
  on every call below, and keep it alive with `POST /control/heartbeat`
  every ~10 s. A session belongs to the claim that created it, and if the
  claim lapses the run slows to a stop.
- **Graph mode:** these are freehand routes, so they are refused while the
  arm's graph mode is STRICT (`409 graph_mode_strict`).

## Trajectory format

```json
{"t": 0.25, "joints_deg": [j1, j2, j3, j4, j5],
 "velocities_deg_s": [...], "accelerations_deg_s2": [...]}
```

- **`t`:** seconds from the trajectory start. It begins at 0, strictly
  increases, and points are at least 10 ms apart.
- **`joints_deg`:** degrees, J1..J5 base to wrist, exactly 5 values.
- **Velocities and accelerations:** optional, but recommended. With both
  given, the path is a quintic, smooth in acceleration. Give them on every
  point or on none.
- **Rest:** the first and last points must be at rest.
- **Start:** the first point must be within 0.5° of the arm's measured
  pose; a short lead-in closes the gap.
- **Limits:** see [Limits](#limits).
- **No collision checking.** Passing validation is not a collision-safety
  proof; the planner owns collision checking.

## Limits

`GET /control/freehand/trajectory/limits` returns these live; the speed
depends on the safety level chosen at Connect. They are checked on the interpolated path at the command rate, not just at
the points.

| | Value | Source |
|---|---|---|
| J1 | −360 to +360° | `src/settings/safety.yaml` (model 5) |
| J2 | −118 to +120° | same |
| J3 | −225 to +11° | same |
| J4 | −97 to +180° | same |
| J5 | −360 to +360° | same |
| Joint speed | 180°/s × the safety-level factor: LOW 1.0, MEDIUM 0.5, HIGH 0.25, EMERGENCY 0.1. MEDIUM, the default at Connect, gives **90°/s** | `max_joint_speed` in `safety.yaml` |
| Joint acceleration | 500 deg/s² | `max_joint_acc_deg_s2` in `src/settings/trajectory.yaml` |
| Duration | at most 600 s | `trajectory.yaml` |
| Points | at most 60 000 per trajectory, 5000 per chunk; at least 10 ms apart (one period at 100 Hz) | `trajectory.yaml` |
| Start | the first point within 0.5° of the measured pose | `trajectory.yaml` |
| Command rate | 100 Hz by default; a session may ask for 50–200 Hz | `trajectory.yaml` |

The controller enforces its own limits underneath. On 2026-10-10 it
reported 180°/s and 1000 mm/s, no TCP boundary, and joint ranges slightly
inside the table: J2 −117 to +116°, J3 −219 to +10°. Stay inside those.

Trajectories are joint-space only. For reference, the arm's other
(Cartesian) moves are limited to:
- x ±700, y ±700, z −250 to +700 mm;
- 1000 mm/s × the safety-level factor (500 mm/s at MEDIUM);
- an acceleration cap of 50 000 mm/s².

## Endpoints

All are under `/control/freehand/trajectory`.

| Call | Purpose |
|---|---|
| `GET /limits` | **Read first.** The limits in force right now: joint ranges, joint speed at the current safety level, acceleration, duration, points, rate, start tolerance and the rules. Needs sign-in but no claim |
| `POST /validate {points, rate_hz?}` | Check a whole trajectory against the arm and its measured pose. Moves nothing; needs no claim. 200 with the report, or 422 `trajectory_invalid` with `detail.report.errors` |
| `POST {rate_hz?}` | Open a session; 201 with `session_id`. One open session per arm (409 `session_open`) |
| `PUT` (or `POST`) `/{id}/chunks/{seq} {points, final}` | Upload chunk 0, 1, 2… The last one carries `"final": true`. Re-sending identical content is a harmless duplicate. Errors: 409 `chunk_conflict`, `chunk_out_of_order` (with `expected_seq`), `session_closed`; 422 with the report |
| `POST /{id}/start` | Start the complete trajectory. 412 if the arm is not ready or still, or in manual mode. 409 `motion_in_progress` or `stopped_during_start`. 422 if the arm moved away from the first point |
| `GET /{id}` | Status, for one-way sync. Poll at 10–20 Hz at most |
| `POST /{id}/cancel` | Constrained stop along the path, then mode 0 |
| `GET /{id}/log` | After the run: every command sent and every reported arm pose |
| `POST /control/stop` | Emergency stop (needs no claim). Afterwards, `POST /control/clear_errors` |

**Status fields:**
- **`state`:** one of `created`, `starting`, `running`, `stopping`,
  `completed`, `cancelled`, `stopped`, `failed`, `expired`.
- **`reason`:** why it stopped or failed.
- **`started_at_utc`:** the wall time of `t = 0`, after the lead-in.
- **`executed.t_exec`:** how far the trajectory has played, in its own
  time.
- **`measured`:** the latest joints the arm reported, and their age.
- **`servo`:** timing statistics.
- **`final`:** settle time and end error.

## Syncing a digital twin (one-way)

1. Read `GET /limits` and plan within them. Then validate, create, upload
   the whole trajectory as one or more chunks, and start.
2. Poll `GET /{id}`. Play the twin's copy at `executed.t_exec`; this needs
   no shared clock. Use `started_at_utc` only if both clocks are
   synchronised. Draw `measured.joints_deg` as the real arm.
3. When the state is terminal and `log_available` is true, fetch `GET
   /{id}/log` to compare plan and reality exactly.

While a run is active, the arm's other read and setup routes answer `409
trajectory_running`, so don't poll `/positions` during a run.
Commands that arrive after the start do not change the motion: this version
runs only complete trajectories.

## Minimal example (Python)

```python
import requests, time
s = requests.Session()                       # carries the signed-in cookie
base = "http://sdl2-pc-03-cytation:8000"
tok = s.post(f"{base}/control/claim", json={"owner": "astra", "session_id": "twin-1"}).json()["claim_token"]
h = {"X-Claim-Token": tok}
sid = s.post(f"{base}/control/freehand/trajectory", json={}, headers=h).json()["session_id"]
s.put(f"{base}/control/freehand/trajectory/{sid}/chunks/0", json={"points": points, "final": True}, headers=h)
s.post(f"{base}/control/freehand/trajectory/{sid}/start", headers=h).raise_for_status()
while (st := s.get(f"{base}/control/freehand/trajectory/{sid}").json())["state"] in ("starting", "running", "stopping"):
    twin.show(t=st["executed"]["t_exec"], real=st["measured"])
    time.sleep(0.1)
s.post(f"{base}/control/release", headers=h)
```
